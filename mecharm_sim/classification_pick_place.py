import json
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

from mecharm_sim.classification_config import (
    SUPPORTED_CLASSES,
    add_vectors,
    load_classification_config,
    objects_by_grid,
)
from mecharm_sim.pick_place import PickPlaceNode


class CommandError(RuntimeError):
    def __init__(self, error_code: str, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


class ClassificationPickPlaceNode(PickPlaceNode):
    """Parameterised multi-object adapter around the proven pick-ball controller."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("config_file", "")
        self.declare_parameter("scene.target_start_tolerance_m", 0.03)
        config_path = self._resolve_config_path(
            str(self.get_parameter("config_file").value)
        )
        self.config = load_classification_config(config_path)
        self.expected_by_grid = objects_by_grid(self.config)
        self.target_start_tolerance_m = float(
            self.get_parameter("scene.target_start_tolerance_m").value
        )
        self.reset_ball_each_attempt = False

        topics = self.config["topics"]
        self.command_queue: Deque[Dict[str, Any]] = deque()
        self.seen_request_ids = set()
        self.result_publisher = self.create_publisher(
            String, str(topics["result"]), 10
        )
        self.create_subscription(
            String, str(topics["command"]), self._on_command, 10
        )
        self.command_counter = 0
        self.ready = False

    def initialize(self) -> bool:
        self._prepare_log()
        if not self._wait_for_servers():
            self.get_logger().error("Classification controller dependencies are not ready")
            return False

        # Gazebo DetachableJoint instances start attached. Detach every gripper
        # joint once, while leaving each object's table lock attached.
        try:
            for target in self.config["objects"]:
                self._send_empty_transport_command(
                    str(target["detach_topic"]),
                    "initial detach",
                    str(target["model_name"]),
                )
            self._move_gripper(
                "open",
                self.gripper_open,
                timeout_sec=self.initialization_action_timeout_sec,
            )
            self._move_arm("home")
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize classification controller: {exc}")
            return False

        self.ready = True
        self.get_logger().info("Classification pick/place controller is ready")
        return True

    def _on_command(self, message: String) -> None:
        try:
            command = json.loads(message.data)
            self._validate_command(command)
        except (CommandError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            request_id = ""
            try:
                request_id = str(json.loads(message.data).get("request_id", ""))
            except (AttributeError, json.JSONDecodeError):
                pass
            error_code = exc.error_code if isinstance(exc, CommandError) else "invalid_command"
            self._publish_status(request_id, "failed", "validation", error_code, str(exc))
            self.get_logger().error(f"Rejected classification command: {exc}")
            return

        request_id = str(command["request_id"])
        if request_id in self.seen_request_ids:
            self.get_logger().warning(f"Ignoring duplicate request {request_id}")
            return
        self.seen_request_ids.add(request_id)
        self.command_queue.append(command)
        self.get_logger().info(
            f"Queued request {request_id}: {command['model_name']} -> {command['destination_bin']}"
        )

    def _validate_command(self, command: Dict[str, Any]) -> None:
        required = {
            "request_id",
            "model_name",
            "class_name",
            "grid_id",
            "pick_position",
            "destination_bin",
            "destination_position",
            "attach_topic",
            "detach_topic",
            "table_attach_topic",
            "table_detach_topic",
        }
        missing = required.difference(command)
        if missing:
            raise CommandError("invalid_command", f"missing fields: {sorted(missing)}")
        if not str(command["request_id"]):
            raise CommandError("invalid_command", "request_id must not be empty")

        grid_id = str(command["grid_id"])
        expected = self.expected_by_grid.get(grid_id)
        if expected is None:
            raise CommandError("unknown", f"unknown grid_id '{grid_id}'")
        if not bool(self.config["grids"][grid_id].get("reachable", True)):
            raise CommandError("unreachable", f"{grid_id} is marked unreachable")

        exact_fields = (
            "model_name",
            "class_name",
            "attach_topic",
            "detach_topic",
            "table_attach_topic",
            "table_detach_topic",
        )
        for field in exact_fields:
            if str(command[field]) != str(expected[field]):
                raise CommandError(
                    "invalid_command",
                    f"{field} does not match configured target for {grid_id}",
                )

        class_name = str(command["class_name"])
        expected_bin = SUPPORTED_CLASSES.get(class_name)
        if expected_bin is None:
            raise CommandError("unknown", f"unsupported class '{class_name}'")
        if str(command["destination_bin"]) != expected_bin:
            raise CommandError(
                "invalid_command",
                f"class '{class_name}' must be routed to {expected_bin}",
            )

        pick_position = self._vector3(command["pick_position"], "pick_position")
        configured_pick = self.config["grids"][grid_id]["world_position"]
        if self._distance(pick_position, configured_pick) > 1e-6:
            raise CommandError("invalid_command", "pick_position does not match grid config")

        destination_position = self._vector3(
            command["destination_position"], "destination_position"
        )
        destination = self.config["bins"][expected_bin]
        allowed_positions = [
            add_vectors(destination["world_position"], offset)
            for offset in destination.get("drop_offsets", [[0.0, 0.0, 0.0]])
        ]
        if not any(
            self._distance(destination_position, allowed) <= 1e-6
            for allowed in allowed_positions
        ):
            raise CommandError(
                "invalid_command",
                f"destination_position is not a configured slot in {expected_bin}",
            )

    @staticmethod
    def _vector3(value: Any, label: str) -> List[float]:
        if not isinstance(value, list) or len(value) != 3:
            raise CommandError("invalid_command", f"{label} must have three elements")
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError) as exc:
            raise CommandError("invalid_command", f"{label} must contain numbers") from exc

    def process_next_command(self) -> None:
        if not self.ready or not self.command_queue:
            return
        command = self.command_queue.popleft()
        request_id = str(command["request_id"])
        self.command_counter += 1
        phase = "pick"
        try:
            self._configure_target(command)
            self._publish_status(request_id, "in_progress", "pick")
            self.pick_object(command)
            phase = "place"
            self._publish_status(request_id, "in_progress", "place")
            self.place_object(command, command["destination_position"])
            phase = "return"
            self._publish_status(request_id, "in_progress", "return")
            self.return_home()
        except Exception as exc:
            error_code = self._error_code(phase, exc)
            self.get_logger().error(
                f"Request {request_id} failed during {phase}: {exc}"
            )
            self._try_abort_to_safe()
            self._append_log(self.command_counter, "failed", f"{error_code}: {exc}")
            self._publish_status(
                request_id, "failed", phase, error_code, str(exc)
            )
            return

        self._append_log(self.command_counter, "success", "completed")
        self._publish_status(request_id, "success", "complete", "", "completed")

    def _configure_target(self, command: Dict[str, Any]) -> None:
        self.active_command = command
        self.target_model = str(command["model_name"])
        self.ball_position = self._vector3(command["pick_position"], "pick_position")
        self.place_position = self._vector3(
            command["destination_position"], "destination_position"
        )
        self.attach_topic = str(command["attach_topic"])
        self.detach_topic = str(command["detach_topic"])
        self.table_lock_attach_topic = str(command["table_attach_topic"])
        self.table_lock_detach_topic = str(command["table_detach_topic"])

    def pick_object(self, target: Dict[str, Any]) -> None:
        """Pick the current model using a configured fixed grid coordinate."""
        configured_position = self._vector3(target["pick_position"], "pick_position")
        grid_id = str(target["grid_id"])
        grasp_target_offset = self._vector3(
            self.config["grids"][grid_id].get(
                "grasp_target_offset", [0.0, 0.0, 0.0]
            ),
            f"{grid_id}.grasp_target_offset",
        )
        calibrated_joint_target = self.config["grids"][grid_id].get(
            "grasp_joint_target"
        )
        if calibrated_joint_target is not None:
            calibrated_joint_target = [float(value) for value in calibrated_joint_target]
            if len(calibrated_joint_target) != len(self.joints):
                raise CommandError(
                    "invalid_command",
                    f"{grid_id}.grasp_joint_target must contain {len(self.joints)} values",
                )
        measured_position = self._require_entity_position(self.target_model)
        start_error = self._distance(measured_position, configured_position)
        if start_error > self.target_start_tolerance_m:
            raise CommandError(
                "unreachable",
                f"{self.target_model} is {start_error:.3f} m from configured "
                f"{target['grid_id']} center",
            )

        self._set_detachable_joint(False)
        self._move_gripper("open", self.gripper_open)
        if self.use_moveit:
            assert self.moveit_planner is not None
            above = add_vectors(
                self._offset_z(configured_position, self.approach_height_m),
                grasp_target_offset,
            )
            grasp = add_vectors(
                self._offset_z(
                    configured_position, self.grasp_center_height_offset_m
                ),
                grasp_target_offset,
            )
            self.moveit_planner.lock_tool_pose(
                self._estimate_gripper_base_orientation(self.poses["a_pick"]),
                self.poses["a_pick"],
            )
            self._move_pose(f"{self.target_model}_approach", above)
            self._move_pose_precisely(
                f"{self.target_model}_grasp",
                grasp,
                calibrated_joint_target=calibrated_joint_target,
            )
            # The per-grid offset calibrates the visible finger centre, which
            # is not identical to the nominal MoveIt target point at every
            # wrist angle.  Verify that the arm reached that calibrated point.
            self._ensure_grasp_alignment(
                add_vectors(measured_position, grasp_target_offset)
            )
        else:
            # Joint-space fallback is retained from pick-ball and requires
            # per-grid calibration before it is suitable for all six grids.
            self._move_arm("a_above", safe_move=True)
            self._move_arm("a_pick")

        self._move_gripper("closed", self.gripper_closed)
        self._set_detachable_joint(True)
        self._set_table_ball_lock(False)
        if self.use_moveit:
            self._move_pose(
                f"{self.target_model}_lift",
                above,
            )
        else:
            self._move_arm("a_above", safe_move=True)

    def place_object(self, target: Dict[str, Any], destination: Any) -> None:
        """Place the attached current model at a manager-selected bin slot."""
        del target
        self.place_position = self._vector3(destination, "destination_position")
        if self.use_moveit:
            assert self.moveit_planner is not None
            above = self._offset_z(self.place_position, self.approach_height_m)
            place = self._offset_z(
                self.place_position, self.place_center_height_offset_m
            )
            self.moveit_planner.lock_tool_pose(
                self._estimate_gripper_base_orientation(self.poses["b_place"]),
                self.poses["b_place"],
            )
            self._move_pose(f"{self.target_model}_bin_approach", above)
            self._move_pose(f"{self.target_model}_release", place)
            self._move_ball_to_release_target()
        else:
            self._move_arm("b_above", safe_move=True)
            self._move_arm("b_place")

        self._move_gripper("open", self.gripper_open)
        self._set_detachable_joint(False)
        self._ensure_ball_at_place()
        if self.use_moveit:
            self._move_pose(
                f"{self.target_model}_bin_lift",
                self._offset_z(self.place_position, self.approach_height_m),
            )
        else:
            self._move_arm("b_above", safe_move=True)

    def return_home(self) -> None:
        if self.moveit_planner is not None:
            self.moveit_planner.clear_orientation_constraint()
        self._move_arm("home", safe_move=True)

    def _publish_status(
        self,
        request_id: str,
        status: str,
        phase: str,
        error_code: str = "",
        reason: str = "",
    ) -> None:
        active = getattr(self, "active_command", {})
        payload = {
            "schema_version": 1,
            "request_id": request_id,
            "status": status,
            "phase": phase,
            "error_code": error_code,
            "reason": reason,
            "model_name": active.get("model_name", ""),
            "grid_id": active.get("grid_id", ""),
            "destination_bin": active.get("destination_bin", ""),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.result_publisher.publish(message)

    @staticmethod
    def _error_code(phase: str, error: Exception) -> str:
        if isinstance(error, CommandError):
            return error.error_code
        text = str(error).lower()
        if "cartesian fk error" in text or "grasp is not centered" in text:
            return "alignment_failed"
        if "failed to plan" in text or "unable to reach" in text:
            return "unreachable"
        return {
            "pick": "pick_failed",
            "place": "place_failed",
            "return": "return_failed",
        }.get(phase, "pick_failed")

    @staticmethod
    def _resolve_config_path(parameter_value: str) -> str:
        if parameter_value:
            return str(Path(parameter_value).expanduser())
        return str(
            Path(get_package_share_directory("mecharm_sim"))
            / "config"
            / "classification_fake.yaml"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClassificationPickPlaceNode()
    exit_code = 0
    try:
        if not node.initialize():
            exit_code = 1
        while rclpy.ok() and exit_code == 0:
            # Process callbacks first, then execute queued motion outside a ROS
            # subscription callback. The inherited controller can safely use
            # spin_until_future_complete and spin_once during motion this way.
            rclpy.spin_once(node, timeout_sec=0.1)
            node.process_next_command()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
