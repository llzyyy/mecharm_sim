import json
import time
import uuid
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mecharm_sim.classification_config import (
    SUPPORTED_CLASSES,
    add_vectors,
    load_classification_config,
    objects_by_grid,
)


class TaskState(str, Enum):
    WAIT_DETECTION = "WAIT_DETECTION"
    SELECT_TARGET = "SELECT_TARGET"
    PICK = "PICK"
    PLACE = "PLACE"
    RETURN = "RETURN"
    NEXT_TARGET = "NEXT_TARGET"
    FINISHED = "FINISHED"


class ClassificationManager(Node):
    """Select detected targets and route classes to configured bins."""

    def __init__(self) -> None:
        super().__init__("classification_manager")
        self.declare_parameter("config_file", "")
        config_path = self._resolve_config_path(
            str(self.get_parameter("config_file").value)
        )
        self.config = load_classification_config(config_path)
        manager_config = self.config.get("manager", {})

        self.minimum_confidence = float(manager_config.get("minimum_confidence", 0.5))
        self.detection_timeout_sec = float(manager_config.get("detection_timeout_sec", 5.0))
        self.command_timeout_sec = float(manager_config.get("command_timeout_sec", 180.0))
        self.max_attempts = int(manager_config.get("max_attempts_per_target", 1))
        self.target_order = [str(value) for value in manager_config["target_order"]]
        self.result_log = Path(
            str(manager_config.get("result_log", "~/.ros/mecharm_sim/classification_manager.jsonl"))
        ).expanduser()

        self.expected_by_grid = objects_by_grid(self.config)
        self.state = TaskState.WAIT_DETECTION
        self.detections: Optional[Dict[str, Dict[str, Any]]] = None
        self.last_detection_time = 0.0
        self.processed_grids = set()
        self.successful_grids = set()
        self.attempts: Dict[str, int] = {}
        self.bin_counts = {bin_name: 0 for bin_name in self.config["bins"]}
        self.current: Optional[Dict[str, Any]] = None
        self.command_started_at = 0.0
        self.finished_logged = False

        topics = self.config["topics"]
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, str(topics["detections"]), self._on_detections, detection_qos
        )
        self.create_subscription(String, str(topics["result"]), self._on_result, 10)
        self.command_publisher = self.create_publisher(
            String, str(topics["command"]), 10
        )
        update_period = float(manager_config.get("update_period_sec", 0.2))
        self.timer = self.create_timer(update_period, self._step)
        self.get_logger().info(f"Classification manager state={self.state.value}")

    def _on_detections(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            raw_detections = payload["detections"]
            if not isinstance(raw_detections, list):
                raise ValueError("detections must be a list")
            detections: Dict[str, Dict[str, Any]] = {}
            for detection in raw_detections:
                if not isinstance(detection, dict):
                    raise ValueError("each detection must be an object")
                grid_id = str(detection.get("grid_id", ""))
                if not grid_id:
                    self.get_logger().warning("Ignoring detection without grid_id")
                    continue
                if grid_id in detections:
                    raise ValueError(f"duplicate detection for {grid_id}")
                detections[grid_id] = detection
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Invalid Fake Vision JSON: {exc}")
            return

        self.detections = detections
        self.last_detection_time = time.monotonic()
        if self.state == TaskState.WAIT_DETECTION:
            self._transition(TaskState.SELECT_TARGET)

    def _on_result(self, message: String) -> None:
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid controller result JSON: {exc}")
            return
        if self.current is None or result.get("request_id") != self.current["request_id"]:
            self.get_logger().warning("Ignoring result for an inactive classification request")
            return

        status = str(result.get("status", ""))
        phase = str(result.get("phase", ""))
        if status == "in_progress":
            phase_states = {
                "pick": TaskState.PICK,
                "place": TaskState.PLACE,
                "return": TaskState.RETURN,
            }
            if phase in phase_states:
                self._transition(phase_states[phase])
            return

        grid_id = str(self.current["grid_id"])
        if status == "success":
            self.processed_grids.add(grid_id)
            self.successful_grids.add(grid_id)
            self.bin_counts[self.current["destination_bin"]] += 1
            self._record("success", "completed", self.current)
            self.get_logger().info(
                f"Completed {self.current['model_name']} from {grid_id} -> "
                f"{self.current['destination_bin']}"
            )
        else:
            error_code = str(result.get("error_code") or "pick_failed")
            reason = str(result.get("reason") or "controller reported failure")
            if self.attempts.get(grid_id, 0) >= self.max_attempts:
                self.processed_grids.add(grid_id)
            self._record(error_code, reason, self.current)
            self.get_logger().error(f"{error_code}: {reason}")

        self.current = None
        self._transition(TaskState.NEXT_TARGET)

    def _step(self) -> None:
        if self.state == TaskState.WAIT_DETECTION:
            return
        if self.state == TaskState.SELECT_TARGET:
            self._select_target()
            return
        if self.state in (TaskState.PICK, TaskState.PLACE, TaskState.RETURN):
            if time.monotonic() - self.command_started_at > self.command_timeout_sec:
                assert self.current is not None
                grid_id = str(self.current["grid_id"])
                self.processed_grids.add(grid_id)
                self._record("pick_failed", "controller command timed out", self.current)
                self.get_logger().error(
                    f"pick_failed: request {self.current['request_id']} timed out"
                )
                self.current = None
                self._transition(TaskState.NEXT_TARGET)
            return
        if self.state == TaskState.NEXT_TARGET:
            self._transition(TaskState.SELECT_TARGET)
            return
        if self.state == TaskState.FINISHED and not self.finished_logged:
            self.finished_logged = True
            self.get_logger().info(
                f"Classification finished: {len(self.successful_grids)} successful, "
                f"{len(self.processed_grids) - len(self.successful_grids)} skipped/failed"
            )

    def _select_target(self) -> None:
        if self.detections is None:
            self._transition(TaskState.WAIT_DETECTION)
            return
        if time.monotonic() - self.last_detection_time > self.detection_timeout_sec:
            self.get_logger().warning("Fake Vision detections are stale; waiting for a fresh frame")
            self._transition(TaskState.WAIT_DETECTION)
            return

        for grid_id in self.target_order:
            if grid_id in self.processed_grids:
                continue
            detection = self.detections.get(grid_id)
            if detection is None or not detection.get("model_name"):
                self._skip_target(grid_id, "empty", "no object detected in configured grid")
                continue
            if not self._validate_detection(grid_id, detection):
                continue
            self._dispatch(grid_id, detection)
            return
        self._transition(TaskState.FINISHED)

    def _validate_detection(self, grid_id: str, detection: Dict[str, Any]) -> bool:
        expected = self.expected_by_grid.get(grid_id)
        class_name = str(detection.get("class_name", ""))
        model_name = str(detection.get("model_name", ""))
        try:
            confidence = float(detection.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if expected is None:
            self._skip_target(grid_id, "unknown", "grid is not present in task config")
            return False
        if class_name not in SUPPORTED_CLASSES:
            self._skip_target(grid_id, "unknown", f"unsupported class '{class_name}'")
            return False
        if model_name != expected["model_name"] or class_name != expected["class_name"]:
            self._skip_target(
                grid_id,
                "unknown",
                f"detection {model_name}/{class_name} does not match configured object",
            )
            return False
        if confidence < self.minimum_confidence:
            self._skip_target(
                grid_id,
                "unknown",
                f"confidence {confidence:.3f} is below {self.minimum_confidence:.3f}",
            )
            return False
        if not bool(self.config["grids"][grid_id].get("reachable", True)):
            self._skip_target(grid_id, "unreachable", "grid is marked unreachable")
            return False
        return True

    def _dispatch(self, grid_id: str, detection: Dict[str, Any]) -> None:
        class_name = str(detection["class_name"])
        bin_name = SUPPORTED_CLASSES[class_name]
        destination = self.config["bins"][bin_name]
        slot_index = self.bin_counts[bin_name]
        offsets = destination.get("drop_offsets", [[0.0, 0.0, 0.0]])
        if slot_index >= len(offsets):
            self._skip_target(grid_id, "unreachable", f"no free drop slot in {bin_name}")
            return

        expected = self.expected_by_grid[grid_id]
        command = {
            "schema_version": 1,
            "request_id": uuid.uuid4().hex,
            "model_name": str(detection["model_name"]),
            "class_name": class_name,
            "grid_id": grid_id,
            "pick_position": [
                float(value) for value in self.config["grids"][grid_id]["world_position"]
            ],
            "destination_bin": bin_name,
            "destination_position": add_vectors(destination["world_position"], offsets[slot_index]),
            "attach_topic": str(expected["attach_topic"]),
            "detach_topic": str(expected["detach_topic"]),
            "table_attach_topic": str(expected["table_attach_topic"]),
            "table_detach_topic": str(expected["table_detach_topic"]),
        }
        self.attempts[grid_id] = self.attempts.get(grid_id, 0) + 1
        self.current = command
        self.command_started_at = time.monotonic()
        message = String()
        message.data = json.dumps(command, separators=(",", ":"), sort_keys=True)
        self.command_publisher.publish(message)
        self._transition(TaskState.PICK)
        self.get_logger().info(
            f"Dispatched {command['model_name']} at {grid_id} to {bin_name} slot {slot_index + 1}"
        )

    def _skip_target(self, grid_id: str, error_code: str, reason: str) -> None:
        self.processed_grids.add(grid_id)
        details = {"grid_id": grid_id, "model_name": "", "class_name": ""}
        self._record(error_code, reason, details)
        self.get_logger().warning(f"{error_code}: {grid_id}: {reason}")

    def _transition(self, new_state: TaskState) -> None:
        if new_state != self.state:
            self.get_logger().info(f"State {self.state.value} -> {new_state.value}")
            self.state = new_state

    def _record(self, status: str, reason: str, details: Dict[str, Any]) -> None:
        record = {
            "time": time.time(),
            "status": status,
            "reason": reason,
            "grid_id": details.get("grid_id", ""),
            "model_name": details.get("model_name", ""),
            "class_name": details.get("class_name", ""),
            "destination_bin": details.get("destination_bin", ""),
            "request_id": details.get("request_id", ""),
        }
        try:
            self.result_log.parent.mkdir(parents=True, exist_ok=True)
            with self.result_log.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, sort_keys=True) + "\n")
        except OSError as exc:
            self.get_logger().error(f"Unable to write manager result log: {exc}")

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
    node = ClassificationManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
