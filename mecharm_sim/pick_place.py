import csv
import json
import math
import subprocess
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


MOVEIT_SUCCESS = 1


class MoveItPosePlanner:
    def __init__(self, node: Node) -> None:
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import (
            Constraints,
            JointConstraint,
            MotionPlanRequest,
            OrientationConstraint,
            PositionConstraint,
            RobotState,
        )
        from moveit_msgs.srv import GetMotionPlan
        from shape_msgs.msg import SolidPrimitive

        self.node = node
        self.PoseStamped = PoseStamped
        self.Constraints = Constraints
        self.JointConstraint = JointConstraint
        self.MotionPlanRequest = MotionPlanRequest
        self.OrientationConstraint = OrientationConstraint
        self.PositionConstraint = PositionConstraint
        self.RobotState = RobotState
        self.SolidPrimitive = SolidPrimitive
        self.GetMotionPlan = GetMotionPlan

        self.group_name = str(node.get_parameter("moveit.group_name").value)
        self.base_frame = str(node.get_parameter("moveit.base_frame").value)
        self.end_effector_link = str(node.get_parameter("moveit.end_effector_link").value)
        self.position_tolerance_m = float(node.get_parameter("moveit.position_tolerance_m").value)
        self.orientation_tolerance_rad = float(
            node.get_parameter("moveit.orientation_tolerance_rad").value
        )
        self.target_point_offset = [
            float(value)
            for value in node.get_parameter("moveit.target_point_offset").value
        ]
        self.joint_names = list(node.get_parameter("joints").value)
        self.wrist_joint_tolerances_rad = [
            float(value)
            for value in node.get_parameter(
                "moveit.wrist_joint_tolerances_rad"
            ).value
        ]
        if len(self.wrist_joint_tolerances_rad) != 3:
            raise ValueError(
                "moveit.wrist_joint_tolerances_rad must contain three values"
            )
        self.planning_time_sec = float(node.get_parameter("moveit.planning_time_sec").value)
        self.planning_attempts = int(node.get_parameter("moveit.planning_attempts").value)
        self.velocity_scale = float(node.get_parameter("moveit.velocity_scale").value)
        self.acceleration_scale = float(node.get_parameter("moveit.acceleration_scale").value)
        self.calibrated_joint_tolerance_rad = float(
            node.get_parameter("moveit.calibrated_joint_tolerance_rad").value
        )
        self.target_orientation: Optional[List[float]] = None
        self.target_wrist_positions: Optional[List[float]] = None

        self.plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")

    def wait_ready(self, timeout_sec: float) -> bool:
        plan_ready = self.plan_client.wait_for_service(timeout_sec=timeout_sec)
        if not plan_ready:
            self.node.get_logger().error("Missing MoveIt service: /plan_kinematic_path")
        return plan_ready

    def plan_to(
        self,
        label: str,
        position: Sequence[float],
        *,
        candidate_count: int = 1,
        start_positions: Optional[Sequence[float]] = None,
        max_joint_step_rad: Optional[float] = None,
    ) -> JointTrajectory:
        candidate_count = max(1, int(candidate_count))
        candidates = []
        errors = []
        for candidate_index in range(1, candidate_count + 1):
            try:
                trajectory = self._request_plan(position)
            except Exception as exc:
                errors.append(str(exc))
                self.node.get_logger().warning(
                    f"MoveIt candidate {candidate_index}/{candidate_count} for {label} "
                    f"failed: {exc}"
                )
                continue

            score, max_step = self._trajectory_score(trajectory, start_positions)
            if max_joint_step_rad is not None and max_step > max_joint_step_rad:
                errors.append(
                    f"candidate {candidate_index} joint step {max_step:.3f} rad exceeds "
                    f"{max_joint_step_rad:.3f} rad"
                )
                self.node.get_logger().warning(
                    f"Rejecting MoveIt candidate {candidate_index}/{candidate_count} "
                    f"for {label}: joint step {max_step:.3f} rad exceeds "
                    f"{max_joint_step_rad:.3f} rad"
                )
                continue
            candidates.append((score, candidate_index, max_step, trajectory))

        if not candidates:
            detail = "; ".join(errors) if errors else "no candidate trajectory returned"
            raise RuntimeError(f"MoveIt found no safe plan for {label}: {detail}")

        score, candidate_index, max_step, trajectory = min(
            candidates, key=lambda item: item[0]
        )
        if candidate_count > 1:
            self.node.get_logger().info(
                f"Selected MoveIt candidate {candidate_index}/{candidate_count} for {label}: "
                f"score={score:.3f}, max_joint_step={max_step:.3f} rad"
            )
        self.node.get_logger().info(
            f"MoveIt planned {label} to ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )
        return trajectory

    def _request_plan(self, position: Sequence[float]) -> JointTrajectory:
        request = self.GetMotionPlan.Request()
        request.motion_plan_request = self._build_motion_plan_request(position)

        future = self.plan_client.call_async(request)
        self._spin_future(future, "plan pose")
        response = future.result().motion_plan_response
        if response.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f"MoveIt failed to plan pose: error_code={response.error_code.val}"
            )
        return response.trajectory.joint_trajectory

    def plan_to_joints(
        self, label: str, target_positions: Sequence[float]
    ) -> JointTrajectory:
        if len(target_positions) != len(self.joint_names):
            raise ValueError(
                f"{label} joint target has {len(target_positions)} values; "
                f"expected {len(self.joint_names)}"
            )
        request = self.MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time_sec
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        request.start_state = self.RobotState()
        request.start_state.is_diff = True

        goal = self.Constraints()
        for joint_name, position in zip(self.joint_names, target_positions):
            joint = self.JointConstraint()
            joint.joint_name = joint_name
            joint.position = float(position)
            joint.tolerance_above = self.calibrated_joint_tolerance_rad
            joint.tolerance_below = self.calibrated_joint_tolerance_rad
            joint.weight = 1.0
            goal.joint_constraints.append(joint)
        request.goal_constraints = [goal]
        request.path_constraints = self._wrist_constraints()

        service_request = self.GetMotionPlan.Request()
        service_request.motion_plan_request = request
        future = self.plan_client.call_async(service_request)
        self._spin_future(future, f"plan calibrated {label}")
        response = future.result().motion_plan_response
        if response.error_code.val != MOVEIT_SUCCESS:
            raise RuntimeError(
                f"MoveIt failed to plan calibrated {label}: "
                f"error_code={response.error_code.val}"
            )
        self.node.get_logger().info(
            f"MoveIt planned calibrated joint target for {label}"
        )
        return response.trajectory.joint_trajectory

    def _trajectory_score(
        self,
        trajectory: JointTrajectory,
        start_positions: Optional[Sequence[float]],
    ) -> tuple:
        if not trajectory.points:
            return float("inf"), float("inf")
        final_by_name = dict(
            zip(trajectory.joint_names, trajectory.points[-1].positions)
        )
        final_positions = [float(final_by_name[name]) for name in self.joint_names]
        if start_positions is None:
            start_positions = final_positions
        deltas = [
            abs(target - start)
            for target, start in zip(final_positions, start_positions)
        ]
        max_step = max(deltas, default=0.0)
        # Prefer compact IK branches.  Bias the redundant wrist joints toward
        # the calibrated tool pose, since large wrist excursions are the plans
        # most likely to contact the table before the Cartesian goal is reached.
        wrist_error = 0.0
        if self.target_wrist_positions is not None:
            wrist_error = sum(
                abs(target - preferred)
                for target, preferred in zip(
                    final_positions[-3:], self.target_wrist_positions
                )
            )
        return 3.0 * max_step + sum(deltas) + wrist_error, max_step

    def clear_orientation_constraint(self) -> None:
        self.target_orientation = None
        self.target_wrist_positions = None

    def lock_tool_pose(
        self, quaternion: Sequence[float], arm_positions: Sequence[float]
    ) -> None:
        self.target_orientation = [float(value) for value in quaternion]
        self.target_wrist_positions = [float(value) for value in arm_positions[-3:]]

    def _build_motion_plan_request(self, position: Sequence[float]):
        request = self.MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time_sec
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        request.start_state = self.RobotState()
        request.start_state.is_diff = True
        request.goal_constraints = [self._pose_goal(position)]
        request.path_constraints = self._wrist_constraints()
        return request

    def _pose_goal(self, position: Sequence[float]):
        pose = self.PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])

        sphere = self.SolidPrimitive()
        sphere.type = self.SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance_m]

        constraint = self.PositionConstraint()
        constraint.header.frame_id = self.base_frame
        constraint.link_name = self.end_effector_link
        constraint.target_point_offset.x = self.target_point_offset[0]
        constraint.target_point_offset.y = self.target_point_offset[1]
        constraint.target_point_offset.z = self.target_point_offset[2]
        constraint.constraint_region.primitives = [sphere]
        constraint.constraint_region.primitive_poses = [pose.pose]
        constraint.weight = 1.0

        goal = self.Constraints()
        goal.position_constraints = [constraint]
        if self.target_orientation is not None:
            goal.orientation_constraints = [self._orientation_constraint()]
        goal.joint_constraints = self._wrist_constraints().joint_constraints
        return goal

    def _wrist_constraints(self):
        constraints = self.Constraints()
        if self.target_wrist_positions is None:
            return constraints
        for joint_name, position, tolerance in zip(
            self.joint_names[-3:],
            self.target_wrist_positions,
            self.wrist_joint_tolerances_rad,
        ):
            joint = self.JointConstraint()
            joint.joint_name = joint_name
            joint.position = position
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _orientation_constraint(self):
        orientation = self.OrientationConstraint()
        orientation.header.frame_id = self.base_frame
        orientation.link_name = self.end_effector_link
        orientation.orientation.x = self.target_orientation[0]
        orientation.orientation.y = self.target_orientation[1]
        orientation.orientation.z = self.target_orientation[2]
        orientation.orientation.w = self.target_orientation[3]
        orientation.absolute_x_axis_tolerance = self.orientation_tolerance_rad
        orientation.absolute_y_axis_tolerance = self.orientation_tolerance_rad
        orientation.absolute_z_axis_tolerance = self.orientation_tolerance_rad
        orientation.weight = 1.0
        return orientation

    def _spin_future(self, future: Future, label: str) -> None:
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.planning_time_sec + 10.0)
        if not future.done():
            raise TimeoutError(f"Timed out while waiting to {label}")


class PickPlaceNode(Node):
    ARM_POSES = (
        "home",
        "pose1",
        "pose2",
        "a_above",
        "a_pick",
        "b_above",
        "b_place",
    )
    ARM_ORIGINS = (
        ((0.0, 0.0, 0.100), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.038), (-1.5708, 0.0, 0.0)),
        ((0.0, -0.100, 0.0), (0.0, 0.0, 0.0)),
        ((0.108, -0.005, -0.001), (0.0, 1.5708, 0.0)),
        ((-0.001, 0.0, 0.0), (0.0, -1.5708, 0.0)),
        ((0.060, 0.0, 0.0), (0.0, 1.5708, 0.0)),
    )
    GRIPPER_BASE_ORIGIN = ((0.0, 0.0, 0.038), (1.579, 0.0, 0.0))

    def __init__(self) -> None:
        super().__init__("mecharm_sim")

        self.declare_parameter("arm_controller_name", "arm_controller")
        self.declare_parameter("gripper_controller_name", "gripper_action_controller")
        self.declare_parameter("result_log", "~/.ros/mecharm_sim/pick_place_results.csv")
        self.declare_parameter("action_timeout_sec", 20.0)
        self.declare_parameter("initialization_action_timeout_sec", 35.0)
        self.declare_parameter("settle_time_sec", 0.7)
        self.declare_parameter("default_arm_duration_sec", 3.0)
        self.declare_parameter("default_gripper_duration_sec", 1.2)
        self.declare_parameter("execution_tolerance_rad", 0.13)
        self.declare_parameter("arm_velocity_gain", 1.5)
        self.declare_parameter("arm_max_velocity_rad_sec", 0.45)
        self.declare_parameter("repeat_count", 1)
        self.declare_parameter("abort_to_safe_on_error", True)
        self.declare_parameter("use_moveit", True)
        self.declare_parameter("use_detachable_joint", True)
        self.declare_parameter("attach_topic", "/mecharm_gripper/attach")
        self.declare_parameter("detach_topic", "/mecharm_gripper/detach")
        self.declare_parameter("lock_ball_on_table", True)
        self.declare_parameter("table_lock_attach_topic", "/target_ball_table/attach")
        self.declare_parameter("table_lock_detach_topic", "/target_ball_table/detach")
        self.declare_parameter("world_name", "mecharm_pick_place")
        self.declare_parameter("target_model", "target_ball")
        self.declare_parameter("grasp_link_name", "gripper_base")
        self.declare_parameter("robot_spawn_z", 0.34)
        self.declare_parameter("scene.ball_position", [0.22, -0.06, 0.345])
        self.declare_parameter("scene.reset_ball_each_attempt", True)
        self.declare_parameter("scene.grasp_tolerance_m", 0.012)
        self.declare_parameter("scene.grasp_xy_tolerance_m", 0.008)
        self.declare_parameter("scene.grasp_z_tolerance_m", 0.008)
        self.declare_parameter("scene.approach_height_m", 0.115)
        self.declare_parameter("scene.grasp_center_height_offset_m", 0.0)
        self.declare_parameter("scene.place_center_height_offset_m", 0.0)
        self.declare_parameter("scene.place_position", [0.18, 0.16, 0.345])
        self.declare_parameter("scene.release_tolerance_m", 0.012)
        self.declare_parameter("scene.max_place_corrections", 2)
        self.declare_parameter("scene.placement_tolerance_m", 0.045)
        self.declare_parameter("scene.force_place_on_release", False)
        self.declare_parameter("scene.pose_query_timeout_sec", 2.0)
        self.declare_parameter("safety.safe_height_m", 0.18)
        self.declare_parameter("safety.max_joint_step_rad", 1.2)
        self.declare_parameter("moveit.group_name", "arm")
        self.declare_parameter("moveit.base_frame", "world")
        self.declare_parameter("moveit.end_effector_link", "gripper_base")
        self.declare_parameter("moveit.position_tolerance_m", 0.004)
        self.declare_parameter("moveit.orientation_tolerance_rad", 0.08)
        self.declare_parameter("moveit.target_point_offset", [0.0, 0.057, 0.0])
        self.declare_parameter(
            "moveit.wrist_joint_tolerances_rad", [0.35, 0.90, 0.35]
        )
        self.declare_parameter("moveit.planning_time_sec", 5.0)
        self.declare_parameter("moveit.planning_attempts", 10)
        self.declare_parameter("moveit.velocity_scale", 0.2)
        self.declare_parameter("moveit.acceleration_scale", 0.2)
        self.declare_parameter("moveit.execution_time_scale", 4.0)
        self.declare_parameter("moveit.precise_position_tolerance_m", 0.008)
        self.declare_parameter("moveit.precise_position_attempts", 3)
        self.declare_parameter("moveit.precise_joint_tolerance_rad", 0.05)
        self.declare_parameter("moveit.precise_joint_timeout_sec", 3.0)
        self.declare_parameter("moveit.precise_max_velocity_rad_sec", 0.12)
        self.declare_parameter("moveit.calibrated_joint_tolerance_rad", 0.003)
        self.declare_parameter("moveit.precise_retry_retreat_m", 0.09)
        self.declare_parameter("moveit.grasp_plan_candidates", 3)
        self.declare_parameter("moveit.max_grasp_joint_step_rad", 1.0)
        self.declare_parameter(
            "joints",
            [
                "joint1_to_base",
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
            ],
        )

        for pose in self.ARM_POSES:
            self.declare_parameter(f"poses.{pose}", [0.0] * 6)
        self.declare_parameter("gripper.open", 0.10)
        self.declare_parameter("gripper.closed", -0.14)
        self.declare_parameter("safety.joint_limits.joint1_to_base", [-2.792527, 2.792527])
        self.declare_parameter("safety.joint_limits.joint2_to_joint1", [-1.3089, 2.0943])
        self.declare_parameter("safety.joint_limits.joint3_to_joint2", [-3.0543, 1.1344])
        self.declare_parameter("safety.joint_limits.joint4_to_joint3", [-2.7052, 2.7052])
        self.declare_parameter("safety.joint_limits.joint5_to_joint4", [-2.0071, 2.0071])
        self.declare_parameter("safety.joint_limits.joint6_to_joint5", [-3.14, 3.14])
        self.declare_parameter("safety.joint_limits.gripper_controller", [-0.74, 0.15])

        self.joints = list(self.get_parameter("joints").value)
        self.arm_controller_name = self.get_parameter("arm_controller_name").value
        self.gripper_controller_name = self.get_parameter("gripper_controller_name").value
        self.action_timeout_sec = float(self.get_parameter("action_timeout_sec").value)
        self.initialization_action_timeout_sec = float(
            self.get_parameter("initialization_action_timeout_sec").value
        )
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.arm_duration_sec = float(self.get_parameter("default_arm_duration_sec").value)
        self.gripper_duration_sec = float(self.get_parameter("default_gripper_duration_sec").value)
        self.execution_tolerance_rad = float(self.get_parameter("execution_tolerance_rad").value)
        self.arm_velocity_gain = float(self.get_parameter("arm_velocity_gain").value)
        self.arm_max_velocity = float(self.get_parameter("arm_max_velocity_rad_sec").value)
        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.abort_to_safe_on_error = bool(self.get_parameter("abort_to_safe_on_error").value)
        self.use_moveit = bool(self.get_parameter("use_moveit").value)
        self.use_detachable_joint = bool(self.get_parameter("use_detachable_joint").value)
        self.attach_topic = str(self.get_parameter("attach_topic").value)
        self.detach_topic = str(self.get_parameter("detach_topic").value)
        self.lock_ball_on_table = bool(self.get_parameter("lock_ball_on_table").value)
        self.table_lock_attach_topic = str(
            self.get_parameter("table_lock_attach_topic").value
        )
        self.table_lock_detach_topic = str(
            self.get_parameter("table_lock_detach_topic").value
        )
        self.world_name = str(self.get_parameter("world_name").value)
        self.target_model = str(self.get_parameter("target_model").value)
        self.grasp_link_name = str(self.get_parameter("grasp_link_name").value)
        self.robot_spawn_z = float(self.get_parameter("robot_spawn_z").value)
        self.ball_position = self._get_vector3_parameter("scene.ball_position")
        self.reset_ball_each_attempt = bool(
            self.get_parameter("scene.reset_ball_each_attempt").value
        )
        self.grasp_tolerance_m = float(self.get_parameter("scene.grasp_tolerance_m").value)
        self.grasp_xy_tolerance_m = float(
            self.get_parameter("scene.grasp_xy_tolerance_m").value
        )
        self.grasp_z_tolerance_m = float(
            self.get_parameter("scene.grasp_z_tolerance_m").value
        )
        self.approach_height_m = float(self.get_parameter("scene.approach_height_m").value)
        self.grasp_center_height_offset_m = float(
            self.get_parameter("scene.grasp_center_height_offset_m").value
        )
        self.place_center_height_offset_m = float(
            self.get_parameter("scene.place_center_height_offset_m").value
        )
        self.grasp_center_offset = self._get_vector3_parameter(
            "moveit.target_point_offset"
        )
        self.place_position = self._get_vector3_parameter("scene.place_position")
        self.release_tolerance_m = float(
            self.get_parameter("scene.release_tolerance_m").value
        )
        self.max_place_corrections = int(
            self.get_parameter("scene.max_place_corrections").value
        )
        self.placement_tolerance_m = float(self.get_parameter("scene.placement_tolerance_m").value)
        self.force_place_on_release = bool(self.get_parameter("scene.force_place_on_release").value)
        self.pose_query_timeout_sec = float(self.get_parameter("scene.pose_query_timeout_sec").value)
        self.safe_height_m = float(self.get_parameter("safety.safe_height_m").value)
        self.max_joint_step_rad = float(self.get_parameter("safety.max_joint_step_rad").value)
        self.moveit_execution_time_scale = float(self.get_parameter("moveit.execution_time_scale").value)
        self.moveit_precise_position_tolerance_m = float(
            self.get_parameter("moveit.precise_position_tolerance_m").value
        )
        self.moveit_precise_position_attempts = int(
            self.get_parameter("moveit.precise_position_attempts").value
        )
        self.moveit_precise_joint_tolerance_rad = float(
            self.get_parameter("moveit.precise_joint_tolerance_rad").value
        )
        self.moveit_precise_joint_timeout_sec = float(
            self.get_parameter("moveit.precise_joint_timeout_sec").value
        )
        self.moveit_precise_max_velocity = float(
            self.get_parameter("moveit.precise_max_velocity_rad_sec").value
        )
        self.moveit_precise_retry_retreat_m = float(
            self.get_parameter("moveit.precise_retry_retreat_m").value
        )
        self.moveit_grasp_plan_candidates = int(
            self.get_parameter("moveit.grasp_plan_candidates").value
        )
        self.moveit_max_grasp_joint_step_rad = float(
            self.get_parameter("moveit.max_grasp_joint_step_rad").value
        )
        self.poses = {pose: self._get_pose(pose) for pose in self.ARM_POSES}
        self.gripper_open = float(self.get_parameter("gripper.open").value)
        self.gripper_closed = float(self.get_parameter("gripper.closed").value)
        self.result_log = Path(str(self.get_parameter("result_log").value)).expanduser()
        self.last_arm_goal: Optional[List[float]] = None

        self.joint_limits = {
            joint: tuple(
                float(v)
                for v in self.get_parameter(f"safety.joint_limits.{joint}").value
            )
            for joint in self.joints
        }
        self.gripper_limits = tuple(
            float(v) for v in self.get_parameter("safety.joint_limits.gripper_controller").value
        )

        self.latest_joint_positions: Dict[str, float] = {}
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.arm_trajectory_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.arm_controller_name}/commands",
            10,
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"{self.gripper_controller_name}/follow_joint_trajectory",
        )
        self.moveit_planner: Optional[MoveItPosePlanner] = None
        if self.use_moveit:
            self.moveit_planner = MoveItPosePlanner(self)

    def _get_pose(self, name: str) -> List[float]:
        values = [float(v) for v in self.get_parameter(f"poses.{name}").value]
        if len(values) != len(self.joints):
            raise ValueError(f"poses.{name} must contain {len(self.joints)} joint values")
        return values

    def _get_vector3_parameter(self, name: str) -> List[float]:
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return values

    def run(self) -> bool:
        self._prepare_log()
        if not self._wait_for_servers():
            self._append_log(0, "failed", "action server not available")
            return False

        ok = True
        for attempt in range(1, self.repeat_count + 1):
            self.get_logger().info(f"Starting pick-place attempt {attempt}/{self.repeat_count}")
            reason = "completed"
            try:
                self._run_one_attempt()
            except Exception as exc:
                ok = False
                reason = str(exc)
                self.get_logger().error(reason)
                if self.abort_to_safe_on_error:
                    self._try_abort_to_safe()
            self._append_log(attempt, "success" if reason == "completed" else "failed", reason)
            if reason != "completed":
                break

        return ok

    def _run_one_attempt(self) -> None:
        self._prepare_robot()
        self._pick_ball()
        self._place_ball()
        self._move_arm("home", safe_move=True)

    def _prepare_robot(self) -> None:
        # Both detachable joints start attached. Release them before resetting
        # the ball, then lock only the table joint while the gripper approaches.
        self._set_detachable_joint(False)
        self._set_table_ball_lock(False)
        self._settle()
        if self.reset_ball_each_attempt:
            self._set_model_pose(self.ball_position)
            self._settle()
        self._set_table_ball_lock(True)
        self._move_gripper("open", self.gripper_open)
        self._move_arm("home")

    def _pick_ball(self) -> None:
        if self.use_moveit:
            ball_position = self._require_entity_position(self.target_model)
            above = self._offset_z(ball_position, self.approach_height_m)
            grasp = self._offset_z(
                ball_position, self.grasp_center_height_offset_m
            )
            self.moveit_planner.lock_tool_pose(
                self._estimate_gripper_base_orientation(self.poses["a_pick"]),
                self.poses["a_pick"],
            )
            self._move_pose("ball_approach", above)
            self._move_pose_precisely("ball_grasp", grasp)
            self._ensure_grasp_alignment(ball_position)
        else:
            self._move_arm("a_above", safe_move=True)
            self._move_arm("a_pick")
        self._move_gripper("closed", self.gripper_closed)
        # Transfer ownership without an unlocked interval in which contact can
        # push the spherical target away from the gripper center.
        self._set_detachable_joint(True)
        self._set_table_ball_lock(False)
        if self.use_moveit:
            self._move_pose("ball_lift", self._offset_z(ball_position, self.approach_height_m))
        else:
            self._move_arm("a_above", safe_move=True)

    def _place_ball(self) -> None:
        if self.use_moveit:
            above = self._offset_z(self.place_position, self.approach_height_m)
            place = self._offset_z(
                self.place_position, self.place_center_height_offset_m
            )
            self.moveit_planner.lock_tool_pose(
                self._estimate_gripper_base_orientation(self.poses["b_place"]),
                self.poses["b_place"],
            )
            self._move_pose("place_approach", above)
            self._move_pose("place_release", place)
            self._move_ball_to_release_target()
        else:
            self._move_arm("b_above", safe_move=True)
            self._move_arm("b_place")
        self._move_gripper("open", self.gripper_open)
        self._set_detachable_joint(False)
        self._ensure_ball_at_place()
        if self.use_moveit:
            self._move_pose("place_lift", self._offset_z(self.place_position, self.approach_height_m))
        else:
            self._move_arm("b_above", safe_move=True)

    def _wait_for_servers(self) -> bool:
        timeout = self.action_timeout_sec
        gripper_ready = self.gripper_client.wait_for_server(timeout_sec=timeout)
        joints_ready = self._wait_for_joint_states(timeout)
        arm_ready = self._wait_for_arm_controller(timeout)
        moveit_ready = True
        if self.moveit_planner is not None:
            moveit_ready = self.moveit_planner.wait_ready(timeout)
        if not joints_ready:
            self.get_logger().error("No /joint_states received")
        if not gripper_ready:
            self.get_logger().error(
                f"Missing action server: {self.gripper_controller_name}/follow_joint_trajectory"
            )
        if not arm_ready:
            self.get_logger().error(
                f"Missing command subscriber: {self.arm_controller_name}/commands"
            )
        return joints_ready and gripper_ready and arm_ready and moveit_ready

    def _wait_for_arm_controller(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.arm_trajectory_pub.get_subscription_count() > 0:
                zero = Float64MultiArray(data=[0.0] * len(self.joints))
                warmup_deadline = time.monotonic() + 1.0
                while time.monotonic() < warmup_deadline:
                    self.arm_trajectory_pub.publish(zero)
                    rclpy.spin_once(self, timeout_sec=0.05)
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _move_pose(
        self,
        label: str,
        position: Sequence[float],
        *,
        candidate_count: int = 1,
        max_joint_step_rad: Optional[float] = None,
        completion_timeout_sec: Optional[float] = None,
        completion_tolerance_rad: Optional[float] = None,
        completion_max_velocity_rad_sec: Optional[float] = None,
        cartesian_target: Optional[Sequence[float]] = None,
    ) -> List[float]:
        if self.moveit_planner is None:
            raise RuntimeError("MoveIt planner is not initialized")
        trajectory = self.moveit_planner.plan_to(
            label,
            position,
            candidate_count=candidate_count,
            start_positions=self._current_arm_positions(),
            max_joint_step_rad=max_joint_step_rad,
        )
        final_positions = self._trajectory_final_positions(trajectory)
        self._execute_planned_arm_trajectory(
            label,
            trajectory,
            completion_timeout_sec=completion_timeout_sec,
            completion_tolerance_rad=completion_tolerance_rad,
            completion_max_velocity_rad_sec=completion_max_velocity_rad_sec,
            cartesian_target=cartesian_target,
        )
        return final_positions

    def _move_joint_target(
        self,
        label: str,
        target_positions: Sequence[float],
        cartesian_target: Sequence[float],
    ) -> List[float]:
        if self.moveit_planner is None:
            raise RuntimeError("MoveIt planner is not initialized")
        trajectory = self.moveit_planner.plan_to_joints(label, target_positions)
        final_positions = self._trajectory_final_positions(trajectory)
        self._execute_planned_arm_trajectory(
            label,
            trajectory,
            completion_timeout_sec=self.moveit_precise_joint_timeout_sec,
            completion_tolerance_rad=self.moveit_precise_joint_tolerance_rad,
            completion_max_velocity_rad_sec=self.moveit_precise_max_velocity,
            cartesian_target=cartesian_target,
        )
        return final_positions

    def _move_pose_precisely(
        self,
        label: str,
        position: Sequence[float],
        calibrated_joint_target: Optional[Sequence[float]] = None,
    ) -> None:
        for attempt in range(1, self.moveit_precise_position_attempts + 1):
            if attempt > 1:
                retreat = self._offset_z(
                    position, self.moveit_precise_retry_retreat_m
                )
                self.get_logger().warning(
                    f"{label} remained outside Cartesian tolerance; retreating "
                    f"{self.moveit_precise_retry_retreat_m:.3f} m before retry"
                )
                self._move_pose(f"{label}_retry_retreat_{attempt - 1}", retreat)
            attempt_label = label if attempt == 1 else f"{label}_correction_{attempt - 1}"
            try:
                if calibrated_joint_target is not None:
                    self._move_joint_target(
                        attempt_label, calibrated_joint_target, position
                    )
                else:
                    self._move_pose(
                        attempt_label,
                        position,
                        candidate_count=self.moveit_grasp_plan_candidates,
                        max_joint_step_rad=self.moveit_max_grasp_joint_step_rad,
                        completion_timeout_sec=self.moveit_precise_joint_timeout_sec,
                        completion_tolerance_rad=self.moveit_precise_joint_tolerance_rad,
                        completion_max_velocity_rad_sec=self.moveit_precise_max_velocity,
                        cartesian_target=position,
                    )
            except (RuntimeError, TimeoutError) as exc:
                # A low-height trajectory can stall on contact, and MoveIt can
                # occasionally return an unnecessarily large IK branch.  Both
                # are recoverable here: the next attempt first retreats, then
                # asks for a fresh set of candidate plans.
                self.get_logger().warning(
                    f"{attempt_label} descent failed: {exc}; replanning after retreat"
                )
                error = float("inf")
                continue
            if self.settle_time_sec > 0:
                self._settle()
            self.last_arm_goal = self._current_arm_positions()
            measured = self._estimate_grasp_center_position()
            error = self._distance(measured, position)
            xy_error = math.hypot(
                float(measured[0]) - float(position[0]),
                float(measured[1]) - float(position[1]),
            )
            z_error = abs(float(measured[2]) - float(position[2]))
            self.get_logger().info(
                f"{label} Cartesian FK error {error:.3f} m "
                f"(xy={xy_error:.3f}, z={z_error:.3f}) after attempt "
                f"{attempt}/{self.moveit_precise_position_attempts}"
            )
            if (
                error <= self.moveit_precise_position_tolerance_m
                and xy_error <= self.grasp_xy_tolerance_m
                and z_error <= self.grasp_z_tolerance_m
            ):
                return
        raise RuntimeError(
            f"Unable to reach {label}: Cartesian FK error {error:.3f} m exceeds "
            f"{self.moveit_precise_position_tolerance_m:.3f} m tolerance"
        )

    def _execute_planned_arm_trajectory(
        self,
        label: str,
        trajectory: JointTrajectory,
        *,
        completion_timeout_sec: Optional[float] = None,
        completion_tolerance_rad: Optional[float] = None,
        completion_max_velocity_rad_sec: Optional[float] = None,
        cartesian_target: Optional[Sequence[float]] = None,
    ) -> None:
        if not trajectory.points:
            raise RuntimeError(f"MoveIt returned an empty trajectory for {label}")
        trajectory.joint_names = list(trajectory.joint_names)
        self._scale_trajectory_time(trajectory, self.moveit_execution_time_scale)
        self._publish_arm_trajectory(
            trajectory,
            label,
            completion_timeout_sec=completion_timeout_sec,
            completion_tolerance_rad=completion_tolerance_rad,
            completion_max_velocity_rad_sec=completion_max_velocity_rad_sec,
            cartesian_target=cartesian_target,
        )
        self.last_arm_goal = self._current_arm_positions()

    def _scale_trajectory_time(self, trajectory: JointTrajectory, scale: float) -> None:
        if scale <= 0.0:
            raise ValueError("moveit.execution_time_scale must be positive")
        for point in trajectory.points:
            seconds = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            point.time_from_start = Duration(seconds=seconds * scale).to_msg()

    def _trajectory_final_positions(self, trajectory: JointTrajectory) -> List[float]:
        final_point = trajectory.points[-1]
        positions_by_name = dict(zip(trajectory.joint_names, final_point.positions))
        try:
            return [float(positions_by_name[joint]) for joint in self.joints]
        except KeyError as exc:
            raise RuntimeError(f"MoveIt trajectory is missing joint {exc.args[0]}") from exc

    def _move_arm(self, pose_name: str, safe_move: bool = False) -> None:
        positions = self.poses[pose_name]
        self._validate_positions(self.joints, positions, self.joint_limits, pose_name)
        if self.last_arm_goal is not None:
            self._validate_joint_step(pose_name, positions)
        if safe_move:
            self.get_logger().info(f"Safe-height move via {pose_name}; configured clearance {self.safe_height_m:.3f} m")
        self._send_arm_positions(positions, self.arm_duration_sec, pose_name)
        self.last_arm_goal = self._current_arm_positions()

    def _send_arm_positions(self, positions: Iterable[float], duration_sec: float, label: str) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        trajectory.points = [point]
        self._publish_arm_trajectory(trajectory, label)

    def _move_gripper(
        self,
        name: str,
        position: float,
        timeout_sec: Optional[float] = None,
    ) -> None:
        self._validate_positions(
            ["gripper_controller"],
            [position],
            {"gripper_controller": self.gripper_limits},
            f"gripper.{name}",
        )
        self._send_trajectory(
            self.gripper_client,
            ["gripper_controller"],
            [position],
            self.gripper_duration_sec,
            f"gripper.{name}",
            timeout_sec=timeout_sec,
        )

    def _send_trajectory(
        self,
        client: ActionClient,
        joint_names: Iterable[str],
        positions: Iterable[float],
        duration_sec: float,
        label: str,
        timeout_sec: Optional[float] = None,
    ) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        trajectory.points = [point]

        self._send_follow_joint_trajectory(
            client, trajectory, label, timeout_sec=timeout_sec
        )

    def _send_follow_joint_trajectory(
        self,
        client: ActionClient,
        trajectory: JointTrajectory,
        label: str,
        timeout_sec: Optional[float] = None,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = client.send_goal_async(goal)
        self._spin_future(send_future, f"send {label}", timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError(f"Controller rejected trajectory: {label}")

        result_future = goal_handle.get_result_async()
        self._spin_future(result_future, f"execute {label}", timeout_sec=timeout_sec)
        wrapped_result = result_future.result()
        status = getattr(wrapped_result, "status", None)
        error_code = wrapped_result.result.error_code
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Trajectory failed: {label}, status={status}, error_code={error_code}, "
                f"message={wrapped_result.result.error_string}"
            )
        self.get_logger().info(f"Reached {label}")
        if self.settle_time_sec > 0:
            self._settle()

    def _publish_arm_trajectory(
        self,
        trajectory: JointTrajectory,
        label: str,
        *,
        completion_timeout_sec: Optional[float] = None,
        completion_tolerance_rad: Optional[float] = None,
        completion_max_velocity_rad_sec: Optional[float] = None,
        cartesian_target: Optional[Sequence[float]] = None,
    ) -> None:
        final_positions = self._trajectory_final_positions(trajectory)
        end_time = self._trajectory_duration_sec(trajectory)

        target = ", ".join(f"{value:.3f}" for value in final_positions)
        self.get_logger().info(
            f"Publishing arm trajectory for {label}: points={len(trajectory.points)}, "
            f"duration={end_time:.2f}s, target=[{target}]"
        )
        if len(trajectory.points) > 1:
            self._follow_arm_trajectory(
                trajectory,
                completion_timeout_sec=completion_timeout_sec,
                completion_tolerance_rad=completion_tolerance_rad,
                completion_max_velocity_rad_sec=completion_max_velocity_rad_sec,
                cartesian_target=cartesian_target,
            )
        else:
            self._drive_arm_to(
                final_positions,
                completion_timeout_sec
                if completion_timeout_sec is not None
                else max(end_time + 3.0, self.action_timeout_sec),
                tolerance_rad=completion_tolerance_rad,
                max_velocity_rad_sec=completion_max_velocity_rad_sec,
                cartesian_target=cartesian_target,
                cartesian_tolerance_m=(
                    self.moveit_precise_position_tolerance_m
                    if cartesian_target is not None
                    else None
                ),
                cartesian_xy_tolerance_m=self.grasp_xy_tolerance_m,
                cartesian_z_tolerance_m=self.grasp_z_tolerance_m,
            )
        self.get_logger().info(f"Reached {label}")
        if self.settle_time_sec > 0:
            self._settle()

    def _follow_arm_trajectory(
        self,
        trajectory: JointTrajectory,
        *,
        completion_timeout_sec: Optional[float] = None,
        completion_tolerance_rad: Optional[float] = None,
        completion_max_velocity_rad_sec: Optional[float] = None,
        cartesian_target: Optional[Sequence[float]] = None,
    ) -> None:
        samples = [(0.0, self._current_arm_positions())]
        for point in trajectory.points:
            positions_by_name = dict(zip(trajectory.joint_names, point.positions))
            positions = [float(positions_by_name[joint]) for joint in self.joints]
            stamp = point.time_from_start
            sample_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            if sample_time <= samples[-1][0]:
                samples[-1] = (samples[-1][0], positions)
            else:
                samples.append((sample_time, positions))

        if len(samples) == 1:
            self._drive_arm_to(
                samples[0][1],
                self.action_timeout_sec
                if completion_timeout_sec is None
                else completion_timeout_sec,
                tolerance_rad=completion_tolerance_rad,
                max_velocity_rad_sec=completion_max_velocity_rad_sec,
                cartesian_target=cartesian_target,
                cartesian_tolerance_m=(
                    self.moveit_precise_position_tolerance_m
                    if cartesian_target is not None
                    else None
                ),
                cartesian_xy_tolerance_m=self.grasp_xy_tolerance_m,
                cartesian_z_tolerance_m=self.grasp_z_tolerance_m,
            )
            return

        start = time.monotonic()
        segment = 0
        command = Float64MultiArray()
        try:
            while True:
                if not rclpy.ok():
                    raise KeyboardInterrupt
                elapsed = time.monotonic() - start
                if elapsed >= samples[-1][0]:
                    break
                while segment + 1 < len(samples) - 1 and elapsed >= samples[segment + 1][0]:
                    segment += 1

                start_time, start_positions = samples[segment]
                end_time, end_positions = samples[segment + 1]
                ratio = (elapsed - start_time) / max(end_time - start_time, 1e-6)
                ratio = max(0.0, min(1.0, ratio))
                target_positions = [
                    start_value + ratio * (end_value - start_value)
                    for start_value, end_value in zip(start_positions, end_positions)
                ]
                errors = [
                    target - self.latest_joint_positions.get(joint, target)
                    for joint, target in zip(self.joints, target_positions)
                ]
                command.data = [
                    max(
                        -self.arm_max_velocity,
                        min(self.arm_max_velocity, self.arm_velocity_gain * error),
                    )
                    for error in errors
                ]
                self.arm_trajectory_pub.publish(command)
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            if rclpy.ok():
                command.data = [0.0] * len(self.joints)
                self.arm_trajectory_pub.publish(command)

        self._drive_arm_to(
            samples[-1][1],
            self.action_timeout_sec
            if completion_timeout_sec is None
            else completion_timeout_sec,
            tolerance_rad=completion_tolerance_rad,
            max_velocity_rad_sec=completion_max_velocity_rad_sec,
            cartesian_target=cartesian_target,
            cartesian_tolerance_m=(
                self.moveit_precise_position_tolerance_m
                if cartesian_target is not None
                else None
            ),
            cartesian_xy_tolerance_m=self.grasp_xy_tolerance_m,
            cartesian_z_tolerance_m=self.grasp_z_tolerance_m,
        )

    def _settle(self) -> None:
        deadline = time.monotonic() + self.settle_time_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _trajectory_duration_sec(self, trajectory: JointTrajectory) -> float:
        duration = trajectory.points[-1].time_from_start
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _wait_for_joint_states(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if all(joint in self.latest_joint_positions for joint in self.joints):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _drive_arm_to(
        self,
        target_positions: Sequence[float],
        timeout_sec: float,
        tolerance_rad: Optional[float] = None,
        max_velocity_rad_sec: Optional[float] = None,
        cartesian_target: Optional[Sequence[float]] = None,
        cartesian_tolerance_m: Optional[float] = None,
        cartesian_xy_tolerance_m: Optional[float] = None,
        cartesian_z_tolerance_m: Optional[float] = None,
    ) -> None:
        tolerance = (
            self.execution_tolerance_rad
            if tolerance_rad is None
            else float(tolerance_rad)
        )
        max_velocity = (
            self.arm_max_velocity
            if max_velocity_rad_sec is None
            else min(self.arm_max_velocity, float(max_velocity_rad_sec))
        )
        if max_velocity <= 0.0:
            raise ValueError("arm completion velocity must be positive")
        deadline = time.monotonic() + timeout_sec
        command = Float64MultiArray()
        try:
            while time.monotonic() < deadline:
                if not rclpy.ok():
                    raise KeyboardInterrupt
                errors = [
                    target - self.latest_joint_positions.get(joint, target)
                    for joint, target in zip(self.joints, target_positions)
                ]
                # For low-height Cartesian work, joint convergence alone is
                # not success: small joint residuals can still leave the
                # fingers centimetres from the object.  Only use the joint
                # threshold as the terminal condition for joint-only moves.
                if (
                    cartesian_target is None
                    and max(abs(error) for error in errors) <= tolerance
                ):
                    return
                if cartesian_target is not None and cartesian_tolerance_m is not None:
                    current_positions = self._current_arm_positions()
                    measured = self._estimate_grasp_center_position(current_positions)
                    distance = self._distance(measured, cartesian_target)
                    xy_error = math.hypot(
                        float(measured[0]) - float(cartesian_target[0]),
                        float(measured[1]) - float(cartesian_target[1]),
                    )
                    z_error = abs(
                        float(measured[2]) - float(cartesian_target[2])
                    )
                    xy_ok = (
                        cartesian_xy_tolerance_m is None
                        or xy_error <= cartesian_xy_tolerance_m
                    )
                    z_ok = (
                        cartesian_z_tolerance_m is None
                        or z_error <= cartesian_z_tolerance_m
                    )
                    if distance <= cartesian_tolerance_m and xy_ok and z_ok:
                        return
                command.data = [
                    max(
                        -max_velocity,
                        min(max_velocity, self.arm_velocity_gain * error),
                    )
                    for error in errors
                ]
                self.arm_trajectory_pub.publish(command)
                rclpy.spin_once(self, timeout_sec=0.05)
            raise TimeoutError(
                f"Timed out waiting for arm trajectory; final error={self._arm_error(target_positions):.3f} rad"
            )
        finally:
            if rclpy.ok():
                command.data = [0.0] * len(self.joints)
                self.arm_trajectory_pub.publish(command)

    def _arm_error(self, target_positions: Sequence[float]) -> float:
        errors = [
            abs(self.latest_joint_positions.get(joint, float("inf")) - target)
            for joint, target in zip(self.joints, target_positions)
        ]
        return max(errors)

    def _current_arm_positions(self) -> List[float]:
        return [self.latest_joint_positions[joint] for joint in self.joints]

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.latest_joint_positions[name] = float(position)

    def _spin_future(
        self,
        future: Future,
        label: str,
        timeout_sec: Optional[float] = None,
    ) -> None:
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=(
                self.action_timeout_sec if timeout_sec is None else timeout_sec
            ),
        )
        if not future.done():
            raise TimeoutError(f"Timed out while waiting to {label}")

    def _validate_positions(
        self,
        names: Iterable[str],
        positions: Iterable[float],
        limits: Dict[str, tuple],
        label: str,
    ) -> None:
        for name, value in zip(names, positions):
            if not math.isfinite(value):
                raise ValueError(f"{label}: {name} is not finite")
            lower, upper = limits[name]
            if value < lower or value > upper:
                raise ValueError(
                    f"{label}: {name}={value:.4f} exceeds limit [{lower:.4f}, {upper:.4f}]"
                )

    def _validate_joint_step(self, pose_name: str, next_goal: List[float]) -> None:
        deltas = [abs(a - b) for a, b in zip(self.last_arm_goal, next_goal)]
        max_delta = max(deltas)
        if max_delta > self.max_joint_step_rad:
            raise ValueError(
                f"{pose_name}: joint step {max_delta:.3f} rad exceeds safety limit "
                f"{self.max_joint_step_rad:.3f} rad"
            )

    def _try_abort_to_safe(self) -> None:
        try:
            self.get_logger().warn("Trying to return to home after failure")
            self._set_detachable_joint(False)
            self.last_arm_goal = None
            self._move_arm("home", safe_move=True)
            self._move_gripper("open", self.gripper_open)
        except Exception as exc:
            self.get_logger().error(f"Failed to return to safe pose: {exc}")

    def _set_detachable_joint(self, attach: bool) -> None:
        if not self.use_detachable_joint:
            return
        topic = self.attach_topic if attach else self.detach_topic
        action = "attach" if attach else "detach"
        self._send_empty_transport_command(topic, action, self.target_model)

    def _set_table_ball_lock(self, attach: bool) -> None:
        if not self.lock_ball_on_table:
            return
        topic = self.table_lock_attach_topic if attach else self.table_lock_detach_topic
        action = "lock" if attach else "unlock"
        self._send_empty_transport_command(topic, action, f"{self.target_model} on table")

    def _send_empty_transport_command(self, topic: str, action: str, subject: str) -> None:
        command = [
            "ign",
            "topic",
            "-t",
            topic,
            "-m",
            "ignition.msgs.Empty",
            "-p",
            "unused: true",
        ]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=5.0, check=False)
        if completed.returncode != 0:
            stderr = completed.stderr.strip()
            stdout = completed.stdout.strip()
            raise RuntimeError(f"Failed to {action} {subject} via {topic}: {stderr or stdout}")
        self.get_logger().info(f"Detachable joint {action} command sent on {topic}")

    def _ensure_ball_at_place(self) -> None:
        measured = self._read_entity_position(self.target_model)
        if measured is not None and self._distance(measured, self.place_position) <= self.placement_tolerance_m:
            self.get_logger().info(
                f"{self.target_model} reached place target at {self._format_xyz(measured)}"
            )
            return

        if measured is not None:
            self.get_logger().warn(
                f"{self.target_model} is at {self._format_xyz(measured)}, outside "
                f"{self.placement_tolerance_m:.3f} m target tolerance"
            )

        if self.force_place_on_release:
            self._set_model_pose(self.place_position)
            measured = self._read_entity_position(self.target_model)

        if measured is None:
            raise RuntimeError(f"Unable to read {self.target_model} pose after release")

        error = self._distance(measured, self.place_position)
        if error > self.placement_tolerance_m:
            raise RuntimeError(
                f"{self.target_model} placement error {error:.3f} m exceeds "
                f"{self.placement_tolerance_m:.3f} m tolerance; measured={self._format_xyz(measured)} "
                f"target={self._format_xyz(self.place_position)}"
            )
        self.get_logger().info(
            f"{self.target_model} verified at place target {self._format_xyz(measured)} "
            f"(error {error:.3f} m)"
        )

    def _ensure_grasp_alignment(self, ball_position: Sequence[float]) -> None:
        grasp_center = self._estimate_grasp_center_position()
        desired_center = self._offset_z(
            ball_position, self.grasp_center_height_offset_m
        )
        distance = self._distance(desired_center, grasp_center)
        xy_error = math.hypot(
            float(desired_center[0]) - grasp_center[0],
            float(desired_center[1]) - grasp_center[1],
        )
        z_error = abs(float(desired_center[2]) - grasp_center[2])
        self.get_logger().info(
            f"Pre-grasp center alignment: distance={distance:.3f} m, "
            f"xy_error={xy_error:.3f} m, z_error={z_error:.3f} m; "
            f"{self.target_model}={self._format_xyz(ball_position)}, "
            f"grasp_center={self._format_xyz(grasp_center)}"
        )
        if (
            distance > self.grasp_tolerance_m
            or xy_error > self.grasp_xy_tolerance_m
            or z_error > self.grasp_z_tolerance_m
        ):
            raise RuntimeError(
                f"Refusing to close/attach: grasp is not centered; distance={distance:.3f} m "
                f"(limit {self.grasp_tolerance_m:.3f}), xy_error={xy_error:.3f} m "
                f"(limit {self.grasp_xy_tolerance_m:.3f}), z_error={z_error:.3f} m "
                f"(limit {self.grasp_z_tolerance_m:.3f})"
            )

    def _move_ball_to_release_target(self) -> None:
        for correction in range(self.max_place_corrections + 1):
            measured = self._require_entity_position(self.target_model)
            error = [
                float(target) - float(actual)
                for target, actual in zip(self.place_position, measured)
            ]
            error_norm = math.sqrt(sum(value * value for value in error))
            self.get_logger().info(
                f"Pre-release ball error {error_norm:.3f} m: "
                f"measured={self._format_xyz(measured)}, "
                f"target={self._format_xyz(self.place_position)}"
            )
            if error_norm <= self.release_tolerance_m:
                return
            if correction >= self.max_place_corrections:
                raise RuntimeError(
                    f"Refusing to release above the plate: {self.target_model} is "
                    f"{error_norm:.3f} m from target, above "
                    f"{self.release_tolerance_m:.3f} m release tolerance"
                )

            gripper_position = self._estimate_grasp_center_position()
            corrected_target = [
                gripper_position[index] + error[index] for index in range(3)
            ]
            self._move_pose(f"place_correction_{correction + 1}", corrected_target)

    def _require_entity_position(self, entity_name: str) -> List[float]:
        position = self._read_entity_position(entity_name)
        if position is None:
            raise RuntimeError(f"Unable to read {entity_name} pose")
        return position

    @staticmethod
    def _offset_z(position: Sequence[float], offset: float) -> List[float]:
        return [float(position[0]), float(position[1]), float(position[2]) + float(offset)]

    def _estimate_gripper_base_position(self) -> List[float]:
        transform = self._estimate_gripper_base_transform(self.last_arm_goal)
        return [transform[0][3], transform[1][3], transform[2][3]]

    def _estimate_grasp_center_position(
        self, arm_positions: Optional[Sequence[float]] = None
    ) -> List[float]:
        if arm_positions is None:
            arm_positions = self.last_arm_goal
        transform = self._estimate_gripper_base_transform(arm_positions)
        return [
            transform[row][3]
            + sum(
                transform[row][column] * self.grasp_center_offset[column]
                for column in range(3)
            )
            for row in range(3)
        ]

    def _estimate_gripper_base_orientation(
        self, arm_positions: Optional[Sequence[float]] = None
    ) -> List[float]:
        transform = self._estimate_gripper_base_transform(arm_positions)
        return self._rotation_matrix_to_quaternion(transform)

    def _estimate_gripper_base_transform(
        self, arm_positions: Optional[Sequence[float]] = None
    ) -> List[List[float]]:
        if arm_positions is None:
            raise RuntimeError("Cannot estimate gripper pose before an arm trajectory has completed")
        transform = self._translation_matrix((0.0, 0.0, self.robot_spawn_z))
        for (xyz, rpy), joint_angle in zip(self.ARM_ORIGINS, arm_positions):
            transform = self._matmul(transform, self._transform_matrix(xyz, rpy))
            transform = self._matmul(transform, self._z_rotation_matrix(joint_angle))
        xyz, rpy = self.GRIPPER_BASE_ORIGIN
        transform = self._matmul(transform, self._transform_matrix(xyz, rpy))
        return transform

    def _read_entity_position(self, entity_name: str) -> Optional[List[float]]:
        topic = f"/world/{self.world_name}/pose/info"
        command = [
            "ign",
            "topic",
            "-t",
            topic,
            "-e",
            "-n",
            "1",
            "--json-output",
        ]
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=self.pose_query_timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timed out reading Gazebo pose topic {topic}")
            return None

        if completed.returncode != 0:
            self.get_logger().warn(
                f"Failed to read Gazebo pose topic {topic}: "
                f"{completed.stderr.strip() or completed.stdout.strip()}"
            )
            return None

        position = self._position_from_pose_json(completed.stdout, entity_name)
        if position is not None:
            return position

        for line in completed.stdout.splitlines():
            position = self._position_from_pose_json(line, entity_name)
            if position is not None:
                return position
        return None

    def _position_from_pose_json(self, line: str, entity_name: str) -> Optional[List[float]]:
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            return None

        poses = message.get("pose") or message.get("poses") or []
        if isinstance(poses, dict):
            poses = [poses]
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            pose_name = str(pose.get("name", ""))
            if pose_name != entity_name and not pose_name.endswith(f"::{entity_name}"):
                continue
            position = pose.get("position", {})
            try:
                return [float(position[axis]) for axis in ("x", "y", "z")]
            except (KeyError, TypeError, ValueError):
                return None
        return None

    def _set_model_pose(self, position: Sequence[float]) -> None:
        service = f"/world/{self.world_name}/set_pose"
        request = (
            f'name: "{self.target_model}", '
            f"position: {{x: {position[0]:.6f}, y: {position[1]:.6f}, z: {position[2]:.6f}}}, "
            "orientation: {w: 1.0}"
        )
        command = [
            "ign",
            "service",
            "-s",
            service,
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "-r",
            request,
        ]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=5.0, check=False)
        output = f"{completed.stdout}\n{completed.stderr}".lower()
        if completed.returncode != 0 or "false" in output:
            raise RuntimeError(
                f"Failed to set {self.target_model} pose via {service}: "
                f"{completed.stderr.strip() or completed.stdout.strip()}"
            )
        self.get_logger().info(f"Set {self.target_model} pose to {self._format_xyz(position)}")

    @staticmethod
    def _distance(a: Sequence[float], b: Sequence[float]) -> float:
        return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))

    @staticmethod
    def _format_xyz(values: Sequence[float]) -> str:
        return f"({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})"

    @staticmethod
    def _matmul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
        return [
            [sum(a[row][idx] * b[idx][col] for idx in range(4)) for col in range(4)]
            for row in range(4)
        ]

    @classmethod
    def _transform_matrix(cls, xyz: Sequence[float], rpy: Sequence[float]) -> List[List[float]]:
        rotation = cls._rpy_matrix(rpy[0], rpy[1], rpy[2])
        return [
            [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
            [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
            [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _translation_matrix(xyz: Sequence[float]) -> List[List[float]]:
        return [
            [1.0, 0.0, 0.0, xyz[0]],
            [0.0, 1.0, 0.0, xyz[1]],
            [0.0, 0.0, 1.0, xyz[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _z_rotation_matrix(angle: float) -> List[List[float]]:
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return [
            [cos_angle, -sin_angle, 0.0, 0.0],
            [sin_angle, cos_angle, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @classmethod
    def _rpy_matrix(cls, roll: float, pitch: float, yaw: float) -> List[List[float]]:
        return cls._matmul3(
            cls._matmul3(cls._z_rotation_matrix3(yaw), cls._y_rotation_matrix3(pitch)),
            cls._x_rotation_matrix3(roll),
        )

    @staticmethod
    def _matmul3(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
        return [
            [sum(a[row][idx] * b[idx][col] for idx in range(3)) for col in range(3)]
            for row in range(3)
        ]

    @staticmethod
    def _x_rotation_matrix3(angle: float) -> List[List[float]]:
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return [
            [1.0, 0.0, 0.0],
            [0.0, cos_angle, -sin_angle],
            [0.0, sin_angle, cos_angle],
        ]

    @staticmethod
    def _y_rotation_matrix3(angle: float) -> List[List[float]]:
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return [
            [cos_angle, 0.0, sin_angle],
            [0.0, 1.0, 0.0],
            [-sin_angle, 0.0, cos_angle],
        ]

    @staticmethod
    def _z_rotation_matrix3(angle: float) -> List[List[float]]:
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return [
            [cos_angle, -sin_angle, 0.0],
            [sin_angle, cos_angle, 0.0],
            [0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _rotation_matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> List[float]:
        trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            return [
                (matrix[2][1] - matrix[1][2]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale,
                (matrix[1][0] - matrix[0][1]) / scale,
                0.25 * scale,
            ]

        diagonal = [matrix[0][0], matrix[1][1], matrix[2][2]]
        axis = diagonal.index(max(diagonal))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
            return [
                0.25 * scale,
                (matrix[0][1] + matrix[1][0]) / scale,
                (matrix[0][2] + matrix[2][0]) / scale,
                (matrix[2][1] - matrix[1][2]) / scale,
            ]
        if axis == 1:
            scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
            return [
                (matrix[0][1] + matrix[1][0]) / scale,
                0.25 * scale,
                (matrix[1][2] + matrix[2][1]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale,
            ]

        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        return [
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        ]

    def _prepare_log(self) -> None:
        self.result_log.parent.mkdir(parents=True, exist_ok=True)
        if not self.result_log.exists():
            with self.result_log.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["attempt", "status", "reason"])

    def _append_log(self, attempt: int, status: str, reason: str) -> None:
        with self.result_log.open("a", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow([attempt, status, reason])
        self.get_logger().info(f"Result logged to {self.result_log}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
