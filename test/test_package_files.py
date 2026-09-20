from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_required_files_exist():
    required = [
        "package.xml",
        "setup.py",
        "launch/sim.launch.py",
        "urdf/mecharm_270_pick_place.urdf.xacro",
        "config/ros2_controllers.yaml",
        "config/pick_place_poses.yaml",
        "worlds/pick_place.sdf",
        "mecharm_sim/pick_place.py",
        "docs/development.md",
    ]
    for relative_path in required:
        assert (ROOT / relative_path).exists(), relative_path


def test_target_ball_is_dynamic_but_cannot_roll_on_table():
    world = ET.parse(ROOT / "worlds/pick_place.sdf").getroot()
    ball = world.find(".//model[@name='target_ball']")
    table = world.find(".//model[@name='table']")

    assert ball is not None
    assert table is not None
    assert ball.findtext("static", default="false") == "false"
    assert float(ball.findtext("link/velocity_decay/angular")) >= 50.0
    assert float(ball.findtext("link/collision/surface/friction/ode/mu")) >= 5.0
    assert float(ball.findtext("link/collision/surface/friction/ode/mu2")) >= 5.0
    support = ball.find("link/collision[@name='ball_support_collision']")
    assert support is not None
    assert float(support.findtext("geometry/cylinder/radius")) <= 0.005
    assert ball.find("link/collision/geometry/sphere") is None
    table_lock = table.find("plugin[child_model='target_ball']")
    assert table_lock is not None
    assert table_lock.findtext("attach_topic") == "/target_ball_table/attach"
    assert table_lock.findtext("detach_topic") == "/target_ball_table/detach"


def test_prepare_robot_releases_and_resets_ball_before_motion():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _prepare_robot(self) -> None:", 1)[1]
    method = method.split("\n    def ", 1)[0]

    detach = method.index("self._set_detachable_joint(False)")
    reset = method.index("self._set_model_pose(self.ball_position)")
    gripper = method.index('self._move_gripper("open", self.gripper_open)')
    home = method.index('self._move_arm("home")')

    assert detach < reset < gripper < home


def test_moveit_execution_tracks_full_trajectory():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    publish_method = source.split("    def _publish_arm_trajectory(", 1)[1]
    publish_method = publish_method.split("\n    def ", 1)[0]
    follow_method = source.split("    def _follow_arm_trajectory(", 1)[1]
    follow_method = follow_method.split("\n    def ", 1)[0]

    assert "if len(trajectory.points) > 1:" in publish_method
    assert "self._follow_arm_trajectory(" in publish_method
    assert "trajectory," in publish_method
    assert "for point in trajectory.points:" in follow_method


def test_moveit_grasp_is_checked_before_closing():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _pick_ball(self) -> None:", 1)[1]
    method = method.split("\n    def ", 1)[0]

    alignment = method.index("self._ensure_grasp_alignment(ball_position)")
    close = method.index('self._move_gripper("closed", self.gripper_closed)')
    attach = method.index("self._set_detachable_joint(True)")
    unlock = method.index("self._set_table_ball_lock(False)")

    assert alignment < close < attach < unlock


def test_moveit_targets_real_gripper_center_and_locks_wrist():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()

    assert "constraint.target_point_offset.x" in source
    assert "self._estimate_grasp_center_position()" in source
    assert "request.path_constraints = self._wrist_constraints()" in source
    assert "goal.joint_constraints = self._wrist_constraints().joint_constraints" in source


def test_precise_grasp_selects_safe_candidates_and_replans_stalled_descents():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _move_pose_precisely(", 1)[1]
    method = method.split("\n    def ", 1)[0]

    assert "candidate_count=self.moveit_grasp_plan_candidates" in method
    assert "max_joint_step_rad=self.moveit_max_grasp_joint_step_rad" in method
    assert "except (RuntimeError, TimeoutError)" in method
    assert "replanning after retreat" in method
    assert "calibrated_joint_target" in method
    assert "self._move_joint_target(" in method


def test_low_grasp_completion_uses_short_timeout_not_global_action_timeout():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _move_pose_precisely(", 1)[1]
    method = method.split("\n    def ", 1)[0]

    assert "completion_timeout_sec=self.moveit_precise_joint_timeout_sec" in method
    assert "completion_max_velocity_rad_sec=self.moveit_precise_max_velocity" in method
    assert "cartesian_target=position" in method

    drive = source.split("    def _drive_arm_to(", 1)[1]
    drive = drive.split("\n    def ", 1)[0]
    assert "cartesian_target is None" in drive
    assert "min(self.arm_max_velocity, float(max_velocity_rad_sec))" in drive


def test_moveit_release_is_blocked_until_ball_reaches_plate():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _place_ball(self) -> None:", 1)[1]
    method = method.split("\n    def ", 1)[0]

    verify = method.index("self._move_ball_to_release_target()")
    open_gripper = method.index('self._move_gripper("open", self.gripper_open)')
    detach = method.index("self._set_detachable_joint(False)")

    assert verify < open_gripper < detach
