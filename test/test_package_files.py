from pathlib import Path


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
    assert "self._follow_arm_trajectory(trajectory)" in publish_method
    assert "for point in trajectory.points:" in follow_method


def test_moveit_grasp_is_checked_before_closing():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _pick_ball(self) -> None:", 1)[1]
    method = method.split("\n    def ", 1)[0]

    alignment = method.index("self._ensure_grasp_alignment(ball_position)")
    close = method.index('self._move_gripper("closed", self.gripper_closed)')
    attach = method.index("self._set_detachable_joint(True)")

    assert alignment < close < attach


def test_moveit_release_is_blocked_until_ball_reaches_plate():
    source = (ROOT / "mecharm_sim/pick_place.py").read_text()
    method = source.split("    def _place_ball(self) -> None:", 1)[1]
    method = method.split("\n    def ", 1)[0]

    verify = method.index("self._move_ball_to_release_target()")
    open_gripper = method.index('self._move_gripper("open", self.gripper_open)')
    detach = method.index("self._set_detachable_joint(False)")

    assert verify < open_gripper < detach
