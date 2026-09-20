import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "config" / "classification_fake.yaml"


def _config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def test_classification_files_exist():
    required = [
        "config/classification_fake.yaml",
        "worlds/classification_fake.sdf",
        "urdf/mecharm_270_classification.urdf.xacro",
        "launch/classification_fake.launch.py",
        "mecharm_sim/classification_config.py",
        "mecharm_sim/fake_vision.py",
        "mecharm_sim/classification_manager.py",
        "mecharm_sim/classification_pick_place.py",
        "UBUNTU_TODO.md",
    ]
    for relative_path in required:
        assert (ROOT / relative_path).is_file(), relative_path


def test_python_sources_parse_without_importing_ros():
    paths = [
        ROOT / "mecharm_sim" / "classification_config.py",
        ROOT / "mecharm_sim" / "fake_vision.py",
        ROOT / "mecharm_sim" / "classification_manager.py",
        ROOT / "mecharm_sim" / "classification_pick_place.py",
        ROOT / "launch" / "classification_fake.launch.py",
    ]
    for path in paths:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_configuration_has_six_unique_targets_and_two_class_bins():
    config = _config()
    objects = config["objects"]
    assert len(config["grids"]) == 6
    assert len(objects) == 6
    assert {item["class_name"] for item in objects} == {"cube", "cylinder"}
    assert [item["class_name"] for item in objects].count("cube") == 3
    assert [item["class_name"] for item in objects].count("cylinder") == 3
    assert len({item["model_name"] for item in objects}) == 6
    assert len({item["grid_id"] for item in objects}) == 6
    assert config["bins"]["cube_bin"]["class_name"] == "cube"
    assert config["bins"]["cylinder_bin"]["class_name"] == "cylinder"


def test_optional_grid_grasp_offsets_have_three_numeric_components():
    config = _config()
    for grid in config["grids"].values():
        offset = grid.get("grasp_target_offset", [0.0, 0.0, 0.0])
        assert len(offset) == 3
        assert all(isinstance(value, (int, float)) for value in offset)


def test_all_grids_have_calibrated_six_joint_grasp_targets():
    config = _config()
    for grid in config["grids"].values():
        target = grid["grasp_joint_target"]
        assert len(target) == 6
        assert all(isinstance(value, (int, float)) for value in target)


def test_classification_pick_uses_per_grid_grasp_offset():
    source = (
        ROOT / "mecharm_sim" / "classification_pick_place.py"
    ).read_text(encoding="utf-8")
    method = source.split("    def pick_object(", 1)[1].split("\n    def ", 1)[0]
    assert '"grasp_target_offset"' in method
    assert "above = add_vectors(" in method
    assert "grasp = add_vectors(" in method
    assert "add_vectors(measured_position, grasp_target_offset)" in method


def test_world_models_and_positions_match_configuration():
    config = _config()
    world = ET.parse(ROOT / "worlds" / "classification_fake.sdf").getroot()
    assert world.find("./world").attrib["name"] == "mecharm_classification_fake"
    for grid_id, grid in config["grids"].items():
        assert world.find(f".//model[@name='{grid_id}']") is not None
        target = next(item for item in config["objects"] if item["grid_id"] == grid_id)
        model = world.find(f".//model[@name='{target['model_name']}']")
        assert model is not None
        assert model.findtext("static", default="false") == "false"
        assert model.find("link[@name='object_link']") is not None
        pose = [float(value) for value in model.findtext("pose").split()[:3]]
        assert pose == target["initial_position"]
        assert grid["world_position"] == target["initial_position"]


def test_detachable_joint_topics_match_configuration():
    config = _config()
    world = ET.parse(ROOT / "worlds" / "classification_fake.sdf").getroot()
    robot = ET.parse(
        ROOT / "urdf" / "mecharm_270_classification.urdf.xacro"
    ).getroot()
    table = world.find(".//model[@name='table']")
    assert table is not None
    for target in config["objects"]:
        table_plugin = table.find(
            f"plugin[child_model='{target['model_name']}']"
        )
        assert table_plugin is not None
        assert table_plugin.findtext("child_link") == "object_link"
        assert table_plugin.findtext("attach_topic") == target["table_attach_topic"]
        assert table_plugin.findtext("detach_topic") == target["table_detach_topic"]

        gripper_plugin = robot.find(
            f".//plugin[child_model='{target['model_name']}']"
        )
        assert gripper_plugin is not None
        assert gripper_plugin.findtext("child_link") == "object_link"
        assert gripper_plugin.findtext("attach_topic") == target["attach_topic"]
        assert gripper_plugin.findtext("detach_topic") == target["detach_topic"]


def test_fake_vision_payload_does_not_assign_a_bin():
    source = (ROOT / "mecharm_sim" / "fake_vision.py").read_text(encoding="utf-8")
    payload_block = source.split("detections = [", 1)[1].split("now =", 1)[0]
    assert '"model_name"' in payload_block
    assert '"class_name"' in payload_block
    assert '"grid_id"' in payload_block
    assert '"confidence"' in payload_block
    assert "destination_bin" not in payload_block


def test_manager_and_controller_contract_is_present():
    manager = (ROOT / "mecharm_sim" / "classification_manager.py").read_text(
        encoding="utf-8"
    )
    controller = (
        ROOT / "mecharm_sim" / "classification_pick_place.py"
    ).read_text(encoding="utf-8")
    for state in (
        "WAIT_DETECTION",
        "SELECT_TARGET",
        "PICK",
        "PLACE",
        "RETURN",
        "NEXT_TARGET",
        "FINISHED",
    ):
        assert state in manager
    for method in ("pick_object", "place_object", "return_home"):
        assert f"def {method}" in controller


def test_failed_grasp_alignment_retreats_and_is_not_retried():
    config = _config()
    assert "alignment_failed" in config["manager"]["non_retryable_error_codes"]
    assert config["controller"]["moveit"]["precise_joint_timeout_sec"] < config["controller"]["action_timeout_sec"]
    assert config["controller"]["moveit"]["precise_max_velocity_rad_sec"] < config["controller"]["arm_max_velocity_rad_sec"]
    assert 0.0 < config["controller"]["moveit"]["calibrated_joint_tolerance_rad"] < 0.01
    assert config["controller"]["moveit"]["precise_retry_retreat_m"] > 0.0
    assert config["controller"]["moveit"]["precise_position_tolerance_m"] == config["controller"]["scene"]["grasp_tolerance_m"]
    assert config["controller"]["moveit"]["grasp_plan_candidates"] >= 2
    assert 0.0 < config["controller"]["moveit"]["max_grasp_joint_step_rad"] <= 1.0
    assert config["controller"]["initialization_action_timeout_sec"] > config["controller"]["action_timeout_sec"]

    controller = (
        ROOT / "mecharm_sim" / "classification_pick_place.py"
    ).read_text(encoding="utf-8")
    manager = (ROOT / "mecharm_sim" / "classification_manager.py").read_text(
        encoding="utf-8"
    )
    motion = (ROOT / "mecharm_sim" / "pick_place.py").read_text(
        encoding="utf-8"
    )
    assert 'return "alignment_failed"' in controller
    assert "error_code in self.non_retryable_error_codes" in manager
    assert "self.moveit_precise_retry_retreat_m" in motion
    assert "self.moveit_precise_joint_timeout_sec" in motion
    assert "cartesian_xy_tolerance_m=self.grasp_xy_tolerance_m" in motion
    assert "cartesian_z_tolerance_m=self.grasp_z_tolerance_m" in motion


def test_packaging_keeps_original_entrypoint_and_adds_classification_entries():
    setup_source = (ROOT / "setup.py").read_text(encoding="utf-8")
    for entrypoint in (
        "pick_place = mecharm_sim.pick_place:main",
        "fake_vision = mecharm_sim.fake_vision:main",
        "classification_manager = mecharm_sim.classification_manager:main",
        "classification_pick_place = mecharm_sim.classification_pick_place:main",
    ):
        assert entrypoint in setup_source

    launch_source = (
        ROOT / "launch" / "classification_fake.launch.py"
    ).read_text(encoding="utf-8")
    for reference in (
        "classification_fake.sdf",
        "mecharm_270_classification.urdf.xacro",
        "classification_fake.yaml",
        'executable="fake_vision"',
        'executable="classification_manager"',
        'executable="classification_pick_place"',
    ):
        assert reference in launch_source
