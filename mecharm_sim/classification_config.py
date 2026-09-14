from pathlib import Path
from typing import Any, Dict, Iterable, List

import yaml


SUPPORTED_CLASSES = {"cube": "cube_bin", "cylinder": "cylinder_bin"}
REQUIRED_OBJECT_KEYS = {
    "model_name",
    "class_name",
    "grid_id",
    "initial_position",
    "attach_topic",
    "detach_topic",
    "table_attach_topic",
    "table_detach_topic",
}


def load_classification_config(path: str) -> Dict[str, Any]:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f"Classification config does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Classification config root must be a mapping")
    validate_classification_config(config)
    return config


def validate_classification_config(config: Dict[str, Any]) -> None:
    for key in ("topics", "grids", "bins", "objects", "controller"):
        if key not in config:
            raise ValueError(f"Classification config is missing '{key}'")

    topics = config["topics"]
    for key in ("detections", "command", "result"):
        if not isinstance(topics.get(key), str) or not topics[key].startswith("/"):
            raise ValueError(f"topics.{key} must be an absolute ROS topic")

    grids = config["grids"]
    bins = config["bins"]
    objects = config["objects"]
    if len(grids) != 6 or len(objects) != 6:
        raise ValueError("Fake classification requires exactly six grids and six objects")

    _require_unique(objects, "model_name")
    _require_unique(objects, "grid_id")
    for target in objects:
        missing = REQUIRED_OBJECT_KEYS.difference(target)
        if missing:
            raise ValueError(
                f"Object {target.get('model_name', '<unnamed>')} is missing {sorted(missing)}"
            )
        class_name = target["class_name"]
        grid_id = target["grid_id"]
        if class_name not in SUPPORTED_CLASSES:
            raise ValueError(f"Unsupported class_name '{class_name}'")
        if grid_id not in grids:
            raise ValueError(f"Unknown grid_id '{grid_id}'")
        if SUPPORTED_CLASSES[class_name] not in bins:
            raise ValueError(f"Missing destination bin for class '{class_name}'")
        _require_vector3(target["initial_position"], f"{target['model_name']}.initial_position")
        _require_vector3(grids[grid_id]["world_position"], f"{grid_id}.world_position")
        for topic_key in (
            "attach_topic",
            "detach_topic",
            "table_attach_topic",
            "table_detach_topic",
        ):
            if not str(target[topic_key]).startswith("/"):
                raise ValueError(f"{target['model_name']}.{topic_key} must be absolute")

    for bin_name, destination in bins.items():
        _require_vector3(destination.get("world_position"), f"{bin_name}.world_position")
        offsets = destination.get("drop_offsets", [[0.0, 0.0, 0.0]])
        if not offsets:
            raise ValueError(f"{bin_name}.drop_offsets must not be empty")
        for index, offset in enumerate(offsets):
            _require_vector3(offset, f"{bin_name}.drop_offsets[{index}]")


def objects_by_grid(config: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {str(item["grid_id"]): item for item in config["objects"]}


def add_vectors(left: Iterable[float], right: Iterable[float]) -> List[float]:
    return [float(a) + float(b) for a, b in zip(left, right)]


def flatten_ros_parameters(values: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """Flatten nested YAML dictionaries to ROS parameter dot notation."""
    flattened: Dict[str, Any] = {}
    for key, value in values.items():
        full_key = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flattened.update(flatten_ros_parameters(value, full_key))
        else:
            flattened[full_key] = value
    return flattened


def _require_vector3(value: Any, label: str) -> None:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must be a three-element list")
    try:
        [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numbers") from exc


def _require_unique(items: Iterable[Dict[str, Any]], key: str) -> None:
    values = [item.get(key) for item in items]
    if len(values) != len(set(values)):
        raise ValueError(f"Object field '{key}' must be unique")
