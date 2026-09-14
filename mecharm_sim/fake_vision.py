import json
from pathlib import Path
from typing import Any, Dict

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mecharm_sim.classification_config import load_classification_config


class FakeVisionNode(Node):
    """Publish deterministic grid occupancy without camera or detector input."""

    def __init__(self) -> None:
        super().__init__("fake_vision")
        self.declare_parameter("config_file", "")
        config_path = self._resolve_config_path(
            str(self.get_parameter("config_file").value)
        )
        self.config = load_classification_config(config_path)

        topic = str(self.config["topics"]["detections"])
        vision_config: Dict[str, Any] = self.config.get("fake_vision", {})
        self.confidence = float(vision_config.get("confidence", 0.99))
        period_sec = float(vision_config.get("publish_period_sec", 1.0))
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("fake_vision.confidence must be between 0 and 1")
        if period_sec <= 0.0:
            raise ValueError("fake_vision.publish_period_sec must be positive")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, topic, qos)
        self.sequence = 0
        self.timer = self.create_timer(period_sec, self.publish_detections)
        self.get_logger().info(
            f"Fake Vision ready: publishing {len(self.config['objects'])} detections on {topic}"
        )

    def publish_detections(self) -> None:
        self.sequence += 1
        detections = [
            {
                "model_name": str(target["model_name"]),
                "class_name": str(target["class_name"]),
                "grid_id": str(target["grid_id"]),
                "confidence": self.confidence,
            }
            for target in self.config["objects"]
        ]
        now = self.get_clock().now().to_msg()
        payload = {
            "schema_version": 1,
            "sequence": self.sequence,
            "stamp": {"sec": now.sec, "nanosec": now.nanosec},
            "detections": detections,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.publisher.publish(message)
        self.get_logger().debug(
            f"Published Fake Vision frame {self.sequence} with {len(detections)} objects"
        )

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
    node = FakeVisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
