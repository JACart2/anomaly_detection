"""
Bridges low-frequency camera context frames onto the anomaly logging topic.

Subscribes to the configured ZED camera topics and, on a throttled timer,
republishes the latest frame from each camera as an AnomalyMsg(type=IMAGE) on
raw_input_topic. This keeps camera context on the same topic as every other
anomaly log message, so recording raw_input_topic alone captures both.
"""
import os

import rclpy
import yaml
from anomaly_msg.msg import AnomalyMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image as ROSImage

DEFAULT_CAMERA_TOPICS = [
    "/zed_front/zed_node_0/rgb/color/rect/image",
    "/zed_rear/zed_node_1/rgb/color/rect/image",
]


def _friendly_source_label(topic: str) -> str:
    """Return a human-readable camera label for a configured topic."""
    lowered = topic.lower()
    if "front" in lowered:
        return "Front camera"
    if "rear" in lowered:
        return "Rear camera"
    return topic


class CameraContextPublisher(Node):
    """Republish the latest frame from each configured camera as an AnomalyMsg."""

    def __init__(self):
        super().__init__("camera_context_publisher")

        config = self._load_config()
        llm_config = config.get("llm", {})

        self.raw_input_topic = config.get("raw_input_topic", "/ai_anomaly_logging")

        configured_topics = llm_config.get("camera_topics", DEFAULT_CAMERA_TOPICS)
        if isinstance(configured_topics, str):
            configured_topics = [configured_topics]
        self.camera_topics = [
            str(topic).strip() for topic in configured_topics if str(topic).strip()
        ]

        self.publish_period_sec = max(
            0.5,
            float(llm_config.get("camera_bridge_publish_period_sec", 10.0)),
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._latest_frames: dict[str, ROSImage] = {}

        self.anomaly_pub = self.create_publisher(AnomalyMsg, self.raw_input_topic, 10)

        self.camera_subscriptions = [
            self.create_subscription(
                ROSImage,
                topic,
                lambda msg, topic=topic: self._camera_callback(msg, topic),
                best_effort_qos,
            )
            for topic in self.camera_topics
        ]

        self.publish_timer = self.create_timer(
            self.publish_period_sec, self._publish_latest_frames
        )

        self.get_logger().info(
            "Camera context publisher started with "
            f"raw_input_topic={self.raw_input_topic}, "
            f"camera_topics={self.camera_topics}, "
            f"publish_period_sec={self.publish_period_sec}"
        )

    def _camera_callback(self, msg: ROSImage, topic: str) -> None:
        """Cache the latest frame seen on a configured camera topic."""
        self._latest_frames[topic] = msg

    def _publish_latest_frames(self) -> None:
        """Republish the latest cached frame per camera as an AnomalyMsg."""
        for topic, image_msg in list(self._latest_frames.items()):
            anomaly = AnomalyMsg()
            anomaly.header = image_msg.header
            anomaly.node_name = self.get_name()
            anomaly.importance = AnomalyMsg.INFO
            anomaly.type = AnomalyMsg.IMAGE
            anomaly.msg = f"{_friendly_source_label(topic)} image received"
            anomaly.image = image_msg
            anomaly.data_type = ""
            anomaly.data = []
            self.anomaly_pub.publish(anomaly)

    def _load_config(self) -> dict:
        """Load config.yaml using the same resolution order as the AAD node."""
        env_path = os.getenv("AAD_CONFIG_PATH")
        if env_path and os.path.isfile(env_path):
            config_path = os.path.abspath(env_path)
        else:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        if not os.path.isfile(config_path):
            self.get_logger().warn(
                f"Config file not found at {config_path}. Using defaults."
            )
            return {}

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"Failed to load config: {e}")
            return {}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraContextPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
