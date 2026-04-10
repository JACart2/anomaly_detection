"""UI payload processor node.

Subscribes to a UI-facing JSON topic, converts payloads into the standardized
AnomalyMsg type, and republishes them on the core anomaly logging topic.

Author: AAD Team Spring 26'
Version: 4/10/2026
"""
import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as ROSString
from sensor_msgs.msg import Image

from anomaly_msg.msg import AnomalyMsg


class UIPayloadProcessorNode(Node):
    """
    Description
    -----------
        Converts UI JSON payloads into AnomalyMsg and republishes them.
        - Subscribes to /ai_anomaly_logging_ui for raw JSON
        - Parses payload into AnomalyMsg fields
        - Publishes standardized messages to /ai_anomaly_logging

    Attributes
    ----------
        ui_topic (str): UI payload topic to subscribe to.

        output_topic (str): Standardized logging topic to publish to.

        default_frame_id (str): Fallback frame_id if missing in payload header.

        default_node_name (str): Fallback node_name if missing in payload.

        subscription (Subscription): ROS subscription for UI payloads.

        publisher (Publisher): ROS publisher for AnomalyMsg output.

    Methods
    -------
        ui_payload_callback(msg: ROSString):
            Parses JSON payloads and republishes as AnomalyMsg.
    """

    def __init__(self) -> None:
        """Initialize subscriptions, publisher, and defaults."""
        super().__init__("ui_payload_processor")

        self.ui_topic = "ai_anomaly_logging_ui"
        self.output_topic = "ai_anomaly_logging"
        self.default_frame_id = "ui"
        self.default_node_name = "ui_payload"

        self.subscription = self.create_subscription(
            ROSString,
            self.ui_topic,
            self.ui_payload_callback,
            10,
        )
        self.publisher = self.create_publisher(AnomalyMsg, self.output_topic, 10)

    @staticmethod
    def _safe_int(value: Any, default: Optional[int]) -> Optional[int]:
        """Convert value to int, returning default on conversion errors."""
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _apply_header(self, anomaly: AnomalyMsg, header: Dict[str, Any]) -> None:
        """Apply header fields from payload, using node time when missing."""
        stamp = header.get("stamp") if isinstance(header, dict) else None
        if isinstance(stamp, dict) and "sec" in stamp and "nanosec" in stamp:
            sec = self._safe_int(stamp.get("sec"), None)
            nanosec = self._safe_int(stamp.get("nanosec"), None)
            if sec is not None and nanosec is not None:
                anomaly.header.stamp.sec = sec
                anomaly.header.stamp.nanosec = nanosec
            else:
                anomaly.header.stamp = self.get_clock().now().to_msg()
        else:
            anomaly.header.stamp = self.get_clock().now().to_msg()

        frame_id = header.get("frame_id") if isinstance(header, dict) else None
        if isinstance(frame_id, str) and frame_id:
            anomaly.header.frame_id = frame_id
        else:
            anomaly.header.frame_id = self.default_frame_id

    def _apply_image(self, anomaly: AnomalyMsg, image_payload: Any) -> None:
        """Populate the Image field when the payload includes an image."""
        if not isinstance(image_payload, dict):
            return

        img = Image()
        img.height = self._safe_int(image_payload.get("height"), 0)
        img.width = self._safe_int(image_payload.get("width"), 0)
        encoding = image_payload.get("encoding")
        if isinstance(encoding, str):
            img.encoding = encoding
        img.is_bigendian = self._safe_int(image_payload.get("is_bigendian"), 0)
        img.step = self._safe_int(image_payload.get("step"), 0)

        data = image_payload.get("data")
        if isinstance(data, list):
            img.data = [self._safe_int(x, 0) & 0xFF for x in data]
        anomaly.image = img

    def _build_anomaly_msg(self, payload: Dict[str, Any]) -> AnomalyMsg:
        """Build an AnomalyMsg from a JSON dictionary payload."""
        anomaly = AnomalyMsg()

        header = payload.get("header", {})
        self._apply_header(anomaly, header)

        node_name = payload.get("node_name")
        if isinstance(node_name, str) and node_name:
            anomaly.node_name = node_name
        else:
            anomaly.node_name = self.default_node_name

        anomaly.importance = self._safe_int(
            payload.get("importance"),
            AnomalyMsg.INFO,
        )
        anomaly.type = self._safe_int(payload.get("type"), AnomalyMsg.TEXT)

        msg = payload.get("msg")
        anomaly.msg = msg if isinstance(msg, str) else ""

        data_type = payload.get("data_type")
        anomaly.data_type = data_type if isinstance(data_type, str) else ""

        data = payload.get("data")
        if isinstance(data, list):
            anomaly.data = [self._safe_int(x, 0) & 0xFF for x in data]
        else:
            anomaly.data = []

        if anomaly.type == AnomalyMsg.IMAGE:
            self._apply_image(anomaly, payload.get("image"))

        return anomaly

    def ui_payload_callback(self, msg: ROSString) -> None:
        """ROS callback that validates JSON and publishes AnomalyMsg."""
        raw_data = msg.data
        try:
            payload = json.loads(raw_data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON on {self.ui_topic}: {exc} | payload={raw_data}")
            return

        if not isinstance(payload, dict):
            self.get_logger().error(f"Expected JSON object on {self.ui_topic}, got {type(payload).__name__}")
            return

        anomaly_msg = self._build_anomaly_msg(payload)
        self.publisher.publish(anomaly_msg)


def main(args=None) -> None:
    """Entry point for the UI payload processor node."""
    rclpy.init(args=args)
    node = UIPayloadProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
