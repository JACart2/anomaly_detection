import json
import os
import random
from datetime import datetime

import rclpy
from rclpy.node import Node
from anomaly_msg.msg import AnomalyMsg


class RawInputPublisherTester(Node):
    """Publishes realistic AnomalyMsg traffic so ADN has cache/raw data to bag."""

    def __init__(self):
        super().__init__("raw_input_publisher_tester")

        self.topic_name = os.getenv("AAD_RAW_INPUT_TOPIC", "/ai_anomaly_logging")
        self.publish_period = float(os.getenv("AAD_TEST_PUBLISH_PERIOD", "0.5"))
        self.frame_id = os.getenv("AAD_TEST_FRAME_ID", "aad_test_frame")
        self.node_name_value = os.getenv("AAD_TEST_SOURCE_NODE", "api_bag_test_source")

        self.publisher = self.create_publisher(AnomalyMsg, self.topic_name, 10)
        self.timer = self.create_timer(self.publish_period, self.publish_message)
        self.counter = 0

        self.get_logger().info(
            f"RawInputPublisherTester started. Publishing to {self.topic_name} every {self.publish_period:.2f}s"
        )

    def publish_message(self):
        now = self.get_clock().now()
        msg = AnomalyMsg()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.node_name = self.node_name_value

        # Alternate severities to create slightly different bagged samples.
        if self.counter % 3 == 0:
            msg.importance = AnomalyMsg.INFO
        elif self.counter % 3 == 1:
            msg.importance = AnomalyMsg.WARNING
        else:
            msg.importance = AnomalyMsg.ERROR

        msg.type = AnomalyMsg.TEXT
        msg.msg = (
            f"API bagging test event #{self.counter} | "
            f"ts={datetime.utcnow().isoformat()}Z | sensor_value={random.randint(10, 99)}"
        )
        msg.data_type = "json"

        payload = {
            "sequence": self.counter,
            "scenario": "api_artifact_bagging",
            "source": self.node_name_value,
            "note": "Synthetic raw topic data for bag validation",
        }
        msg.data = json.dumps(payload).encode("utf-8")

        self.publisher.publish(msg)
        self.get_logger().info(f"Published test AnomalyMsg #{self.counter}")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = RawInputPublisherTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
