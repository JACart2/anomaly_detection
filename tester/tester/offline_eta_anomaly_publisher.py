import rclpy
from rclpy.node import Node
from anomaly_msg.msg import AnomalyMsg
import struct
import random


class ETAAnomalyPublisher(Node):
    def __init__(self):
        super().__init__('eta_anomaly_node_offline')

        # Publisher for anomaly messages
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )

        # Timer to simulate ETA input every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_fake_eta)

    def publish_fake_eta(self):
        # Simulate an ETA value
        eta_value = random.uniform(1.0, 20.0)

        anomaly = AnomalyMsg()

        # Header
        anomaly.header.stamp = self.get_clock().now().to_msg()
        anomaly.header.frame_id = "eta_sensor_frame"

        # Required fields
        anomaly.node_name = self.get_name()
        anomaly.importance = AnomalyMsg.INFO
        anomaly.type = AnomalyMsg.DATA

        # Human-readable message
        anomaly.msg = f"Simulated ETA value: {eta_value:.2f}"

        # DATA-specific fields
        anomaly.data_type = "std_msgs/Float32"
        anomaly.data = list(struct.pack('f', eta_value))

        # Publish
        self.anomaly_pub.publish(anomaly)
        self.get_logger().info(f"Published offline ETA anomaly: {eta_value:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ETAAnomalyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()