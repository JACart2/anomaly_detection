import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from anomaly_msg.msg import AnomalyMsg
import struct


class ETAAnomalyPublisher(Node):
    def __init__(self):
        super().__init__('eta_anomaly_node')

        # Subscriber to the /eta topic
        self.eta_sub = self.create_subscription(
            Float32,
            '/eta',
            self.eta_callback,
            10
        )

        # Publisher for anomaly messages
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )

    def eta_callback(self, eta_msg: Float32):
        anomaly = AnomalyMsg()

        # Header
        anomaly.header.stamp = self.get_clock().now().to_msg()
        anomaly.header.frame_id = "eta_sensor_frame"

        # Required fields
        anomaly.node_name = self.get_name()
        anomaly.importance = AnomalyMsg.INFO
        anomaly.type = AnomalyMsg.DATA

        # Human-readable message
        anomaly.msg = f"Received ETA value: {eta_msg.data}"

        # DATA-specific fields
        anomaly.data_type = "std_msgs/Float32"
        anomaly.data = list(struct.pack('f', eta_msg.data))

        # Publish
        self.anomaly_pub.publish(anomaly)
        self.get_logger().info(f"Published ETA anomaly: {eta_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ETAAnomalyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()