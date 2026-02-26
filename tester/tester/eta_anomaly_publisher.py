import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from anomaly_msg.msg import AnomalyMsg


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
        # Create an AnomalyMsg
        anomaly = AnomalyMsg()

        # Header
        anomaly.header.stamp = self.get_clock().now().to_msg()
        anomaly.header.frame_id = "eta_sensor_frame"

        # Metadata
        anomaly.publisher_name = self.get_name()
        anomaly.source_type = "eta_sensor"
        anomaly.sensor_info = "simulated_eta_sensor"
        anomaly.topic_name = '/eta'
        anomaly.data_type = "std_msgs/Float32"

        # Description
        anomaly.description = f"Received ETA value: {eta_msg.data}"

        # Optionally, store the value in data as bytes
        # Here we convert float to 4 bytes (float32) for demonstration
        import struct
        anomaly.data = list(struct.pack('f', eta_msg.data))

        # Leave image empty
        anomaly.image = None

        # Publish
        self.anomaly_pub.publish(anomaly)
        self.get_logger().info(f"Published anomaly for ETA: {eta_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ETAAnomalyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
