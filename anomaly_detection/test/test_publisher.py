import rclpy
from rclpy.node import Node
from anomaly_msg.msg import AnomalyMsg


class TestAnomalyPublisher(Node):
    __test__ = False  # This is a manual ROS publisher, not a pytest test case.

    def __init__(self):
        super().__init__('aad_test_publisher')
        self.publisher = self.create_publisher(AnomalyMsg, '/ai_anomaly_logging', 10)
        self.timer = self.create_timer(2.0, self.publish_message)
        self.get_logger().info("Test anomaly publisher started.")

    def publish_message(self):
        msg = AnomalyMsg()

        # Timestamp
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        # Required fields from the real message
        msg.node_name = "test_navigation_node"
        msg.importance = AnomalyMsg.ERROR
        msg.type = AnomalyMsg.TEXT
        msg.msg = "Cart navigates off path"

        self.publisher.publish(msg)
        self.get_logger().info("Published test anomaly message")

def main(args=None):
    rclpy.init(args=args)
    node = TestAnomalyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
