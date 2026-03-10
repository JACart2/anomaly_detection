import rclpy
from rclpy.node import Node
from anomaly_msg.msg import AnomalyLog


class TestAnomalyPublisher(Node):
    def __init__(self):
        super().__init__('aad_test_publisher')
        self.publisher = self.create_publisher(AnomalyLog, '/ai_anomaly_logging', 10)
        self.timer = self.create_timer(2.0, self.publish_message)
        self.get_logger().info("Test anomaly publisher started.")

    def publish_message(self):
        msg = AnomalyLog()

        # Timestamp
        now = self.get_clock().now().to_msg()
        msg.stamp = now

        # Required fields from the real message
        msg.node_name = "test_navigation_node"
        msg.source = "navigation"
        msg.description = "Cart navigates off path"
        msg.topic_name = "/nav/status"
        msg.data_type = "text"
        msg.data = []

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