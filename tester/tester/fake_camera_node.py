import rclpy
from rclpy.node import Node
from anomaly_logging.msg import AnomalyLog
import random


class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        self.publisher = self.create_publisher(AnomalyLog, '/ai_anomaly_logging', 10)
        self.timer = self.create_timer(2.0, self.publish_data)

    def publish_data(self):
        msg = AnomalyLog()
        msg.stamp = self.get_clock().now().to_msg()
        msg.node_name = "fake_camera_node"
        msg.source = "camera"
        msg.description = "front_camera"
        msg.data_type = "image"
        msg.data = [random.randint(0, 255) for _ in range(100)]
        self.publisher.publish(msg)
        self.get_logger().info("Published fake camera anomaly data")


def main():
    rclpy.init()
    node = FakeCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
