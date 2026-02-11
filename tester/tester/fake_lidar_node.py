import rclpy
from rclpy.node import Node
from anomaly_logging.msg import AnomalyLog
import random
import struct


class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')
        self.publisher = self.create_publisher(AnomalyLog, '/ai_anomaly_logging', 10)
        self.timer = self.create_timer(3.0, self.publish_data)

    def publish_data(self):
        msg = AnomalyLog()
        msg.stamp = self.get_clock().now().to_msg()
        msg.node_name = "fake_lidar_node"
        msg.source = "lidar"
        msg.description = "roof_lidar"
        msg.data_type = "pointcloud"
        fake_points = [random.uniform(0.0, 100.0) for _ in range(25)]
        msg.data = list(b''.join(struct.pack('f', p) for p in fake_points))
        self.publisher.publish(msg)
        self.get_logger().info("Published fake lidar anomaly data")


def main():
    rclpy.init()
    node = FakeLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
