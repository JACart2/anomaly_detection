import rclpy
from rclpy.node import Node
from anomaly_msg.msg import AnomalyLog
import random
import struct
from sensor_msgs.msg import PointCloud2


class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        self.publisher = self.create_publisher(AnomalyLog, '/ai_anomaly_logging', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            10
        )

        self.lidar_data = None

        self.timer = self.create_timer(5.0, self.publish_data)

    def lidar_callback(self, data):
        self.lidar_data = data

    def publish_data(self):
        msg = AnomalyLog()
        msg.stamp = self.get_clock().now().to_msg()
        msg.node_name = "lidar_test_node"
        msg.source = "lidar"
        msg.description = "roof_lidar"
        msg.data_type = "PointCloud"
        msg.data = list(self.lidar_data.data)
        if self.lidar_data:
            self.publisher.publish(msg)
            self.get_logger().info("Published lidar data")

def main():
    rclpy.init()
    node = LidarTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
