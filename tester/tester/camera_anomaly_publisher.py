import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from anomaly_msg.msg import AnomalyMsg


class CameraAnomalyPublisher(Node):
    def __init__(self):
        super().__init__('camera_anomaly_node')

        # Subscriber to the real camera topic
        self.camera_sub = self.create_subscription(
            Image,
            '/zed_rear/zed_node_1/rgb/color/rect/image',
            self.camera_callback,
            10
        )

        # Publisher for anomaly messages
        self.anomaly_pub = self.create_publisher(
            AnomalyMsg,
            '/ai_anomaly_logging',
            10
        )

    def camera_callback(self, img_msg: Image):
        anomaly = AnomalyMsg()

        # Copy header from image
        anomaly.header = img_msg.header

        # Required metadata
        anomaly.node_name = self.get_name()
        anomaly.importance = AnomalyMsg.INFO   # INFO level
        anomaly.type = AnomalyMsg.IMAGE        # IMAGE type

        # Message text (used even for image context description)
        anomaly.msg = "Front camera image received"

        # Attach image (since type = IMAGE)
        anomaly.image = img_msg

        # Not used for IMAGE type
        anomaly.data_type = ""
        anomaly.data = []

        # Publish
        self.anomaly_pub.publish(anomaly)
        self.get_logger().info("Published anomaly with image")


def main(args=None):
    rclpy.init(args=args)
    node = CameraAnomalyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
