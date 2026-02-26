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
            '/camera/image_raw',  # Replace with your camera topic
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
        # Create an AnomalyMsg
        anomaly = AnomalyMsg()

        # Copy the header from the camera image
        anomaly.header.stamp = img_msg.header.stamp
        anomaly.header.frame_id = img_msg.header.frame_id  # sensor frame

        # Metadata
        anomaly.publisher_name = self.get_name()
        anomaly.source_type = "camera"
        anomaly.sensor_info = "front_camera"
        anomaly.topic_name = '/camera/image_raw'
        anomaly.data_type = "sensor_msgs/Image"

        # Human-readable description
        anomaly.description = "This is the image from the front camera"

        # Attach the real image
        anomaly.image = img_msg

        # Leave raw data empty since we are sending structured Image
        anomaly.data = []

        # Publish the anomaly
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