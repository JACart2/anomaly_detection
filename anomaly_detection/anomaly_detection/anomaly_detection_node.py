import rclpy
from rclpy.node import Node
from anomaly_logging.msg import AnomalyLog
from pathlib import Path
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from datetime import datetime


class AnomalyDetectionNode(Node):

    def __init__(self):
        super().__init__('anomaly_detection')

        self.topic_name = "/ai_anomaly_logging"

        ################### ADD CACHE ####################

        # Create bags folder
        self.bag_dir = Path("bags")
        self.bag_dir.mkdir(exist_ok=True)

        # Create first bag immediately
        self.create_new_bag()

        # Subscriber
        self.create_subscription(AnomalyLog, self.topic_name, self.anomaly_logging, 10)

        # Rotate bag every 10 seconds
        self.bag_timer = self.create_timer(10.0, self.create_new_bag)

        # Check anomaly every 10 seconds
        self.create_timer(10.0, self.llm_callback)


    def create_new_bag(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = self.bag_dir / f"ai_anomaly_bag_{timestamp}"

        self.get_logger().info(f"Opening new bag: {bag_name}")

        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=str(bag_name), storage_id="sqlite3")
        converter_options = ConverterOptions("", "")

        self.writer.open(storage_options, converter_options)

        topic_info = TopicMetadata(
            id=0,
            name=self.topic_name,
            type='anomaly_logging/msg/AnomalyLog',
            serialization_format='cdr'
        )

        self.writer.create_topic(topic_info)

    def anomaly_logging(self, log_msg):
        self.get_logger().info("Convert data to JSON")

    def llm_callback(self):
        if not self.msgs:
            self.get_logger().info("No anomaly messages received yet.")
        else:
            self.get_logger().info(
                f"Processing {len(self.msgs)} anomaly messages..."
            )
            self.msgs.clear()

def main():
    rclpy.init()
    node = AnomalyDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
