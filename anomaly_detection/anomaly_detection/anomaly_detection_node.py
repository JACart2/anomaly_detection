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
        self.msgs = []

        # Create bags folder
        self.bag_dir = Path("bags")
        self.bag_dir.mkdir(exist_ok=True)

        # Use timestamped bag folder to avoid overwrite
        bag_name = f"bags/ai_anomaly_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        storage_options = StorageOptions(uri=bag_name, storage_id="sqlite3")
        converter_options = ConverterOptions("", "")

        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        # Topic info
        self.topic_name = "/ai_anomaly_logging"
        topic_info = TopicMetadata(
            id=0,  # just start with 0
            name=self.topic_name,
            type='anomaly_logging/msg/AnomalyLog',  # **string** here
            serialization_format='cdr'
        )

        self.writer.create_topic(topic_info)

        # Subscriber
        self.create_subscription(AnomalyLog, self.topic_name, self.anomaly_logging, 10)

        # Timer for LLM processing
        self.llm_timer = self.create_timer(10, self.llm_callback)

    def anomaly_logging(self, log_msg):
        self.msgs.append(log_msg)
        timestamp = log_msg.stamp.sec * 1_000_000_000 + log_msg.stamp.nanosec
        serialized_msg = serialize_message(log_msg)
        self.writer.write(self.topic_name, serialized_msg, timestamp)

    def llm_callback(self):
        if not self.msgs:
            self.get_logger().info("No anomaly messages received yet.")
        else:
            # Process the messages with your LLM logic here
            self.get_logger().info(f"Processing {len(self.msgs)} anomaly messages...")
            self.msgs.clear()


def main():
    rclpy.init()
    node = AnomalyDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
