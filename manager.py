"""
AAD Manager Node 

This file was derived from the EmotionDetection.py file stripping down any emotion detecting logic
and only retaining the ROS 2 node and class structure. 
This is more of a starting point for structuring the AAD Anomaly Detection Pipeline

Core responsibilities to implement later:
- Subscribe to selected raw topic(s)
- Perform lightweight processing/trimming of incoming data
- Publish to an AAD processed/trimmed topic
- Trigger artifact recording and API calls (config-driven)
"""

import rclpy
from rclpy.node import Node


class AADManager(Node):
    """
    AAD Manager Node

    Minimal ROS2 node scaffold. This will evolve into the central
    point for:
    - ingesting raw data from the cart stack
    - producing a trimmed/processed AAD topic
    - coordinating artifact recording + API triggers
    """

    def __init__(self):
        super().__init__('aad_manager_node')

        # Placeholders for ROS interfaces. Need to fill in once names are finalized. 
        self.subscription = None
        self.publisher = None

        # Uses get_logger instead of print so logs integrate with ROS tooling.
        self.get_logger().info("AAD Manager node started.")

    def listener_callback(self, msg) -> None:
        """
        Callback stub for incoming messages.

        Args:
            msg: ROS message received from the subscribed topic.
        """
        # Placeholder: implement trimming/transformation logic here.
        # Then publish to your processed topic via self.publisher.publish(processed_msg).
        self.get_logger().debug("Received message in listener_callback (stub).")


def main(args=None) -> None:
    """
    Entry point for running the node.
    """
    rclpy.init(args=args)
    node = AADManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
