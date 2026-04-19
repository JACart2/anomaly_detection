"""Dummy trigger to test modular trigger integration in AAD node.

Author: John Rosario Cruz
Version: 4/19/2026
"""
## ROS2 packages
import os
import yaml

from rclpy.node import Node
from std_msgs.msg import String as ROSString

class DummyTrigger(Node):
    """
    Description
    ------------
        A dummy trigger script for testing the ROS2 integration. This script does not perform any actual anomaly detection but simulates the behavior of a trigger by publishing a message to a specified topic at regular intervals.

    Attributes
    ----------
        publisher (obj): The ROS2 publisher that sends messages to the trigger input topic.

        timer (obj): A ROS2 timer that triggers the publish_message method at regular intervals.

    Methods:
    -------
        publish_message(): Publishes a dummy message to the trigger input topic.
    """
    def __init__(self):
        super().__init__('dummy_trigger')
        config = self._load_config()
        self.trigger_input_topic = config.get("trigger_input_topic", "/trigger_messages")
        self.publisher = self.create_publisher(ROSString, self.trigger_input_topic, 10)
        self.timer = self.create_timer(5, self.publish_message)  # Publish every 5 seconds

    def publish_message(self):
        msg = ROSString()
        msg.data = "This is a dummy trigger message."
        self.publisher.publish(msg)
        self.get_logger().info('Published dummy trigger message: "%s"' % msg.data)

    def _load_config(self) -> dict:
        env_path = os.getenv("AAD_CONFIG_PATH")
        if env_path and os.path.isfile(env_path):
            config_path = env_path
        else:
            config_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "..", "config.yaml")
            )

        if not os.path.isfile(config_path):
            self.get_logger().warn(f"Config file not found at {config_path}. Using defaults.")
            return {}

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                self.get_logger().warn("Config file loaded but is not a YAML mapping. Using defaults.")
                return {}
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load config file {config_path}: {e}. Using defaults.")
            return {}
