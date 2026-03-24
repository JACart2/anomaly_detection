## ROS2 packages
from tokenize import String

from rclpy.node import Node

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
        self.timer = self.create_timer(5.0, self.publish_message)  # Publish every 5 seconds

    def publish_message(self):
        msg = String()
        msg.data = "This is a dummy trigger message."
        self.get_logger().info('Published dummy trigger message: "%s"' % msg.data)