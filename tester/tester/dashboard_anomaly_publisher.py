"""Publish synthetic anomaly logs and alerts for dashboard smoke testing."""

from anomaly_msg.msg import AnomalyMsg

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DashboardAnomalyPublisher(Node):
    """Publish a repeating mix of anomaly severities on dashboard topics."""

    _SAMPLES = (
        (AnomalyMsg.INFO, 'info', 'Synthetic system heartbeat is healthy'),
        (
            AnomalyMsg.WARNING,
            'warning',
            'Synthetic sensor reading is approaching its limit',
        ),
        (
            AnomalyMsg.ERROR,
            'error',
            'Synthetic navigation subsystem failure detected',
        ),
    )

    def __init__(self) -> None:
        """Configure publishers and the periodic sample timer."""
        super().__init__('dashboard_anomaly_test_publisher')
        self.declare_parameter('logging_topic', '/ai_anomaly_logging')
        self.declare_parameter('alert_topic', '/aad/alerts')
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('source_node', 'dashboard_anomaly_test')

        logging_topic = self.get_parameter('logging_topic').value
        alert_topic = self.get_parameter('alert_topic').value
        publish_period = float(self.get_parameter('publish_period').value)
        self._source_node = str(self.get_parameter('source_node').value)

        if publish_period <= 0.0:
            raise ValueError('publish_period must be greater than zero')

        self._logging_publisher = self.create_publisher(
            AnomalyMsg, logging_topic, 10
        )
        self._alert_publisher = self.create_publisher(String, alert_topic, 10)
        self._sequence = 0
        self._timer = self.create_timer(publish_period, self._publish_sample)

        self.get_logger().info(
            f'Publishing dashboard test traffic every {publish_period:.2f}s: '
            f'logs={logging_topic}, alerts={alert_topic}'
        )

    def _publish_sample(self) -> None:
        importance, severity, description = self._SAMPLES[
            self._sequence % len(self._SAMPLES)
        ]
        timestamp = self.get_clock().now()
        event_text = f'[dashboard test #{self._sequence}] {description}'

        anomaly = AnomalyMsg()
        anomaly.header.stamp = timestamp.to_msg()
        anomaly.header.frame_id = 'dashboard_test'
        anomaly.node_name = self._source_node
        anomaly.importance = importance
        anomaly.type = AnomalyMsg.TEXT
        anomaly.msg = event_text
        anomaly.data_type = ''
        anomaly.data = []
        self._logging_publisher.publish(anomaly)

        alert = String()
        alert.data = (
            f'[AAD TEST] severity={severity} action=alert_admin '
            f'summary={event_text}'
        )
        self._alert_publisher.publish(alert)

        self.get_logger().info(
            f'Published sample #{self._sequence} with severity={severity}'
        )
        self._sequence += 1


def main(args=None) -> None:
    """Run the dashboard anomaly test publisher."""
    rclpy.init(args=args)
    node = DashboardAnomalyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
