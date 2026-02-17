import os
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from anomaly_logging.msg import AnomalyLog


class AnomalyDetectionNode(Node)::
    def __init__(self):
        super().__init__('anomaly_detection')

        # Load config once on startup
        self.config = self._load_config()

        # Sprint-required timing variables (with safe defaults)
        self.artifact_frequency_seconds = self.config.get("artifact_frequency_seconds", 30)
        self.artifact_duration_seconds = self.config.get("artifact_duration_seconds", 10)
        self.api_frequency_seconds = self.config.get("api_frequency_seconds", 60)

        # Load topic names from config (fallback to defaults)
        self.raw_input_topic = self.config.get("raw_input_topic", "raw_input")
        #self.trimmed_output_topic = self.config.get("trimmed_output_topic", "trimmed_data")

        self.get_logger().info(
            "Loaded config: artifact_frequency_seconds=%s, artifact_duration_seconds=%s, api_frequency_seconds=%s",
            str(self.artifact_frequency_seconds),
            str(self.artifact_duration_seconds),
            str(self.api_frequency_seconds),
        )

        self.get_logger().info(
            "Topics configured: raw_input_topic=%s, trimmed_output_topic=%s",
            self.raw_input_topic,
            #self.trimmed_output_topic,
        )

        # Create subscription for raw input
        self.subscription = self.create_subscription(
            AnomalyLog,
            self.raw_input_topic,
            self.log_caching_callback,
            10
        )

        # Simple message counter for debugging
        self._msg_count = 0
        self.get_logger().info("AAD Manager node started.")

    def _load_config(self) -> dict:
        """
        Loads config.yaml from disk.

        Resolution order:
        1) AAD_CONFIG_PATH environment variable (useful for Docker/compose)
        2) config.yaml in the same folder as this script
        """
        env_path = os.getenv("AAD_CONFIG_PATH")
        if env_path and os.path.isfile(env_path):
            config_path = env_path
        else:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

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

    def log_caching_callback(self, msg) -> None:
        


def main(args=None) -> None:
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
