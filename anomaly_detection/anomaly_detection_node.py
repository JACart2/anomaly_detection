import os
import yaml
import json
from datetime import datetime
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from anomaly_msg.msg import AnomalyLog
from anomaly_detection.openai_call import call_openai
from anomaly_detection.response_handler import parse_llm_response
from anomaly_detection.StringRingBuffer import StringRingBuffer


class AnomalyDetectionNode(Node):
    """
    AAD ROS2 node:
      - Subscribes to AnomalyLog messages
      - Caches a bounded string representation of recent messages
      - Periodically calls the LLM/API
      - Parses the result via response_handler into a Decision
      - Publishes alerts to /aad/alerts when anomalies are detected
    """

    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()

        # Topics (configurable)
        self.raw_input_topic = self.config.get("raw_input_topic", "raw_input")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")

        # Sprint-required timing variables (with safe defaults)
        self.artifact_frequency_seconds = self.config.get("artifact_frequency_seconds", 30)
        self.artifact_duration_seconds = self.config.get("artifact_duration_seconds", 10)
        self.api_frequency_seconds = self.config.get("api_frequency_seconds", 60)

        # Cache sizing (configurable)
        self.cache_max_items = int(self.config.get("cache_max_items", 100))

        # Publisher for alerts
        self.alert_pub = self.create_publisher(String, self.alert_topic, 10)

        # Ring buffer cache
        self.queue = StringRingBuffer(max_items=self.cache_max_items)

        # Subscription for raw input
        self.subscription = self.create_subscription(
            AnomalyLog,
            self.raw_input_topic,
            self.log_caching_callback,
            10,
        )

        # Timer to trigger LLM pipeline
        self.timer = self.create_timer(self.api_frequency_seconds, self.llm_callback)

        # Debug counters / log throttling
        self._msg_count = 0
        self._log_every_n_msgs = int(self.config.get("log_every_n_msgs", 20))

        self.get_logger().info(
            "AAD node started with config: "
            f"raw_input_topic={self.raw_input_topic}, alert_topic={self.alert_topic}, "
            f"api_frequency_seconds={self.api_frequency_seconds}, "
            f"cache_max_items={self.cache_max_items}, "
            f"artifact_frequency_seconds={self.artifact_frequency_seconds}, "
            f"artifact_duration_seconds={self.artifact_duration_seconds}"
        )

    def _load_config(self) -> Dict[str, Any]:
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

    def log_caching_callback(self, msg: AnomalyLog) -> None:
        """
        Converts AnomalyLog into a compact, LLM-friendly string and pushes it into the ring buffer.

        IMPORTANT: We avoid str(msg) because AnomalyLog can include large binary fields
        (e.g., image/data arrays), which would explode token usage and slow the pipeline.
        """
        self._msg_count += 1
        if self._log_every_n_msgs > 0 and (self._msg_count % self._log_every_n_msgs == 0):
            self.get_logger().info(f"Received {self._msg_count} AnomalyLog messages")

        try:
            # Header stamp may not always be populated depending on publisher
            sec = getattr(getattr(getattr(msg, "header", None), "stamp", None), "sec", None)
            nsec = getattr(getattr(getattr(msg, "header", None), "stamp", None), "nanosec", None)
            frame_id = getattr(getattr(msg, "header", None), "frame_id", "")

            ts = "unknown"
            if sec is not None and nsec is not None:
                ts = f"{sec}.{nsec:09d}"

            # Avoid dumping large binary fields; include lengths only
            data_len = len(getattr(msg, "data", []))

            msg_str = (
                f"[t={ts} frame={frame_id}] "
                f"publisher={getattr(msg, 'publisher_name', '')} "
                f"source={getattr(msg, 'source_type', '')} "
                f"topic={getattr(msg, 'topic_name', '')} "
                f"type={getattr(msg, 'data_type', '')} "
                f"sensor={getattr(msg, 'sensor_info', '')} "
                f"desc={getattr(msg, 'description', '')} "
                f"data_len={data_len}"
            )

            self.queue.add(msg_str)

        except Exception as e:
            # Never let a bad message crash the node
            self.get_logger().warn(f"Failed to cache message safely: {e}")

    @staticmethod
    def to_jsonl(buffer) -> str:
        """
        Utility for artifact logging: convert list of JSON-serializable objects to JSONL.
        """
        return "\n".join(json.dumps(obj, separators=(",", ":")) for obj in buffer)

    def llm_callback(self) -> None:
        """
        Periodically called by a ROS2 timer.
          - snapshots cached context
          - calls API/LLM
          - parses via response_handler
          - publishes alert if anomaly
          - clears cache
        """
        raw_list = self.queue.snapshot()

        if not raw_list:
            self.get_logger().info("No anomaly messages received yet.")
            return

        # Use newlines to preserve message boundaries in context
        full_payload = "\n".join(raw_list)

        try:
            api_resp = call_openai(full_payload)
            decision = parse_llm_response(api_resp)

            # Logging outcomes
            if decision.raw is None:
                self.get_logger().error(
                    f"[AAD] Malformed API response. Fallback used. Summary={decision.summary}"
                )
            elif decision.action == "none" and decision.severity == "unknown":
                self.get_logger().warn(
                    f"[AAD] Validation failed. Fallback used. Summary={decision.summary}"
                )
            else:
                self.get_logger().info(
                    f"[AAD] Parsed response: anomaly={decision.anomaly}, "
                    f"severity={decision.severity}, action={decision.action}, "
                    f"summary={decision.summary}"
                )

            # Downstream behavior: publish alert if anomaly
            if decision.anomaly:
                alert = String()
                alert.data = (
                    f"[AAD ALERT] severity={decision.severity} "
                    f"action={decision.action} summary={decision.summary}"
                )
                self.alert_pub.publish(alert)

        except Exception as e:
            # Safe fallback: do nothing dangerous if API/handler fails
            self.get_logger().error(
                f"[AAD] Exception during API call/handling: {e}. Safe fallback: no action."
            )

        # Clear cache after processing so context does not re-send
        self.queue.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnomalyDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
