import os
import yaml
import json

from std_msgs.msg import String
from anomaly_msg.msg import AnomalyMsg
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from anomaly_detection.response_handler import parse_llm_response
from anomaly_detection.StringRingBuffer import StringRingBuffer
from anomaly_detection.llm_client import LLMClient

## TODO document class and submethods.
## TODO use python logging class for our log messages to include the stack trace/line number
class AnomalyDetectionNode(Node):
    """
    AAD ROS2 node:
      - Subscribes to /ai_anomaly_logging (standardized logging topic)
      - Caches a bounded, LLM-friendly representation of AnomalyMsg
      - Periodically calls the LLM/API
      - Parses result via response_handler into a Decision
      - Publishes alerts to /aad/alerts when anomalies are detected
    """

    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()

        # Standard topic defaults
        self.raw_input_topic = self.config.get("raw_input_topic", "/ai_anomaly_logging")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")
        # Determine if trigger script is enabled (default to False if not specified)
        self.trigger_script_enabled = self.config.get("trigger_script", False)

        # Timing (safe defaults)
        self.api_frequency_seconds = float(self.config.get("api_frequency_seconds", 60.0))

        # Cache sizing
        self.cache_max_items = int(self.config.get("cache_max_items", 100))
        self.queue = StringRingBuffer(max_items=self.cache_max_items)

        # Optional INFO throttling to protect context window
        self.throttle_info = bool(self.config.get("throttle_info", True))
        self.info_min_period_sec = float(self.config.get("info_min_period_sec", 1.0))
        self._last_info_time_sec = 0.0
        if (self.trigger_script_enabled):
            self.get_logger().info("Trigger script enabled. Running install.sh")
            self._run_trigger_script_install()

        # Debug logging controls
        self._msg_count = 0
        self._log_every_n_msgs = int(self.config.get("log_every_n_msgs", 50))

        # Publisher for alerts
        self.alert_pub = self.create_publisher(String, self.alert_topic, 10)

        # Subscribe to standardized logging topic
        self.subscription = self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            10,
        )

        if (self.trigger_script_enabled):
            self._start_trigger_script()

        self.timer = self.create_timer(self.api_frequency_seconds, self.llm_callback)

        self.get_logger().info(
            "AAD node started with config: "
            f"raw_input_topic={self.raw_input_topic}, alert_topic={self.alert_topic}, "
            f"api_frequency_seconds={self.api_frequency_seconds}, cache_max_items={self.cache_max_items}, "
            f"throttle_info={self.throttle_info}, info_min_period_sec={self.info_min_period_sec}"
        )

    def _start_trigger_script(self) -> None:
        try:
            from anomaly_detection.anomaly_detection.anomaly_detection.trigger_script.trigger_script import TriggerScript
        except Exception as e:
            self.get_logger().error(f"Failed to import TriggerScript: {e}")
            return

        self.trigger_script_node = TriggerScript()
        self._trigger_script_monitor_stop = threading.Event()
        self._trigger_script_monitor_thread = threading.Thread(
            target=self._monitor_trigger_script_anomalies,
            daemon=True,
        )
        self._trigger_script_monitor_thread.start()

    def _monitor_trigger_script_anomalies(self) -> None:
        poll_seconds = 0.2
        while not self._trigger_script_monitor_stop.is_set():
            msg = self.trigger_script_node.consume_anomaly_message()
            if msg:
                self.queue.add(msg)
                self.llm_callback()
            else:
                time.sleep(poll_seconds)

    def _stop_trigger_script(self) -> None:
        if hasattr(self, "_trigger_script_monitor_stop"):
            self._trigger_script_monitor_stop.set()
        if hasattr(self, "_trigger_script_monitor_thread"):
            if self._trigger_script_monitor_thread.is_alive():
                self._trigger_script_monitor_thread.join(timeout=1.0)

    def _run_trigger_script_install(self) -> None:
        """
        Run trigger script installation (from __init__). Assumes install.sh is in the trigger_script subfolder and is executable.
        """
        script_path = os.path.join(os.path.dirname(__file__), "trigger_script", "install.sh")
        if not os.path.isfile(script_path):
            self.get_logger().error(f"install.sh not found at {script_path}")
            return

        try:
            subprocess.run(["bash", script_path], cwd=os.path.dirname(script_path), check=True)
            self.get_logger().info("install.sh completed.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"install.sh failed: {e}")

    def _load_config(self) -> dict:
        """
        Loads config.yaml from disk.

        Resolution order:
          1) AAD_CONFIG_PATH environment variable
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

    ## TODO is this used anywhere? if not lets remove.
    @staticmethod
    def to_jsonl(buffer) -> str:
        """Convert list of JSON-serializable objects to JSONL."""
        return "\n".join(json.dumps(obj, separators=(",", ":")) for obj in buffer)

    ## TODO do we want these below 3 methods (_now_sec, _importance_to_str, _type_to_str)? they are only used in one place, and it seems a bit cluttered.
    ## do we anticipate more complex formatting later on that would need these?
    def _now_sec(self) -> float:
        """Current node time in seconds (ROS clock)."""
        t = self.get_clock().now()
        return float(t.nanoseconds) / 1e9

    def _importance_to_str(self, importance: int) -> str:
        if importance == AnomalyMsg.ERROR:
            return "ERROR"
        if importance == AnomalyMsg.WARNING:
            return "WARNING"
        return "INFO"

    def _type_to_str(self, msg_type: int) -> str:
        if msg_type == AnomalyMsg.IMAGE:
            return "IMAGE"
        if msg_type == AnomalyMsg.DATA:
            return "DATA"
        return "TEXT"

    def _format_for_llm(self, m: AnomalyMsg) -> str:
        """
        Compact, LLM-friendly representation.

        IMPORTANT:
          - Do NOT dump raw image pixels or uint8[] data into LLM context.
          - Include only metadata and lengths for IMAGE/DATA.
        """
        # Header info
        ts = "unknown"
        ## TODO what is the purpose of these try/except blocks? do we want to allow malformed timestamp?
        try:
            ts = f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}"
        except Exception:
            pass

        frame_id = ""
        try:
            frame_id = m.header.frame_id
        except Exception:
            pass

        imp_s = self._importance_to_str(int(m.importance))
        type_s = self._type_to_str(int(m.type))

        base = (
            f"[t={ts} frame={frame_id}] "
            f"node={m.node_name} importance={imp_s} type={type_s} msg={m.msg}"
        )

        # IMAGE metadata (no pixels)
        if m.type == AnomalyMsg.IMAGE:
            try:
                img = m.image
                base += f" image={img.width}x{img.height} enc={img.encoding}"
            except Exception:
                base += " image=<unavailable>"

        # DATA metadata (no raw bytes)
        if m.type == AnomalyMsg.DATA:
            try:
                base += f" data_type={m.data_type} data_len={len(m.data)}"
            except Exception:
                base += " data=<unavailable>"

        return base

    def log_caching_callback(self, msg: AnomalyMsg) -> None:
        """
        Cache incoming AnomalyMsg strings.
        Throttle INFO messages (optional), but always keep WARNING/ERROR.
        """
        self._msg_count += 1
        if self._log_every_n_msgs > 0 and (self._msg_count % self._log_every_n_msgs == 0):
            self.get_logger().info(f"Received {self._msg_count} messages on {self.raw_input_topic}")

        # Optional INFO throttling
        if self.throttle_info and int(msg.importance) == AnomalyMsg.INFO:
            now = self._now_sec()
            if (now - self._last_info_time_sec) < self.info_min_period_sec:
                return
            self._last_info_time_sec = now

        try:
            self.queue.add(self._format_for_llm(msg))
        except Exception as e:
            self.get_logger().warn(f"Failed to cache message safely: {e}")

    def llm_callback(self) -> None:
        """
        Timer-driven processing:
          - snapshot cache
          - call LLM/API
          - parse with response handler
          - publish alert if anomaly
          - clear cache
        """
        raw_list = self.queue.snapshot()
        if not raw_list:
            self.get_logger().info("No cached log messages yet.")
            return

        # Preserve message boundaries for LLM readability
        full_payload = "\n".join(raw_list)

        try:
            # Process the messages with your LLM logic here
            llm = LLMClient()

            response = llm.chat(full_payload)
            decision = parse_llm_response(response)

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
                    f"severity={decision.severity}, action={decision.action}, summary={decision.summary}"
                )

            if decision.anomaly:
                alert = String()
                alert.data = (
                    f"[AAD ALERT] severity={decision.severity} "
                    f"action={decision.action} summary={decision.summary}"
                )
                self.alert_pub.publish(alert)

        except Exception as e:
            self.get_logger().error(
                f"[AAD] Exception during API call/handling: {e}. Safe fallback: no action."
            )

        self.queue.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if hasattr(node, "trigger_script_node"):
        executor.add_node(node.trigger_script_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_trigger_script()
        if hasattr(node, "trigger_script_node"):
            node.trigger_script_node.stop_event.set()
            node.trigger_script_node.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
