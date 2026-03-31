"""Central manager for AI Anomaly Detection node.

Handles API integration, trigger method integration, response handling,
and lightweight API artifact capture.

Reads std anomaly messages, caches an LLM-friendly representation,
periodically calls the LLM/API, parses results, and publishes alerts.

Each time the LLM is called, the node writes a JSON artifact containing:
- cached_data
- api_response

Author: AAD Team Spring 26'
Version: 3/31/2026
"""

import json
import os
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
import yaml
from anomaly_msg.msg import AnomalyMsg
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from anomaly_detection.llm_client import LLMClient
from anomaly_detection.response_handler import parse_llm_response
from anomaly_detection.StringRingBuffer import StringRingBuffer


class AnomalyDetectionNode(Node):
    """
    Description
    -----------
        Central manager for AI Anomaly Detection node.
        - Subscribes to /ai_anomaly_logging (standardized logging topic)
        - Caches a bounded, LLM-friendly representation of AnomalyMsg
        - Periodically calls the LLM/API
        - Parses result via response_handler into a Decision
        - Publishes alerts to /aad/alerts when anomalies are detected
        - Optionally creates API-triggered rosbag artifacts

    Attributes
    ----------
        config (dict): Configuration loaded from config.yaml.
        raw_input_topic (str): Topic subscribed to for anomaly messages.
        alert_topic (str): Topic used to publish alerts.
        trigger_script_enabled (bool): Whether trigger script is enabled.
        api_frequency_seconds (float): How often to call the API.
        cache_max_items (int): Max number of cached LLM-friendly messages.
        throttle_info (bool): Whether INFO messages are throttled.
        info_min_period_sec (float): Min time between INFO messages.
        log_every_n_msgs (int): How often to print received count logs.
        alert_pub (Publisher): Publisher for anomaly alerts.

        api_artifact_output_dir (str): Directory to store generated JSON artifacts.
    """

    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()

        # Standard topic defaults
        self.raw_input_topic = self.config.get("raw_input_topic", "/ai_anomaly_logging")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")

        # TODO: replace logic with ROS-native approach in sprint 3
        self.trigger_script_enabled = self.config.get("trigger_script", False)

        # Timing
        self.api_frequency_seconds = float(self.config.get("api_frequency_seconds", 60.0))

        # Cache sizing
        self.cache_max_items = int(self.config.get("cache_max_items", 100))
        self.queue = StringRingBuffer(max_items=self.cache_max_items)

        # Keep a lightweight recent raw snapshot for debugging / optional artifact content
        self.raw_history_max_items = int(self.config.get("raw_history_max_items", 50))
        self.raw_msg_history = deque(maxlen=self.raw_history_max_items)

        # Optional INFO throttling to protect context window
        self.throttle_info = bool(self.config.get("throttle_info", True))
        self.info_min_period_sec = float(self.config.get("info_min_period_sec", 1.0))
        self._last_info_time_sec = 0.0

        # Debug logging controls
        self._msg_count = 0
        self.log_every_n_msgs = int(self.config.get("log_every_n_msgs", 50))

        # JSON artifact output config
        self.api_artifact_output_dir = self.config.get(
            "api_artifact_output_dir",
            "/tmp/aad_api_artifacts",
        )

        # Trigger script setup
        if self.trigger_script_enabled:
            self.get_logger().info("Trigger script enabled. Running install.sh")
            self._run_trigger_script_install()

        # Publishers
        self.alert_pub = self.create_publisher(String, self.alert_topic, 10)

        # Subscription to standardized logging topic
        self.subscription = self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            10,
        )

        if self.trigger_script_enabled:
            self._start_trigger_script()

        self.create_timer(self.api_frequency_seconds, self.llm_callback)

        self.get_logger().info(
            "AAD node started with config: "
            f"raw_input_topic={self.raw_input_topic}, "
            f"alert_topic={self.alert_topic}, "
            f"api_frequency_seconds={self.api_frequency_seconds}, "
            f"cache_max_items={self.cache_max_items}, "
            f"throttle_info={self.throttle_info}, "
            f"info_min_period_sec={self.info_min_period_sec}, "
            f"api_artifact_output_dir={self.api_artifact_output_dir}"
        )

    def _generate_artifact_id(self) -> str:
        """Generate a unique artifact ID for a single API invocation."""
        stamp_ns = self.get_clock().now().nanoseconds
        return f"api_artifact_{stamp_ns}"

    def _write_api_artifact(self, artifact_id: str, cached_data: list[str], api_response: str) -> str | None:
        """
        Write a lightweight JSON artifact for a single LLM invocation.

        The artifact contains the cached data sent to the LLM and the raw
        response returned by the LLM.
        """
        try:
            os.makedirs(self.api_artifact_output_dir, exist_ok=True)
            artifact_path = os.path.join(
                self.api_artifact_output_dir,
                f"{artifact_id}.json",
            )

            payload = {
                "artifact_id": artifact_id,
                "timestamp_ns": self.get_clock().now().nanoseconds,
                "cached_data": cached_data,
                "api_response": api_response,
            }

            with open(artifact_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)

            return artifact_path

        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to write API artifact JSON: {e}"
            )
            return None

    def log_caching_callback(self, msg: AnomalyMsg) -> None:
        """
        Callback for incoming AnomalyMsg messages.
        Caches them in an LLM-friendly queue and stores a lightweight raw snapshot.
        """
        self._msg_count += 1
        if self.log_every_n_msgs > 0 and (self._msg_count % self.log_every_n_msgs == 0):
            self.get_logger().info(
                f"Received {self._msg_count} messages on {self.raw_input_topic}"
            )

        # Optional INFO throttling
        if self.throttle_info and int(msg.importance) == AnomalyMsg.INFO:
            now = self._now_sec()
            if (now - self._last_info_time_sec) < self.info_min_period_sec:
                return
            self._last_info_time_sec = now

        # Store a lightweight raw snapshot for debugging
        try:
            self.raw_msg_history.append(
                {
                    "node_name": msg.node_name,
                    "importance": int(msg.importance),
                    "type": int(msg.type),
                    "msg": msg.msg,
                    "data_type": msg.data_type,
                    "data_len": len(msg.data),
                    "stamp_sec": msg.header.stamp.sec,
                    "stamp_nanosec": msg.header.stamp.nanosec,
                    "frame_id": msg.header.frame_id,
                }
            )
        except Exception as e:
            self.get_logger().warn(
                f"Line {sys._getframe().f_lineno}: Failed to store raw message snapshot safely: {e}"
            )

        try:
            self.queue.add(self._format_for_llm(msg))
        except Exception as e:
            self.get_logger().warn(
                f"Line {sys._getframe().f_lineno}: Failed to cache message safely: {e}"
            )

    def llm_callback(self) -> None:
        """
        Timer-driven callback for processing cached log messages with the LLM.

        Flow
        ----
        - snapshot cache
        - call LLM/API
        - write a JSON artifact containing cached_data and api_response
        - parse with response handler
        - publish alert if anomaly
        - clear cache
        """
        raw_list = self.queue.snapshot()

        if not raw_list:
            return

        full_payload = "\n".join(raw_list)

        temp_api_response = None

        try:
            llm = LLMClient()
            response = llm.chat(full_payload)
            temp_api_response = response

        except Exception as e:
            self.get_logger().warn(
                f"[AAD] LLM call failed (likely missing API key). Using mock response for testing: {e}"
            )

            temp_api_response = json.dumps({
                "anomaly": False,
                "severity": "low",
                "action": "none",
                "summary": "Mock response used for artifact testing"
            })

        # Create artifact even if API failed
        artifact_id = self._generate_artifact_id()
        self._write_api_artifact(artifact_id, raw_list, temp_api_response)

        # Try parsing decision if possible
        try:
            decision = parse_llm_response(temp_api_response)

            if decision.anomaly:
                alert = String()
                alert.data = (
                    f"[AAD ALERT] severity={decision.severity} "
                    f"action={decision.action} summary={decision.summary}"
                )
                self.alert_pub.publish(alert)

        except Exception as e:
            self.get_logger().warn(
                f"[AAD] Could not parse decision during testing: {e}"
            )

        self.queue.clear()

    def _start_trigger_script(self) -> None:
        """
        Initializes and starts the trigger script if enabled.
        Runs the trigger script in a separate thread and monitors for anomaly messages.
        """
        try:
            from anomaly_detection.anomaly_detection.trigger_script.trigger_script import TriggerScript
        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to import TriggerScript: {e}"
            )
            return

        self.trigger_script_node = TriggerScript()
        self._trigger_script_monitor_stop = threading.Event()
        self._trigger_script_monitor_thread = threading.Thread(
            target=self._monitor_trigger_script_anomalies,
            daemon=True,
        )
        self._trigger_script_monitor_thread.start()

    def _monitor_trigger_script_anomalies(self) -> None:
        """
        Loop that runs in a separate thread to consume anomaly messages from the
        trigger script and add them to the cache for LLM processing.
        """
        # TODO: replace arbitrary polling with better sprint 3 logic
        poll_seconds = 0.2
        while not self._trigger_script_monitor_stop.is_set():
            msg = self.trigger_script_node.consume_anomaly_message()
            if msg:
                self.queue.add(msg)
                self.llm_callback()
            else:
                time.sleep(poll_seconds)

    def _stop_trigger_script(self) -> None:
        """Stops the trigger script monitoring thread."""
        if hasattr(self, "_trigger_script_monitor_stop"):
            self._trigger_script_monitor_stop.set()
        if hasattr(self, "_trigger_script_monitor_thread"):
            if self._trigger_script_monitor_thread.is_alive():
                self._trigger_script_monitor_thread.join(timeout=1.0)

    def _run_trigger_script_install(self) -> None:
        """
        Run trigger script installation (from __init__).
        Assumes install.sh is in the trigger_script subfolder and is executable.
        """
        script_path = os.path.join(os.path.dirname(__file__), "trigger_script", "install.sh")
        if not os.path.isfile(script_path):
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: install.sh not found at {script_path}"
            )
            return

        try:
            subprocess.run(
                ["bash", script_path],
                cwd=os.path.dirname(script_path),
                check=True,
            )
            self.get_logger().info("install.sh completed.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: install.sh failed: {e}"
            )

    def _load_config(self) -> dict:
        """
        Loads config.yaml from disk.

        Resolution order:
          1) AAD_CONFIG_PATH environment variable
          2) config.yaml in the same folder as this script

        Returns
        -------
            dict: Configuration dictionary. Empty if loading fails.
        """
        env_path = os.getenv("AAD_CONFIG_PATH")
        if env_path and os.path.isfile(env_path):
            config_path = env_path
        else:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        if not os.path.isfile(config_path):
            self.get_logger().warn(
                f"Line {sys._getframe().f_lineno}: Config file not found at "
                f"{config_path}. Using defaults."
            )
            return {}

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            if not isinstance(data, dict):
                self.get_logger().warn(
                    "Config file loaded but is not a YAML mapping. Using defaults."
                )
                return {}

            return data

        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to load config file "
                f"{config_path}: {e}. Using defaults."
            )
            return {}

    def _now_sec(self) -> float:
        """Current node time in seconds using the ROS clock."""
        t = self.get_clock().now()
        return float(t.nanoseconds) / 1e9

    def _importance_to_str(self, importance: int) -> str:
        """Convert an importance level to a string representation."""
        if importance == AnomalyMsg.ERROR:
            return "ERROR"
        if importance == AnomalyMsg.WARNING:
            return "WARNING"
        return "INFO"

    def _type_to_str(self, msg_type: int) -> str:
        """Convert a message type to a string representation."""
        if msg_type == AnomalyMsg.IMAGE:
            return "IMAGE"
        if msg_type == AnomalyMsg.DATA:
            return "DATA"
        return "TEXT"

    def _format_for_llm(self, m: AnomalyMsg) -> str:
        """
        Convert an AnomalyMsg into a compact, LLM-friendly string representation.

        Example
        -------
            [t=1234567890.123456789 frame=base_link]
            node=camera_driver importance=ERROR type=IMAGE
            msg="Camera feed frozen" image=640x480 enc=rgb8
        """
        ts = "unknown"
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

        if m.type == AnomalyMsg.IMAGE:
            try:
                img = m.image
                base += f" image={img.width}x{img.height} enc={img.encoding}"
            except Exception:
                base += " image=<unavailable>"

        if m.type == AnomalyMsg.DATA:
            try:
                base += f" data_type={m.data_type} data_len={len(m.data)}"
            except Exception:
                base += " data=<unavailable>"

        return base


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