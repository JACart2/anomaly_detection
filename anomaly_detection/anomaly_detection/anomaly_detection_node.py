"""Central manager for AI Anomaly Detection node.

Handles API integration, trigger method integration, response handling,
and API-triggered artifact bagging.

Reads std anomaly messages, caches an LLM-friendly representation,
periodically calls the LLM/API, parses results, and publishes alerts.

When enabled through config, each API invocation also creates a rosbag
artifact containing:
- trigger event
- cache snapshot
- API response
- raw input topic data during the capture window

Author: AAD Team Spring 26'
Version: 3/26/2026
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

        api_artifact_bagging_enabled (bool): Enables API-triggered rosbagging.
        api_artifact_output_dir (str): Directory to store generated bag files.
        api_artifact_window_seconds (float): Recording window after API response.
        api_bag_trigger_pub (Publisher): Publishes bag-trigger events.
        api_cache_pub (Publisher): Publishes cache snapshots.
        api_response_pub (Publisher): Publishes API responses.
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

        # Artifact bagging config
        self.api_artifact_bagging_enabled = bool(
            self.config.get("api_artifact_bagging_enabled", False)
        )
        self.api_artifact_output_dir = self.config.get(
            "api_artifact_output_dir",
            "/tmp/aad_api_bags",
        )
        self.api_artifact_window_seconds = float(
            self.config.get("api_artifact_window_seconds", 3.0)
        )
        self.api_artifact_startup_delay_seconds = float(
            self.config.get("api_artifact_startup_delay_seconds", 0.5)
        )

        self.api_artifact_trigger_topic = self.config.get(
            "api_artifact_trigger_topic",
            "/aad/api_bag_trigger",
        )
        self.api_artifact_cache_topic = self.config.get(
            "api_artifact_cache_topic",
            "/aad/cache_snapshot",
        )
        self.api_artifact_response_topic = self.config.get(
            "api_artifact_response_topic",
            "/aad/api_response",
        )

        # Trigger script setup
        if self.trigger_script_enabled:
            self.get_logger().info("Trigger script enabled. Running install.sh")
            self._run_trigger_script_install()

        # Publishers
        self.alert_pub = self.create_publisher(String, self.alert_topic, 10)
        self.api_bag_trigger_pub = self.create_publisher(
            String, self.api_artifact_trigger_topic, 10
        )
        self.api_cache_pub = self.create_publisher(
            String, self.api_artifact_cache_topic, 10
        )
        self.api_response_pub = self.create_publisher(
            String, self.api_artifact_response_topic, 10
        )

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
            f"api_artifact_bagging_enabled={self.api_artifact_bagging_enabled}, "
            f"api_artifact_output_dir={self.api_artifact_output_dir}, "
            f"api_artifact_window_seconds={self.api_artifact_window_seconds}"
        )

    def _generate_artifact_id(self) -> str:
        """Generate a unique artifact ID for a single API invocation."""
        stamp_ns = self.get_clock().now().nanoseconds
        return f"api_artifact_{stamp_ns}"

    def _start_api_artifact_bag_recording(self, artifact_id: str):
        """
        Start a short-lived rosbag recording process for API artifact capture.

        Returns
        -------
            tuple[subprocess.Popen | None, str | None]
                The process handle and bag output path.
        """
        try:
            os.makedirs(self.api_artifact_output_dir, exist_ok=True)
            bag_path = os.path.join(self.api_artifact_output_dir, artifact_id)

            topics = [
                self.raw_input_topic,
                self.api_artifact_trigger_topic,
                self.api_artifact_cache_topic,
                self.api_artifact_response_topic,
            ]

            cmd = ["ros2", "bag", "record", "-o", bag_path] + topics

            self.get_logger().info(
                f"Starting API artifact bag recording: {' '.join(cmd)}"
            )

            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return proc, bag_path

        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to start rosbag recording: {e}"
            )
            return None, None

    def _stop_api_artifact_bag_recording(self, proc) -> None:
        """Stop a rosbag recording process safely."""
        if proc is None:
            return

        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "rosbag record did not stop in time. Killing process."
            )
            proc.kill()
        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to stop rosbag recording cleanly: {e}"
            )

    def _publish_api_trigger(self, artifact_id: str) -> None:
        """Publish an API invocation trigger message."""
        msg = String()
        msg.data = json.dumps(
            {
                "artifact_id": artifact_id,
                "event": "api_invoked",
                "timestamp_ns": self.get_clock().now().nanoseconds,
            }
        )
        self.api_bag_trigger_pub.publish(msg)

    def _publish_cache_snapshot(self, artifact_id: str, raw_list: list[str]) -> None:
        """Publish the current cache snapshot to a topic for bag capture."""
        msg = String()
        msg.data = json.dumps(
            {
                "artifact_id": artifact_id,
                "timestamp_ns": self.get_clock().now().nanoseconds,
                "cached_data": raw_list,
            }
        )
        self.api_cache_pub.publish(msg)

    def _publish_api_response(self, artifact_id: str, response: str, decision) -> None:
        """Publish the API response and parsed decision to a topic for bag capture."""
        msg = String()
        msg.data = json.dumps(
            {
                "artifact_id": artifact_id,
                "timestamp_ns": self.get_clock().now().nanoseconds,
                "response": response,
                "decision": {
                    "anomaly": decision.anomaly,
                    "severity": decision.severity,
                    "action": decision.action,
                    "summary": decision.summary,
                },
            }
        )
        self.api_response_pub.publish(msg)

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
        - optionally start artifact bag recording
        - publish trigger event
        - call LLM/API
        - parse with response handler
        - publish cache snapshot and API response for artifact capture
        - publish alert if anomaly
        - stop bag recording
        - clear cache
        """
        raw_list = self.queue.snapshot()
        if not raw_list:
            self.get_logger().info("No cached log messages yet.")
            return

        full_payload = "\n".join(raw_list)

        artifact_id = None
        bag_proc = None
        bag_path = None

        try:
            if self.api_artifact_bagging_enabled:
                artifact_id = self._generate_artifact_id()
                bag_proc, bag_path = self._start_api_artifact_bag_recording(artifact_id)

                if bag_proc is not None:
                    time.sleep(self.api_artifact_startup_delay_seconds)
                    self._publish_api_trigger(artifact_id)

            llm = LLMClient()
            response = llm.chat(full_payload)
            self.get_logger().info(f"Model responded: {response}")

            decision = parse_llm_response(response)

            if decision.raw is None:
                self.get_logger().error(
                    f"Line {sys._getframe().f_lineno}: [AAD] Malformed API response. "
                    f"Fallback used. Summary={decision.summary}"
                )
            elif decision.action == "none" and decision.severity == "unknown":
                self.get_logger().warn(
                    f"Line {sys._getframe().f_lineno}: [AAD] Validation failed. "
                    f"Fallback used. Summary={decision.summary}"
                )
            else:
                self.get_logger().info(
                    f"[AAD] Parsed response: anomaly={decision.anomaly}, "
                    f"severity={decision.severity}, action={decision.action}, "
                    f"summary={decision.summary}"
                )

            if self.api_artifact_bagging_enabled and artifact_id is not None and bag_proc is not None:
                self._publish_cache_snapshot(artifact_id, raw_list)
                self._publish_api_response(artifact_id, response, decision)

                # Keep recording briefly so the bag definitely captures the published topics
                time.sleep(self.api_artifact_window_seconds)

            if decision.anomaly:
                alert = String()
                alert.data = (
                    f"[AAD ALERT] severity={decision.severity} "
                    f"action={decision.action} summary={decision.summary}"
                )
                self.alert_pub.publish(alert)

        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: [AAD] Exception during API "
                f"call/handling: {e}. Safe fallback: no action."
            )

        finally:
            if bag_proc is not None:
                self._stop_api_artifact_bag_recording(bag_proc)
                if bag_path is not None:
                    self.get_logger().info(f"API artifact bag saved to: {bag_path}")

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