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
import importlib.util

import json
import os
import subprocess

from std_msgs.msg import String as ROSString
import sys
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

        raw_input_topic (str): The ROS topic to subscribe to for raw anomaly messages. Default: /ai_anomaly_logging.

        alert_topic (str): The ROS topic to publish alerts to when anomalies are detected. Default: /aad/alerts.

        trigger_script_enabled (bool): Whether the trigger script is enabled. Default: False.

        api_frequency_seconds (float): How often to call the LLM/API in seconds. Default: 60.0.

        cache_max_items (int): Maximum number of messages to keep in the cache for LLM processing. Default: 100.

        throttle_info (bool): Whether to throttle INFO messages to protect LLM context. Default: True.

        info_min_period_sec (float): Minimum period in seconds between INFO messages if throttling is enabled. Default: 1.0.

        log_every_n_msgs (int): How often to log received messages for debugging. Default: 50.

        alert_pub (Publisher): ROS publisher for anomaly alerts.
    
    Methods
    -------
        log_caching_callback(msg: AnomalyMsg):
            Callback for incoming AnomalyMsg messages. Caches them in a StringRingBuffer after formatting for LLM. Optionally throttles INFO messages.
    
        llm_callback():
            Timer-driven callback that processes cached messages with the LLM, parses the response, and publishes alerts if anomalies are detected. 
            Clears the cache after processing.

        trigger_message_callback():
            Triggered when trigger_input_topic receives a message, adds to queue, triggers LLM call (since anomaly was found by trigger).

        _run_trigger_script_install():
            Run install.sh of respective trigger scripts and import modules to self.trigger_nodes.
            
        _load_config():
            Loads configuration from a YAML file. Resolution order:
              AAD_CONFIG_PATH if set, config.yaml if not.

        _now_sec():
            Helper method to get the current node time in seconds (using ROS clock).

        _importance_to_str(importance: int):
            Helper method to convert AnomalyMsg importance integer to a string representation (INFO, WARNING, ERROR).
        
        _type_to_str(msg_type: int):
            Helper method to convert AnomalyMsg type integer to a string representation (TEXT, IMAGE, DATA).

        _format_for_llm(m: AnomalyMsg):
            Converts an AnomalyMsg into a compact, LLM-friendly string representation.

        api_artifact_output_dir (str): Directory to store generated JSON artifacts.
    
    """


    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()
        self.llm_local = bool(self.config.get("llm", {}).get("local", False))

        # Standard topic defaults
        self.trigger_input_topic = self.config.get("trigger_input_topic", "/trigger_messages")
        self.raw_input_topic = self.config.get("raw_input_topic", "/ai_anomaly_logging")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")

        ## Install deps for chosen triggers & start them
        self.triggers = self.config.get("trigger_scripts", [])
        self.trigger_nodes = []
        for trigger in self.triggers:
            self.get_logger().info(f"Configured trigger script: {trigger}")
            ## create list of node objects to add to executor in main.
            response = self._run_trigger_script_install(trigger)
            if response != None:
                self.trigger_nodes.append(response)
            else:
                self.get_logger().error(f"Line {sys._getframe().f_lineno}: Unable to load trigger script from AAD node")

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


        # Publisher for alerts
        self.alert_pub = self.create_publisher(ROSString, self.alert_topic, 10)
        # Added for the config tests
        self.decision_pub = self.create_publisher(ROSString, "/aad/decisions", 10)

        # JSON artifact output config
        self.api_artifact_output_dir = self.config.get(
            "api_artifact_output_dir",
            "/tmp/aad_api_bags",
        )

        # Subscription to standardized logging topic
        self.subscription = self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            10,
        )
        

        self.create_subscription(
            ROSString,
            self.trigger_input_topic,
            self.trigger_message_callback,
            10,
        )

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

            self.get_logger().info(f"[AAD] JSON artifact created at: {artifact_path}")
            self.get_logger().info(json.dumps(payload, indent=2))
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
        response = ""
        try:
            llm = LLMClient()
            if self.llm_local:
                response = llm.local_chat(full_payload)
            else:
                response = llm.chat(full_payload)

        except Exception as e:
            self.get_logger().warn(
                f"[AAD] LLM call failed. See: {e}"
            )

        # Create artifact even if API failed
        artifact_id = self._generate_artifact_id()
        self._write_api_artifact(artifact_id, raw_list, response)

        # Try parsing decision if possible
        try:
            decision = parse_llm_response(response)

            #### Added for the config tests 
            decision_msg = String()
            decision_msg.data = (
                f"anomaly={decision.anomaly} severity={decision.severity} "
                f"action={decision.action} summary={decision.summary}"
            )
            self.decision_pub.publish(decision_msg)
            ####

            if decision.anomaly:
                alert = ROSString()
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

    def trigger_message_callback(self, msg: ROSString) -> None:
        self._msg_count += 1
        self.queue.add(msg.data)
        self.get_logger().info(f"TRIGGER_MESSAGE_CALLBACK() received message from {self.trigger_input_topic}")
        ## TODO uncomment for actual llm testing
        ##self.llm_callback() 
        return

    def _run_trigger_script_install(self, trigger: str) -> Node:
        """
        Run trigger script installation (from __init__). Assumes install.sh is in the trigger_script subfolder and is executable.
        
        Args:
            trigger (str): The name of the trigger script to install. Should correspond to a subfolder in triggers/ with an install.sh script.
        """
        ## install.sh for deps
        script_path = os.path.join(os.path.dirname(__file__), "triggers", trigger, "install.sh")
        if not os.path.isfile(script_path):
            self.get_logger().error(f"Line {sys._getframe().f_lineno}: install.sh not found at {script_path}")
            return None

        try:
            subprocess.run(
                ["bash", script_path],
                cwd=os.path.dirname(script_path),
                check=True,
            )
            self.get_logger().info("install.sh completed.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Line {sys._getframe().f_lineno}: install.sh failed: {e}")
            return None

        ## import class and append to self.trigger_nodes
        module_path = os.path.join(os.path.dirname(__file__), "triggers", trigger, f"{trigger}.py")
        if not os.path.isfile(module_path):
            self.get_logger().error(f"Line {sys._getframe().f_lineno}: Trigger module not found at {module_path}")
            return None

        module_name = f"aad_trigger_{trigger}"
        try:
            spec = importlib.util.spec_from_file_location(module_name, module_path)
            if spec is None or spec.loader is None:
                self.get_logger().error(f"Line {sys._getframe().f_lineno}: Failed to load module spec for {module_path}")
                return None
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)
        except Exception as e:
            self.get_logger().error(f"Line {sys._getframe().f_lineno}: Failed to import trigger module {module_path}: {e}")
            return None

        class_name = "".join(part.capitalize() for part in trigger.split("_"))
        trigger_cls = getattr(module, class_name, None)
        if not isinstance(trigger_cls, type) or not issubclass(trigger_cls, Node):
            trigger_cls = None
            for obj in module.__dict__.values():
                if isinstance(obj, type) and issubclass(obj, Node) and obj is not Node:
                    trigger_cls = obj
                    break

        if trigger_cls is None:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: No ROS Node subclass found in {module_path}"
            )
            return None

        try:
            trigger_node = trigger_cls()
        except Exception as e:
            self.get_logger().error(
                f"Line {sys._getframe().f_lineno}: Failed to instantiate trigger node {trigger_cls.__name__}: {e}"
            )
            return None

        return trigger_node

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
    if hasattr(node, "trigger_nodes"):
        for trigger_node in node.trigger_nodes:
            executor.add_node(trigger_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ## destroy trigger script nodes if they exist
        if hasattr(node, "trigger_nodes"):
            for trigger_node in node.trigger_nodes:
                trigger_node.destroy_node()

        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
