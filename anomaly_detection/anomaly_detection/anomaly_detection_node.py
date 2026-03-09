import os
import yaml
import json
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from anomaly_msg.msg import AnomalyLog

from anomaly_detection.StringRingBuffer import StringRingBuffer


class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__('anomaly_detection')

        # Load config once on startup
        self.config = self._load_config()

        # Determine if buffer model is enabled (default to False if not specified)
        self.buffer_model_enabled = self.config.get("buffer_model", False)

        # Sprint-required timing variables (with safe defaults)
        self.artifact_frequency_seconds = self.config.get("artifact_frequency_seconds", 30)
        self.artifact_duration_seconds = self.config.get("artifact_duration_seconds", 10)
        self.api_frequency_seconds = self.config.get("api_frequency_seconds", 60)

        # Load topic names from config (fallback to defaults)
        self.raw_input_topic = self.config.get("raw_input_topic", "raw_input")
        #self.trimmed_output_topic = self.config.get("trimmed_output_topic", "trimmed_data")

        self.get_logger().info(
            f"Loaded config: "
            f"artifact_frequency_seconds={self.artifact_frequency_seconds}, "
            f"artifact_duration_seconds={self.artifact_duration_seconds}, "
            f"api_frequency_seconds={self.api_frequency_seconds}"
        )

        if (self.buffer_model_enabled):
            self.get_logger().info("Buffer model enabled. Running install.sh")
            self._run_buffer_model_install()

        # Need to change so it used config #####
        self.queue = StringRingBuffer(max_items=100)

        # Create subscription for raw input
        self.subscription = self.create_subscription(
            AnomalyLog,
            self.raw_input_topic,
            self.log_caching_callback,
            10
        )

        if (self.buffer_model_enabled):
            self._start_buffer_model()

        self.timer = self.create_timer(self.api_frequency_seconds, self.llm_callback)

        # Simple message counter for debugging
        self._msg_count = 0
        self.get_logger().info("Anomaly Detection node started.")

    def _start_buffer_model(self) -> None:
        try:
            from anomaly_detection.anomaly_detection.anomaly_detection.buffer_model.buffer_model import BufferModel
        except Exception as e:
            self.get_logger().error(f"Failed to import BufferModel: {e}")
            return

        self.buffer_model_node = BufferModel()
        self._buffer_model_monitor_stop = threading.Event()
        self._buffer_model_monitor_thread = threading.Thread(
            target=self._monitor_buffer_model_anomalies,
            daemon=True,
        )
        self._buffer_model_monitor_thread.start()

    def _monitor_buffer_model_anomalies(self) -> None:
        poll_seconds = 0.2
        while not self._buffer_model_monitor_stop.is_set():
            msg = self.buffer_model_node.consume_anomaly_message()
            if msg:
                self.queue.add(msg)
                self.llm_callback()
            else:
                time.sleep(poll_seconds)

    def _stop_buffer_model(self) -> None:
        if hasattr(self, "_buffer_model_monitor_stop"):
            self._buffer_model_monitor_stop.set()
        if hasattr(self, "_buffer_model_monitor_thread"):
            if self._buffer_model_monitor_thread.is_alive():
                self._buffer_model_monitor_thread.join(timeout=1.0)

    def _run_buffer_model_install(self) -> None:
        """
        Run buffer model installation (from __init__). Assumes install.sh is in the buffer_model subfolder and is executable.
        """
        script_path = os.path.join(os.path.dirname(__file__), "buffer_model", "install.sh")
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
        # self.get_logger
        self.get_logger().info("Recieved lidar data")
        msg_str = str(msg)
        self.queue.add(msg_str)
    
    def to_jsonl(buffer):
        return "\n".join(json.dumps(obj, separators=(",", ":")) for obj in buffer)

    def llm_callback(self):
        raw_list = self.queue.snapshot()
        
        if not raw_list:
            self.get_logger().info("No anomaly messages received yet.")
            return
            
        # Process the messages with your LLM logic here
        self.get_logger().info(
            f"Processing {len(self.queue.buffer)} anomaly messages..."
        )

        self.queue.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if hasattr(node, "buffer_model_node"):
        executor.add_node(node.buffer_model_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_buffer_model()
        if hasattr(node, "buffer_model_node"):
            node.buffer_model_node.stop_event.set()
            node.buffer_model_node.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
