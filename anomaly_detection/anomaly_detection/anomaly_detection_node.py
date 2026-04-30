"""Central manager for AI Anomaly Detection node.

Handles API integration, trigger method integration, response handling,
and lightweight API artifact capture.

Reads std anomaly messages, caches an LLM-friendly representation,
periodically calls the LLM/API, parses results, and publishes alerts.

Each time the LLM is called, the node writes a JSON artifact containing:
- cached_data
- api_response

Author: AAD Team Spring 26'
Version: 4/21/2026
"""
import importlib.util

import json
import os
import subprocess
import threading
import time

from std_msgs.msg import String as ROSString
from std_msgs.msg import Bool

import sys
from collections import deque

import rclpy
import yaml
from anomaly_msg.msg import AnomalyMsg
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ollama import Client
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy


from anomaly_detection.llm_client import LLMClient
from anomaly_detection.response_handler import parse_llm_response, Decision


class AnomalyDetectionNode(Node):
    """
    Description
    -----------
        Central manager for AI Anomaly Detection node.
        - Subscribes to /ai_anomaly_logging (standardized logging topic)
        - Caches a bounded, LLM-friendly representation of AnomalyMsg
        - Periodically calls the LLM/API (or on-demand through trigger topic)
        - Parses result via response_handler into a Decision
        - Publishes alerts to /aad/alerts when anomalies are detected
        - Leaves JSON artifacts behind of cache -> LLM response in ./logs.
        - Optionally creates API-triggered rosbag artifacts if configured

    Attributes
    ----------
        config (dict): Configuration dictionary loaded from YAML file.
            Default expect config.yaml at level of this file, otherwise uses os.getenv("AAD_CONFIG_PATH"). 
        
        llm_local (bool): Whether to start and use a local Ollama server for LLM inference. 
            Default: False.

        trigger_input_topic (str): The ROS topic to subscribe to for trigger messages that should cause an immediate LLM call. 
            Default: /trigger_messages.

        raw_input_topic (str): The ROS topic to subscribe to for raw anomaly messages. Default: /ai_anomaly_logging.
            Default: /ai_anomaly_logging.

        alert_topic (str): The ROS topic to publish alerts to when anomalies are detected. Default: /aad/alerts.
            Default: /aad/alerts.

        trigger_nodes (list[Node]): List of instantiated trigger script nodes to add to the executor. Populated based on config _trigger_scripts list.
            Defautl: [].
        
        api_frequency_seconds (float): How often to call the LLM/API in seconds. 
            Default: 60.0.

        cache_max_items (int): Maximum number of messages to keep in the cache for LLM processing. 
            Default: 100.

        queue (deque): Thread-safe deque to cache formatted messages for LLM processing.

        throttle_info (bool): Whether to throttle INFO messages to protect LLM context. 
            Default: True.

        info_min_period_sec (float): Minimum period in seconds between INFO messages if throttling is enabled. 
            Default: 1.0.

        log_every_n_msgs (int): How often to log received messages for debugging. 
            Default: 50.

        alert_pub (Publisher): ROS publisher for anomaly alerts.
    
        decision_pub (Publisher): ROS publisher for parsed LLM decisions (added for offline runner).

        llm_called_pub (Publisher): ROS publisher to indicate when the LLM is called (added for offline runner).

        api_artifact_output_dir (str): Directory to write JSON artifacts containing LLM input and output. 
            Default: /root/dev_ws/src/anomaly_detection/log.
        
        llm (LLMClient): Reusable client for making LLM calls, initialized once in __init__.

        _ollama_proc (subprocess.Popen | None): Handle for the local Ollama server process if llm_local is True.

        _triggers list[str]: List of trigger script names to load from the triggers/ subfolder. Each should have an install.
            Default: [].
            
        _queue_lock (threading.Lock): Lock to protect access to the message cache queue.

        _last_info_time_sec (float): Timestamp of the last INFO message processed, used for throttling.

        _msg_count (int): Counter for the number of messages received, used for periodic logging.
    
    
    Methods
    -------
        log_caching_callback(msg: AnomalyMsg):
            Callback for incoming AnomalyMsg messages. Caches them in a bounded deque after formatting for LLM. Optionally throttles INFO messages.
    
        llm_callback():
            Timer-driven callback that processes cached messages with the LLM, parses the response, and publishes alerts if anomalies are detected. 
            Snapshots and clears the cache atomically before processing.

        trigger_message_callback():
            Triggered when trigger_input_topic receives a message, adds to queue, triggers LLM call (since anomaly was found by trigger).

        _is_ollama_ready():
            Quick health check for an already-running local Ollama server.

        _start_local_ollama():
            Starts a local Ollama server as a subprocess if llm_local is True.

        _wait_for_ollama_ready(timeout_sec: float):
            Waits until the local Ollama server is responsive or a timeout is reached.
            
        _warm_local_model():
            Makes a simple call to the local Ollama server to load the model into memory and reduce latency for the first real call.

        _stop_local_ollama():
            Stops the local Ollama server subprocess if it was started by this node.    
        
        _write_api_artifact(artifact_id: str, cached_data: list[str], api_response: str):
            Writes a JSON artifact containing the cached LLM input and the API response for later analysis.

        _run_trigger_script_install():
            Run install.sh of respective trigger scripts and import modules to self.trigger_nodes.
            
        _load_config():
            Loads configuration from a YAML file. Resolution order:
              AAD_CONFIG_PATH if set, config.yaml if not.

        _importance_to_str(importance: int):
            Helper method to convert AnomalyMsg importance integer to a string representation (INFO, WARNING, ERROR).
        
        _type_to_str(msg_type: int):
            Helper method to convert AnomalyMsg type integer to a string representation (TEXT, IMAGE, DATA).

        _format_for_llm(m: AnomalyMsg):
            Converts an AnomalyMsg into a compact, LLM-friendly string representation.
    
    """

    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()
        self.llm_local = bool(self.config.get("llm", {}).get("local", False))
        self._ollama_proc = None

        ## import sub type profiles
        # BEST EFFORT (camera, fast streams)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # RELIABLE (logs, structured data)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Standard topic defaults
        self.trigger_input_topic = self.config.get("trigger_input_topic", "/trigger_messages")
        self.raw_input_topic = self.config.get("raw_input_topic", "/ai_anomaly_logging")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")

        ## Install deps for chosen triggers & start them
        self._triggers = self.config.get("trigger_scripts") if self.config.get("trigger_scripts") != None else []
        self.trigger_nodes = []
        for trigger in self._triggers:
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
        self.queue = deque(maxlen=self.cache_max_items)
        self._queue_lock = threading.Lock()

        self.image_queue = deque(maxlen=self.cache_max_items)
        self._image_queue_lock = threading.Lock()

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
        self.llm_called_pub = self.create_publisher(Bool, "/aad/llm_called", 10)

        # JSON artifact output config
        self.api_artifact_output_dir = self.config.get(
            "api_artifact_output_dir",
            "/root/dev_ws/src/anomaly_detection/logs",
        )

        # Subscription to standardized logging topic
        ## two different profile types to account for different pub modes to raw_input_topic
        self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            best_effort_qos
        )

        self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            reliable_qos,
        )

        self.create_subscription(
            ROSString,
            self.trigger_input_topic,
            self.trigger_message_callback,
            10,
        )

        self.create_timer(self.api_frequency_seconds, self.llm_callback)

        # Start local Ollama once, before first inference
        if self.llm_local:

            if self._is_ollama_ready():
                self.get_logger().info(
                    "Detected existing Ollama server at http://localhost:11434; reusing it."
                )
            else:
                self._start_local_ollama()
                self._wait_for_ollama_ready()
            self._warm_local_model()

        # Create one reusable client
        self.llm = LLMClient()

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

    def log_caching_callback(self, msg: AnomalyMsg) -> None:
        """
        Monitor incoming AnomalyMsg messages, format them for LLM consumption, and cache in a thread-safe deque.
        
        Args
        ----
            msg (AnomalyMsg): The incoming anomaly message.
        
        """
        self._msg_count += 1
        if self.log_every_n_msgs > 0 and (self._msg_count % self.log_every_n_msgs == 0):
            self.get_logger().info(
                f"Received {self._msg_count} messages on {self.raw_input_topic}"
            )

        if msg.type == AnomalyMsg.IMAGE:
            try:
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(msg.image)
                with self._image_queue_lock:
                    self.image_queue.append(cv_image)
            except Exception as e:
                self.get_logger().warn(
                    f"Line {sys._getframe().f_lineno}: Failed to cache message safely: {e}"
                )
        else: 
            try:
                formatted = self._format_for_llm(msg)
                with self._queue_lock:
                    self.queue.append(formatted)
            except Exception as e:
                self.get_logger().warn(
                    f"Line {sys._getframe().f_lineno}: Failed to cache message safely: {e}"
                )

        # Optional INFO throttling
        if self.throttle_info and int(msg.importance) == AnomalyMsg.INFO:
            now = float(self.get_clock().now().nanoseconds) / 1e9
            if (now - self._last_info_time_sec) < self.info_min_period_sec:
                return
            self._last_info_time_sec = now

    def llm_callback(self) -> None:
        """
        Timer-driven callback for processing cached log messages with the LLM.
        Also creates artifacts of llm i/o for future analysis.

        Flow
        ----
        - snapshot cache
        - clear cache
        - call LLM/API
        - write a JSON artifact containing cached_data and api_response
        - parse with response handler
        - publish alert if anomaly
        """
        with self._queue_lock:
            raw_list = list(self.queue)
            self.queue.clear()

        with self._image_queue_lock:
            raw_image_list = list(self.image_queue)
            self.image_queue.clear()

        if not raw_list and not raw_image_list:
            return

        full_payload = "\n".join(raw_list)
        response = ""
        try:
            msg = Bool()
            msg.data = True
            self.llm_called_pub.publish(msg)
            if self.llm_local:
                response = self.llm.local_chat(full_payload, raw_image_list)
            else:
                response = self.llm.chat(full_payload, raw_image_list)

        except Exception as e:
            self.get_logger().warn(
                f"[AAD] LLM call failed. See: {e}"
            )
            
            decision_msg = ROSString()
            decision_msg.data = (
                f"anomaly=False severity=unknown "
                f"action=LLM Call failed summary=LLM call Failed"
            )
            self.decision_pub.publish(decision_msg)

        # Create artifact even if API failed
        artifact_id = f"api_artifact_{self.get_clock().now().nanoseconds}"
        self._write_api_artifact(artifact_id, raw_list, response)

        # Try parsing decision if possible
        try:
            decision = parse_llm_response(response)

            #### Added for the config tests 
            decision_msg = ROSString()
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

    def trigger_message_callback(self, msg: ROSString) -> None:
        """
        Monitor trigger messages that cause an immediate LLM call.
        
        Args
        ----
            msg (ROSString): The incoming trigger message.
        
        """
        self._msg_count += 1
        with self._queue_lock:
            self.queue.append(msg.data)
        self.get_logger().info(f"TRIGGER_MESSAGE_CALLBACK() received message from {self.trigger_input_topic}")
        ## TODO uncomment for actual llm testing
        ##self.llm_callback() 
        return

    def _is_ollama_ready(self) -> bool:
        """
        Quick health check for an already-running Ollama server.
        """
        try:
            Client(host="http://localhost:11434").list()
            return True
        except Exception:
            return False

    def _start_local_ollama(self) -> None:
        """
        Runs local Ollama server as a subprocess. Runs ollama serve on localhost:11434.
        """
        if self._ollama_proc is not None and self._ollama_proc.poll() is None:
            return

        env = os.environ.copy()
        env.setdefault("OLLAMA_HOST", "127.0.0.1:11434")

        self.get_logger().info("Starting local Ollama server...")
        self._ollama_proc = subprocess.Popen(
            ["ollama", "serve"],
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _wait_for_ollama_ready(self, timeout_sec: float = 15.0) -> None:
        """
            Ping local Ollama server until it's ready or timeout is reached.
        
        Args
        ----
            timeout_sec (float): How long to wait until timeout
        """
        deadline = time.time() + timeout_sec
        client = Client(host="http://localhost:11434")

        last_err = None
        while time.time() < deadline:
            if self._ollama_proc is not None and self._ollama_proc.poll() is not None:
                raise RuntimeError("ollama serve exited before becoming ready")

            try:
                client.list()
                self.get_logger().info("Ollama API is ready.")
                return
            except Exception as e:
                last_err = e
                time.sleep(0.5)

        raise RuntimeError(f"Ollama API did not become ready within {timeout_sec}s: {last_err}")

    def _warm_local_model(self) -> None:
        """
        Call the local model to "warm" it up and load into mem.W
        """
        model_name = self.config.get("llm", {}).get("model", "mistral-small")
        client = Client(host="http://localhost:11434")

        self.get_logger().info(f"Warming Ollama model: {model_name}")
        client.chat(
            model=model_name,
            messages=[{"role": "user", "content": "ping"}],
            stream=False,
            keep_alive="15m",
            options={"temperature": 0, "num_predict": 1},
        )

    def _stop_local_ollama(self) -> None:
        """
        Kills the local Ollama server subprocess if it was started by this node.
        """
        if self._ollama_proc is not None and self._ollama_proc.poll() is None:
            self.get_logger().info("Stopping local Ollama server...")
            self._ollama_proc.terminate()
            try:
                self._ollama_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._ollama_proc.kill()
                self._ollama_proc.wait(timeout=5)

    def _write_api_artifact(self, artifact_id: str, cached_data: list[str], api_response: str) -> str | None:
        """
        Write a JSON artifact containing the cached LLM input and the API response.
        
        Args
        ----
            artifact_id (str): The unique ID for the artifact.

            cached_data (list[str]): The cached data sent to the LLM.

            api_response (str): The raw response from the LLM.

        Returns
        -------
            str: The path to the created JSON artifact, or None if creation failed.

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

    def _run_trigger_script_install(self, trigger: str) -> Node | None:
        """
        Run trigger script installation (from __init__). Assumes install.sh is in the ./triggers/trigger subfolder and is executable.
        
        Args:
            trigger (str): The name of the trigger script to install. Should correspond to a subfolder in triggers/ with an install.sh script.

        Returns:
            Node | None: An instance of the trigger node class defined in the trigger's Python module.
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

    def _importance_to_str(self, importance: int) -> str:
        """
        Convert an AnomalyMsg importance integer to a string representation.
        
        Args
        ----
            importance (int): The importance level of the anomaly message.

        Returns
        -------
            str: The string representation of the importance level.
        """
        if importance == AnomalyMsg.ERROR:
            return "ERROR"
        if importance == AnomalyMsg.WARNING:
            return "WARNING"
        return "INFO"

    def _type_to_str(self, msg_type: int) -> str:
        """
        Convert an AnomalyMsg type integer to a string representation.
        
        Args
        ----
            msg_type (int): The type of the anomaly message.

        Returns
        -------
            str: The string representation of the message type.
        
        """
        if msg_type == AnomalyMsg.IMAGE:
            return "IMAGE"
        if msg_type == AnomalyMsg.DATA:
            return "DATA"
        return "TEXT"

    def _format_for_llm(self, m: AnomalyMsg) -> str:
        """
        Convert an AnomalyMsg into a compact, LLM-friendly string representation.
        
        Args
        ----
            m (AnomalyMsg): The anomaly message to format.

        Returns
        -------
            str: The formatted string representation.
        
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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        ## destroy trigger script nodes if they exist
        
        if hasattr(node, "trigger_nodes"):
            for trigger_node in node.trigger_nodes:
                trigger_node.destroy_node()

        # stop ollama first
        if hasattr(node, "_stop_local_ollama"):
            node._stop_local_ollama()
        node.destroy_node()
        executor.shutdown()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
