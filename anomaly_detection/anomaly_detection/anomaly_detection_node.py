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

from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import threading
import time

from datetime import datetime, timezone

from std_msgs.msg import String as ROSString
from std_msgs.msg import Bool

import sys
from collections import deque

import rclpy
import yaml
from anomaly_msg.msg import AnomalyMsg
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ollama import Client
# from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy


from anomaly_detection.llm_client import LLMClient, encode_image
from anomaly_detection.response_handler import parse_llm_response


class AnomalyDetectionNode(Node):
    """Cache anomaly messages, ask the LLM for decisions, and publish alerts."""

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

        # Standard topic defaults
        self.trigger_input_topic = self.config.get("trigger_input_topic", "/trigger_messages")
        self.raw_input_topic = self.config.get("raw_input_topic", "/ai_anomaly_logging")
        self.alert_topic = self.config.get("alert_topic", "/aad/alerts")
        self.formatted_message_topic = self.config.get(
            "formatted_message_topic",
            "/aad/formatted_messages",
        )

        ## Install deps for chosen triggers & start them
        self._triggers = self.config.get("trigger_scripts") or []
        self.trigger_nodes = []
        for trigger in self._triggers:
            self.get_logger().info(f"Configured trigger script: {trigger}")
            if (node := self._run_trigger_script_install(trigger)) is not None:
                self.trigger_nodes.append(node)
            else:
                self.get_logger().error(f"Line {sys._getframe().f_lineno}: Unable to load trigger script from AAD node")

        # Timing
        self.api_frequency_seconds = float(self.config.get("api_frequency_seconds", 60.0))

        # Cache sizing
        self.cache_max_items = int(self.config.get("cache_max_items", 100))
        self.queue = deque(maxlen=self.cache_max_items)
        self._queue_lock = threading.Lock()
        self._llm_state_lock = threading.Lock()
        self._llm_worker_running = False
        self._llm_rerun_requested = False
        self._llm_worker_shutdown = False
        self._llm_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="aad-llm",
        )

        # Keep only the most recent camera frames so image traffic cannot grow the cache.
        self.image_queue = deque(maxlen=2)
        self._image_queue_lock = threading.Lock()
        self._cv_bridge = CvBridge()
        self._error_capture_lock = threading.Lock()
        self._error_capture_timer = None

        self.error_capture_window_sec = float(self.config.get("error_capture_window_sec", 2.0))
        self.duplicate_message_min_period_sec = float(
            self.config.get(
                "duplicate_message_min_period_sec",
                self.config.get("same_error_min_period_sec", 5.0),
            )
        )
        self._last_message_sent_times = {}
        self.publisher_message_min_period_by_importance = (
            self._load_publisher_message_min_periods()
        )
        self._last_publisher_message_sent_times = {}

        # Debug logging controls
        self._msg_count = 0
        self.log_every_n_msgs = int(self.config.get("log_every_n_msgs", 50))


        # Publisher for alerts
        self.alert_pub = self.create_publisher(ROSString, self.alert_topic, 10)
        self.formatted_message_pub = self.create_publisher(
            ROSString,
            self.formatted_message_topic,
            10,
        )
        # Added for the config tests
        self.decision_pub = self.create_publisher(ROSString, "/aad/decisions", 10)
        self.llm_called_pub = self.create_publisher(Bool, "/aad/llm_called", 10)

        # JSON artifact output config
        self.api_artifact_output_dir = self.config.get(
            "api_artifact_output_dir",
            "/root/dev_ws/src/anomaly_detection/logs",
        )

        # BEST_EFFORT subscribers accept both BEST_EFFORT and RELIABLE publishers.
        self.create_subscription(
            AnomalyMsg,
            self.raw_input_topic,
            self.log_caching_callback,
            best_effort_qos
        )

        self.create_subscription(
            ROSString,
            self.trigger_input_topic,
            self.trigger_message_callback,
            10,
        )

        self.api_timer = self.create_timer(self.api_frequency_seconds, self.llm_callback)

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
            f"formatted_message_topic={self.formatted_message_topic}, "
            f"api_frequency_seconds={self.api_frequency_seconds}, "
            f"cache_max_items={self.cache_max_items}, "
            f"duplicate_message_min_period_sec={self.duplicate_message_min_period_sec}, "
            f"publisher_message_min_period_by_importance={self.publisher_message_min_period_by_importance}, "
            f"error_capture_window_sec={self.error_capture_window_sec}, "
            f"api_artifact_output_dir={self.api_artifact_output_dir}"
        )
        self.get_logger().info(
            f"[AAD] Periodic LLM timer armed for every {self.api_frequency_seconds:.2f}s."
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
                cv_image = self._cv_bridge.imgmsg_to_cv2(msg.image, desired_encoding="rgb8")
                with self._image_queue_lock:
                    self.image_queue.append(cv_image)
                    image_count = len(self.image_queue)
                self.get_logger().info(
                    f"[AAD] Cached camera frame; image_count={image_count}"
                )
            except Exception as e:
                self.get_logger().warn(f"[AAD] Failed to convert camera frame: {e}")
            return

        now = time.monotonic()
        message_signature = self._message_signature(msg)
        last_message_sent_time = self._last_message_sent_times.get(message_signature)
        if (
            self.duplicate_message_min_period_sec > 0.0
            and last_message_sent_time is not None
            and (now - last_message_sent_time) < self.duplicate_message_min_period_sec
        ):
            self.get_logger().info(
                "[AAD] Delaying duplicate message to LLM; "
                f"last sent {now - last_message_sent_time:.2f}s ago."
            )
            return

        importance = int(msg.importance)
        publisher_key = self._publisher_rate_limit_key(msg)
        publisher_min_period_sec = self._publisher_message_min_period_sec(importance)
        last_publisher_message_sent_time = self._last_publisher_message_sent_times.get(
            publisher_key
        )
        if (
            publisher_min_period_sec > 0.0
            and last_publisher_message_sent_time is not None
            and (now - last_publisher_message_sent_time) < publisher_min_period_sec
        ):
            publisher_name, importance_name = publisher_key
            self.get_logger().info(
                "[AAD] Limiting message from publisher; "
                f"publisher={publisher_name}, importance={importance_name}, "
                f"last accepted {now - last_publisher_message_sent_time:.2f}s ago."
            )
            return

        is_high_severity = importance == AnomalyMsg.ERROR
        try:
            formatted = self._format_for_llm(msg)
            self._publish_text(self.formatted_message_pub, formatted)
            with self._queue_lock:
                self.queue.append(formatted)
                queue_size = len(self.queue)
            self.get_logger().info(f"[AAD] Queued message for LLM; queue_size={queue_size}")
            self._last_message_sent_times[message_signature] = now
            self._prune_history(
                self._last_message_sent_times,
                now,
                self.duplicate_message_min_period_sec,
            )
            self._last_publisher_message_sent_times[publisher_key] = now
            self._prune_history(
                self._last_publisher_message_sent_times,
                now,
                self._max_publisher_message_period(),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Line {sys._getframe().f_lineno}: Failed to cache message safely: {e}"
            )
            return

        if is_high_severity:
            summary = formatted or f"High severity message from {msg.node_name}"
            self._publish_immediate_stop(summary)
            self._schedule_error_capture_llm()

    def llm_callback(self) -> None:
        """Schedule queued messages for LLM processing without blocking ROS."""
        with self._queue_lock:
            has_text_context = bool(self.queue)
        with self._image_queue_lock:
            has_image_context = bool(self.image_queue)

        if not has_text_context and not has_image_context:
            self.get_logger().info(
                "[AAD] LLM callback fired with no queued text or camera context; "
                "skipping call."
            )
            return

        with self._llm_state_lock:
            if self._llm_worker_shutdown:
                return
            if self._llm_worker_running:
                self._llm_rerun_requested = True
                self.get_logger().info(
                    "[AAD] LLM call already running; queued another processing pass."
                )
                return

            self._llm_worker_running = True
            self._llm_rerun_requested = False
            try:
                self._llm_executor.submit(self._llm_worker_loop)
            except RuntimeError:
                self._llm_worker_running = False
                if not self._llm_worker_shutdown:
                    raise

    def _llm_worker_loop(self) -> None:
        """Process requested LLM passes serially on the dedicated worker."""
        while True:
            try:
                self._process_llm_queue()
            except Exception as e:
                self.get_logger().error(f"[AAD] Unexpected LLM worker failure: {e}")

            with self._llm_state_lock:
                if self._llm_rerun_requested and not self._llm_worker_shutdown:
                    self._llm_rerun_requested = False
                    continue
                self._llm_worker_running = False
                return

    def _process_llm_queue(self) -> None:
        """
        Process one snapshot of cached messages with the LLM.

        Flow
        ----
        - snapshot cache
        - call LLM/API
        - write a JSON artifact containing cached_data and api_response
        - parse with response handler
        - publish alert if anomaly
        - remove the snapshot from cache
        """
        with self._queue_lock:
            raw_list = list(self.queue)
        with self._image_queue_lock:
            raw_image_list = list(self.image_queue)

        if not raw_list and not raw_image_list:
            self.get_logger().info(
                "[AAD] LLM callback fired with no queued text or camera context; "
                "skipping call."
            )
            return

        self.get_logger().info(
            f"[AAD] LLM callback processing {len(raw_list)} text messages and "
            f"{len(raw_image_list)} camera frames."
        )

        try:
            full_payload = "\n".join(raw_list)
            if not full_payload:
                full_payload = "Analyze the attached camera frames for anomalies."
            response = ""
            try:
                self.llm_called_pub.publish(Bool(data=True))
                chat = self.llm.local_chat if self.llm_local else self.llm.chat
                response = chat(full_payload, images=raw_image_list)

            except Exception as e:
                self.get_logger().warn(
                    f"[AAD] LLM call failed. See: {e}"
                )
                
                self._publish_text(
                    self.decision_pub,
                    f"anomaly=False severity=unknown "
                    "action=LLM Call failed summary=LLM call Failed",
                )

            # Create artifact even if API failed
            artifact_id = f"api_artifact_{self.get_clock().now().nanoseconds}"
            artifact_data = raw_list + [encode_image(image) for image in raw_image_list]
            self._write_api_artifact(artifact_id, artifact_data, response)

            # Try parsing decision if possible
            try:
                decision = parse_llm_response(response)

                self._publish_decision(decision)

            except Exception as e:
                self.get_logger().warn(
                    f"[AAD] Could not parse decision during testing: {e}"
                )
        finally:
            self._remove_queue_snapshot(self.queue, self._queue_lock, raw_list)
            # Keep the latest camera frames available for the next artifact. Camera
            # messages arrive less frequently than text messages and may stop
            # temporarily, so consuming the image snapshot here produces artifacts
            # with no visual context even though a valid latest frame exists.

    def _shutdown_llm_worker(self) -> None:
        """Stop accepting LLM work and wait for the active API call to finish."""
        with self._llm_state_lock:
            self._llm_worker_shutdown = True
            self._llm_rerun_requested = False
        self._llm_executor.shutdown(wait=True, cancel_futures=True)

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
        if self._is_high_severity_trigger(msg.data):
            self._publish_immediate_stop(msg.data)
        self._request_immediate_llm("trigger message")

    def _is_high_severity_trigger(self, text: str) -> bool:
        """
        Detect trigger strings that explicitly report high severity.
        """
        normalized = " ".join(str(text).lower().replace('"', "").replace("'", "").split())
        high_severity_tokens = (
            "severity=high",
            "severity: high",
            "severity high",
            "importance=error",
            "importance: error",
            "importance error",
            "high severity",
        )
        return any(token in normalized for token in high_severity_tokens)

    def _queue_items_match(self, current: object, expected: object) -> bool:
        """
        Compare queue items without forcing array-like objects into truth values.
        """
        if current is expected:
            return True

        try:
            return bool(current == expected)
        except Exception:
            return False

    def _remove_queue_snapshot(self, queue: deque, lock: threading.Lock, snapshot: list) -> None:
        """
        Remove snapshot items from the front of a queue after LLM response handling.
        """
        if not snapshot:
            return

        with lock:
            for item in snapshot:
                if not queue:
                    return
                if not self._queue_items_match(queue[0], item):
                    return
                queue.popleft()

    def _message_signature(self, msg: AnomalyMsg) -> tuple:
        """
        Build a stable duplicate key for messages, ignoring timestamp.
        """
        image_signature = None
        if msg.type == AnomalyMsg.IMAGE:
            try:
                image_signature = (
                    int(msg.image.width),
                    int(msg.image.height),
                    str(msg.image.encoding),
                )
            except Exception:
                image_signature = ("image", "unavailable")

        data_signature = None
        if msg.type == AnomalyMsg.DATA:
            try:
                data_signature = (str(msg.data_type), len(msg.data), tuple(msg.data))
            except Exception:
                data_signature = ("data", "unavailable")

        return (
            str(msg.node_name),
            int(msg.importance),
            int(msg.type),
            str(msg.msg),
            image_signature,
            data_signature,
        )

    def _prune_history(self, history: dict, now: float, window: float) -> None:
        """Remove rate-limit entries older than their configured window."""
        if window <= 0.0:
            history.clear()
            return

        stale_cutoff = now - window
        for key, sent_time in list(history.items()):
            if sent_time < stale_cutoff:
                del history[key]

    def _publisher_rate_limit_key(self, msg: AnomalyMsg) -> tuple[str, str]:
        """
        Build the publisher-level rate-limit key.
        """
        publisher = str(msg.node_name or "").strip()
        if not publisher:
            try:
                publisher = str(msg.header.frame_id or "").strip()
            except Exception:
                publisher = ""
        if not publisher:
            publisher = "unknown"

        return (publisher, self._importance_to_str(int(msg.importance)).lower())

    def _publisher_message_min_period_sec(self, importance: int) -> float:
        """
        Return the configured publisher rate-limit window for an importance level.
        """
        importance_name = self._importance_to_str(importance).lower()
        return float(
            self.publisher_message_min_period_by_importance.get(
                importance_name,
                self.publisher_message_min_period_by_importance.get("default", 0.0),
            )
        )

    def _load_publisher_message_min_periods(self) -> dict[str, float]:
        """
        Load per-publisher rate-limit windows from config.
        """
        configured = self.config.get("publisher_message_min_period_sec", {})
        if isinstance(configured, (int, float)):
            return {"default": float(configured)}

        if not isinstance(configured, dict):
            return {}

        periods = {}
        for key, value in configured.items():
            try:
                periods[str(key).lower()] = float(value)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f"Ignoring invalid publisher_message_min_period_sec value for {key}: {value}"
                )
        return periods

    def _max_publisher_message_period(self) -> float:
        """Return the largest configured publisher rate-limit window."""
        return max(self.publisher_message_min_period_by_importance.values(), default=0.0)

    @staticmethod
    def _publish_text(publisher, text: str) -> None:
        """Publish text through a std_msgs/String publisher."""
        publisher.publish(ROSString(data=text))

    def _publish_decision(self, decision) -> None:
        """Publish a parsed decision and its alert, when applicable."""
        details = (
            f"severity={decision.severity} action={decision.action} "
            f"summary={decision.summary}"
        )
        self._publish_text(
            self.decision_pub,
            f"anomaly={decision.anomaly} {details}",
        )
        if decision.anomaly:
            self._publish_text(self.alert_pub, f"[AAD ALERT] {details}")

    def _publish_immediate_stop(self, summary: str) -> None:
        """
        Publish a high-severity stop decision without waiting for the LLM.
        """
        clean_summary = " ".join(str(summary).split())

        self._publish_text(
            self.decision_pub,
            f"anomaly=True severity=high "
            f"action=stop_cart summary={clean_summary}",
        )
        self._publish_text(
            self.alert_pub,
            f"[AAD ALERT] severity=high "
            f"action=stop_cart summary={clean_summary}",
        )

    def _request_immediate_llm(self, reason: str) -> None:
        """
        Run the LLM callback now.
        """
        self.get_logger().info(f"[AAD] Immediate LLM requested by {reason}.")
        self.llm_callback()

    def _schedule_error_capture_llm(self) -> None:
        """
        Collect additional context after an ERROR before sending the queue to the LLM.
        """
        if self.error_capture_window_sec <= 0.0:
            self._request_immediate_llm("high severity anomaly message")
            return

        with self._error_capture_lock:
            if self._error_capture_timer is not None:
                self.get_logger().info(
                    "[AAD] ERROR capture window already active; continuing to collect context."
                )
                return

            self.get_logger().info(
                f"[AAD] ERROR received; collecting {self.error_capture_window_sec:.2f}s "
                "of additional context before LLM call."
            )
            self._error_capture_timer = self.create_timer(
                self.error_capture_window_sec,
                self._error_capture_timer_callback,
            )

    def _error_capture_timer_callback(self) -> None:
        """
        End the ERROR capture window and request the LLM.
        """
        with self._error_capture_lock:
            timer = self._error_capture_timer
            self._error_capture_timer = None

        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        self._request_immediate_llm("error capture window")

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
                "created_at": datetime.now(timezone.utc).isoformat(),
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
        """Convert an AnomalyMsg importance value to text."""
        return {
            AnomalyMsg.ERROR: "ERROR",
            AnomalyMsg.WARNING: "WARNING",
        }.get(importance, "INFO")

    def _type_to_str(self, msg_type: int) -> str:
        """Convert an AnomalyMsg type value to text."""
        return {
            AnomalyMsg.IMAGE: "IMAGE",
            AnomalyMsg.DATA: "DATA",
        }.get(msg_type, "TEXT")

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
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    for trigger_node in node.trigger_nodes:
        executor.add_node(trigger_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for trigger_node in node.trigger_nodes:
            trigger_node.destroy_node()

        node._shutdown_llm_worker()
        node._stop_local_ollama()
        node.destroy_node()
        executor.shutdown()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
