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
from dataclasses import dataclass, field
import json
import os
import subprocess
import threading
import time
from typing import Any

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


from anomaly_detection.llm_client import LLMClient
from anomaly_detection.response_handler import parse_llm_response


@dataclass(frozen=True)
class CachedImage:
    """One latest camera frame tracked by source and generation."""

    source: str
    generation: int
    received_monotonic: float
    captured_at: str
    image: Any = field(compare=False, repr=False)


@dataclass(frozen=True)
class CachedText:
    """One timestamped text context item with an optional trigger token."""

    text: str
    received_monotonic: float
    trigger_generation: int | None = None


def _discard_stale_text(
    queue: deque,
    now_monotonic: float,
    max_age_seconds: float,
) -> int:
    """Discard expired text context from the oldest side of a deque."""
    if max_age_seconds <= 0.0:
        return 0

    removed = 0
    while queue:
        oldest = queue[0]
        if (now_monotonic - oldest.received_monotonic) <= max_age_seconds:
            break
        queue.popleft()
        removed += 1
    return removed


def _pending_trigger_generation(
    items: list[CachedText],
    processed_generation: int,
) -> int | None:
    """Return the newest unprocessed actionable generation in a snapshot."""
    pending = [
        item.trigger_generation
        for item in items
        if item.trigger_generation is not None
        and item.trigger_generation > processed_generation
    ]
    return max(pending, default=None)


def _snapshot_fresh_images(
    cache_by_source: dict[Any, CachedImage],
    now_monotonic: float,
    max_age_seconds: float,
) -> list[CachedImage]:
    """Return fresh camera frames in deterministic arrival order."""
    fresh = [
        cached
        for cached in cache_by_source.values()
        if max_age_seconds <= 0.0
        or (now_monotonic - cached.received_monotonic) <= max_age_seconds
    ]
    return sorted(fresh, key=lambda cached: cached.generation)


def _consume_image_snapshot(
    cache_by_source: dict[Any, CachedImage],
    snapshot: list[CachedImage],
) -> None:
    """Consume snapshot frames without deleting newer replacements."""
    for cached in snapshot:
        for key, current in list(cache_by_source.items()):
            if current.generation == cached.generation:
                del cache_by_source[key]
                break


def _parse_trigger_importance(value: Any) -> int:
    """Convert a configured LLM trigger threshold to an importance value."""
    if isinstance(value, str):
        named_values = {
            "info": AnomalyMsg.INFO,
            "warning": AnomalyMsg.WARNING,
            "error": AnomalyMsg.ERROR,
        }
        normalized = value.strip().lower()
        if normalized in named_values:
            return named_values[normalized]

    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return AnomalyMsg.WARNING

    if parsed < AnomalyMsg.INFO or parsed > AnomalyMsg.ERROR:
        return AnomalyMsg.WARNING
    return parsed


def _build_artifact_payload(
    artifact_id: str,
    created_at: str,
    timestamp_ns: int,
    cached_data: list[str],
    image_metadata: list[dict[str, Any]],
    api_response: str,
) -> dict[str, Any]:
    """Build a JSON-safe artifact without embedding image bytes."""
    return {
        "artifact_id": artifact_id,
        "created_at": created_at,
        "timestamp_ns": timestamp_ns,
        "cached_data": list(cached_data),
        "images": list(image_metadata),
        "api_response": api_response,
    }


def _prune_api_artifacts(
    output_dir: str,
    max_files: int,
    keep_path: str,
) -> list[str]:
    """Remove oldest generated artifacts while preserving unrelated files."""
    if max_files <= 0:
        return []

    keep_path = os.path.abspath(keep_path)
    candidates: list[tuple[int, str, str]] = []
    try:
        entries = list(os.scandir(output_dir))
    except OSError:
        return []

    for entry in entries:
        if (
            not entry.is_file(follow_symlinks=False)
            or not entry.name.startswith("api_artifact_")
            or not entry.name.endswith(".json")
        ):
            continue
        try:
            modified_ns = entry.stat(follow_symlinks=False).st_mtime_ns
        except OSError:
            continue
        candidates.append((modified_ns, entry.name, os.path.abspath(entry.path)))

    newest_first = sorted(candidates, reverse=True)
    retained = {keep_path}
    for _, _, path in newest_first:
        if len(retained) >= max_files:
            break
        retained.add(path)

    removed: list[str] = []
    for _, _, path in newest_first:
        if path in retained:
            continue
        try:
            os.unlink(path)
            removed.append(path)
        except OSError:
            continue
    return removed


class AnomalyDetectionNode(Node):
    """Cache anomaly messages, ask the LLM for decisions, and publish alerts."""

    def __init__(self):
        super().__init__("anomaly_detection")

        # Load config once on startup
        self.config = self._load_config()
        llm_config = self.config.get("llm", {})
        self.llm_local = bool(llm_config.get("local", False))
        self.vision_enabled = bool(llm_config.get("vision_enabled", False))
        self.warm_local_model_on_startup = bool(
            llm_config.get("warm_on_startup", False)
        )
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
        self.cache_max_age_seconds = max(
            0.0,
            float(self.config.get("cache_max_age_seconds", 30.0)),
        )
        self.queue = deque(maxlen=self.cache_max_items)
        self._queue_lock = threading.Lock()
        self._llm_state_lock = threading.Lock()
        self._llm_worker_running = False
        self._llm_rerun_requested = False
        self._llm_worker_shutdown = False
        self.llm_min_trigger_importance = _parse_trigger_importance(
            self.config.get("llm_min_trigger_importance", "warning")
        )
        self._inference_trigger_generation = 0
        self._processed_inference_trigger_generation = 0
        self._llm_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="aad-llm",
        )

        # Keep a bounded rolling pre-event history. Camera frames never trigger
        # inference; they are attached only when actionable text starts a pass.
        self.image_max_frames = max(
            1,
            int(
                llm_config.get(
                    "image_max_frames",
                    llm_config.get("image_max_sources", 2),
                )
            ),
        )
        self.image_max_age_seconds = max(
            0.0,
            float(llm_config.get("image_max_age_seconds", 10.0)),
        )
        self.image_queue: dict[int, CachedImage] = {}
        self._image_generation = 0
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
        self.api_artifact_max_files = max(
            0,
            int(self.config.get("api_artifact_max_files", 250)),
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
            if self.warm_local_model_on_startup:
                self._warm_local_model()
            else:
                self.get_logger().info(
                    "Local model will load lazily on the first actionable event."
                )

        # Create one reusable client
        self.llm = LLMClient(config_path=self.config_path)

        self.get_logger().info(
            "AAD node started with config: "
            f"raw_input_topic={self.raw_input_topic}, "
            f"alert_topic={self.alert_topic}, "
            f"formatted_message_topic={self.formatted_message_topic}, "
            f"api_frequency_seconds={self.api_frequency_seconds}, "
            f"cache_max_items={self.cache_max_items}, "
            f"cache_max_age_seconds={self.cache_max_age_seconds}, "
            f"duplicate_message_min_period_sec={self.duplicate_message_min_period_sec}, "
            f"publisher_message_min_period_by_importance={self.publisher_message_min_period_by_importance}, "
            f"error_capture_window_sec={self.error_capture_window_sec}, "
            f"llm_min_trigger_importance="
            f"{self._importance_to_str(self.llm_min_trigger_importance)}, "
            f"warm_local_model_on_startup={self.warm_local_model_on_startup}, "
            f"vision_enabled={self.vision_enabled}, "
            f"image_max_frames={self.image_max_frames}, "
            f"image_max_age_seconds={self.image_max_age_seconds}, "
            f"api_artifact_max_files={self.api_artifact_max_files}, "
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
            if not self.vision_enabled:
                return
            try:
                cv_image = self._cv_bridge.imgmsg_to_cv2(msg.image, desired_encoding="rgb8")
                received_monotonic = time.monotonic()
                source = self._camera_source(msg)
                captured_at = self._message_timestamp(msg)
                with self._image_queue_lock:
                    self._image_generation += 1
                    self.image_queue[self._image_generation] = CachedImage(
                        source=source,
                        generation=self._image_generation,
                        received_monotonic=received_monotonic,
                        captured_at=captured_at,
                        image=cv_image,
                    )
                    while len(self.image_queue) > self.image_max_frames:
                        oldest_generation = min(self.image_queue)
                        del self.image_queue[oldest_generation]
                    image_count = len(self.image_queue)
                self.get_logger().debug(
                    f"[AAD] Cached camera frame; source={source}, "
                    f"image_count={image_count}"
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
            self.get_logger().debug(
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
            self.get_logger().debug(
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
                trigger_generation = None
                if importance >= self.llm_min_trigger_importance:
                    self._inference_trigger_generation += 1
                    trigger_generation = self._inference_trigger_generation
                self.queue.append(
                    CachedText(
                        text=formatted,
                        received_monotonic=now,
                        trigger_generation=trigger_generation,
                    )
                )
                queue_size = len(self.queue)
            self.get_logger().debug(
                f"[AAD] Queued message for LLM; queue_size={queue_size}"
            )
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
        with self._error_capture_lock:
            capture_active = self._error_capture_timer is not None
        if capture_active:
            self.get_logger().debug(
                "[AAD] Deferring LLM work until the error context window closes."
            )
            return

        with self._queue_lock:
            _discard_stale_text(
                self.queue,
                time.monotonic(),
                self.cache_max_age_seconds,
            )
            has_text_context = bool(self.queue)
            pending_generation = _pending_trigger_generation(
                list(self.queue),
                self._processed_inference_trigger_generation,
            )
            has_pending_trigger = pending_generation is not None

        if not has_text_context:
            self.get_logger().debug(
                "[AAD] LLM callback fired with no queued text; skipping call. "
                "Camera frames never trigger inference by themselves."
            )
            return

        if not has_pending_trigger:
            self.get_logger().debug(
                "[AAD] Retaining INFO messages as context; no warning, error, "
                "or explicit trigger requires an LLM call."
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

            with self._queue_lock:
                _discard_stale_text(
                    self.queue,
                    time.monotonic(),
                    self.cache_max_age_seconds,
                )
                pending_trigger = _pending_trigger_generation(
                    list(self.queue),
                    self._processed_inference_trigger_generation,
                )

            with self._llm_state_lock:
                if (
                    (self._llm_rerun_requested or pending_trigger is not None)
                    and not self._llm_worker_shutdown
                ):
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
            _discard_stale_text(
                self.queue,
                time.monotonic(),
                self.cache_max_age_seconds,
            )
            text_snapshot = list(self.queue)
            trigger_generation = _pending_trigger_generation(
                text_snapshot,
                self._processed_inference_trigger_generation,
            )
            if text_snapshot and trigger_generation is not None:
                # Claim only the generation in this snapshot. A newer warning,
                # error, or trigger arriving during inference remains pending.
                self._processed_inference_trigger_generation = trigger_generation

        if not text_snapshot or trigger_generation is None:
            self.get_logger().debug(
                "[AAD] LLM worker found no pending actionable text; skipping call."
            )
            return

        raw_list = [cached.text for cached in text_snapshot]

        image_snapshot: list[CachedImage] = []
        if self.vision_enabled:
            with self._image_queue_lock:
                image_snapshot = _snapshot_fresh_images(
                    self.image_queue,
                    time.monotonic(),
                    self.image_max_age_seconds,
                )

        self.get_logger().info(
            f"[AAD] LLM callback processing {len(raw_list)} text messages and "
            f"{len(image_snapshot)} fresh camera frames."
        )

        backend_succeeded = False
        used_image_snapshot: list[CachedImage] = []
        try:
            full_payload = "\n".join(raw_list)
            if not full_payload:
                full_payload = "Analyze the queued system event for anomalies."

            prepared_images = []
            image_metadata: list[dict[str, Any]] = []
            if image_snapshot:
                for cached in image_snapshot:
                    try:
                        prepared = self.llm.prepare_images([cached.image])[0]
                    except Exception as e:
                        self.get_logger().warn(
                            f"[AAD] Camera frame preparation failed; skipping "
                            f"source={cached.source}. See: {e}"
                        )
                        continue

                    prepared_images.append(prepared)
                    used_image_snapshot.append(cached)
                    image_metadata.append(
                        {
                            "source": cached.source,
                            "captured_at": cached.captured_at,
                            "generation": cached.generation,
                            "original_width": prepared.original_width,
                            "original_height": prepared.original_height,
                            "width": prepared.width,
                            "height": prepared.height,
                            "mime_type": prepared.mime_type,
                            "encoded_bytes": prepared.byte_size,
                            "sha256": prepared.sha256,
                        }
                    )

                if used_image_snapshot:
                    full_payload += (
                        "\nAttached camera frames, in order:\n"
                        + "\n".join(
                            f"Image {index}: source={cached.source} "
                            f"captured_at={cached.captured_at}"
                            for index, cached in enumerate(
                                used_image_snapshot,
                                start=1,
                            )
                        )
                    )
                elif image_snapshot:
                    self.get_logger().warn(
                        "[AAD] No camera frames could be prepared; continuing "
                        "with text context only."
                    )

            response = ""
            try:
                self.llm_called_pub.publish(Bool(data=True))
                chat = self.llm.local_chat if self.llm_local else self.llm.chat
                response = chat(full_payload, images=prepared_images)
                backend_succeeded = True

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
            self._write_api_artifact(
                artifact_id,
                raw_list,
                response,
                image_metadata=image_metadata,
            )

            # Try parsing decision if possible
            try:
                decision = parse_llm_response(response)

                self._publish_decision(decision)

            except Exception as e:
                self.get_logger().warn(
                    f"[AAD] Could not parse decision during testing: {e}"
                )
        finally:
            if backend_succeeded:
                self._remove_queue_snapshot(
                    self.queue,
                    self._queue_lock,
                    text_snapshot,
                )
                with self._image_queue_lock:
                    _consume_image_snapshot(
                        self.image_queue,
                        used_image_snapshot,
                    )
            else:
                self.get_logger().warn(
                    "[AAD] Preserving queued event context after backend failure; "
                    "a newer actionable event can retry it."
                )

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
            self._inference_trigger_generation += 1
            self.queue.append(
                CachedText(
                    text=msg.data,
                    received_monotonic=time.monotonic(),
                    trigger_generation=self._inference_trigger_generation,
                )
            )
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

    @staticmethod
    def _camera_source(msg: AnomalyMsg) -> str:
        """Return a stable source label for a camera message."""
        try:
            frame_id = str(msg.header.frame_id or "").strip()
        except Exception:
            frame_id = ""
        if frame_id:
            return frame_id

        node_name = str(getattr(msg, "node_name", "") or "").strip()
        return node_name or "unknown_camera"

    @staticmethod
    def _message_timestamp(msg: AnomalyMsg) -> str:
        """Format the ROS timestamp without depending on wall-clock time."""
        try:
            return f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        except Exception:
            return "unknown"

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
        llm_config = self.config.get("llm", {})
        model_name = llm_config.get("model", "mistral-small")
        timeout_seconds = max(
            1.0,
            float(llm_config.get("timeout_seconds", 30.0)),
        )
        keep_alive = str(llm_config.get("keep_alive", "2m"))
        client = Client(
            host="http://localhost:11434",
            timeout=timeout_seconds,
        )

        self.get_logger().info(f"Warming Ollama model: {model_name}")
        client.chat(
            model=model_name,
            messages=[{"role": "user", "content": "ping"}],
            stream=False,
            think=False,
            keep_alive=keep_alive,
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

    def _write_api_artifact(
        self,
        artifact_id: str,
        cached_data: list[str],
        api_response: str,
        image_metadata: list[dict[str, Any]] | None = None,
    ) -> str | None:
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

            payload = _build_artifact_payload(
                artifact_id=artifact_id,
                created_at=datetime.now(timezone.utc).isoformat(),
                timestamp_ns=self.get_clock().now().nanoseconds,
                cached_data=cached_data,
                image_metadata=image_metadata or [],
                api_response=api_response,
            )

            with open(artifact_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)

            removed = _prune_api_artifacts(
                self.api_artifact_output_dir,
                self.api_artifact_max_files,
                artifact_path,
            )
            self.get_logger().info(
                f"[AAD] JSON artifact created at: {artifact_path}; "
                f"text_items={len(cached_data)}, images={len(image_metadata or [])}, "
                f"response_chars={len(api_response)}, pruned={len(removed)}"
            )
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
            config_path = os.path.abspath(env_path)
        else:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        self.config_path = config_path

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
