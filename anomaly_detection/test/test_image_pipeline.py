"""Test bounded image context, actionable triggers, and artifact retention."""

import json
import os
import threading
from collections import deque
from types import SimpleNamespace

from anomaly_detection.anomaly_detection_node import (
    AnomalyDetectionNode,
    CachedImage,
    CachedText,
    _build_artifact_payload,
    _consume_image_snapshot,
    _discard_stale_text,
    _parse_trigger_importance,
    _pending_trigger_generation,
    _prune_api_artifacts,
    _snapshot_fresh_images,
)


def _cached(source, generation, received):
    return CachedImage(
        source=source,
        generation=generation,
        received_monotonic=received,
        captured_at=f"stamp-{generation}",
        image=object(),
    )


def _text(text, received, trigger_generation=None, anomaly_candidate=None):
    if anomaly_candidate is None:
        anomaly_candidate = trigger_generation is not None
    return CachedText(
        text=text,
        received_monotonic=received,
        trigger_generation=trigger_generation,
        anomaly_candidate=anomaly_candidate,
    )


def test_snapshot_selects_fresh_images_in_generation_order():
    """Only fresh frames are returned, ordered by their generation."""
    cache = {
        "rear": _cached("rear", 3, 98.0),
        "stale": _cached("stale", 1, 94.9),
        "front": _cached("front", 2, 95.0),
    }

    snapshot = _snapshot_fresh_images(
        cache,
        now_monotonic=100.0,
        max_age_seconds=5.0,
    )

    assert [cached.source for cached in snapshot] == ["front", "rear"]


def test_snapshot_can_include_multiple_pre_event_frames_from_same_camera():
    """A trigger receives history, rather than only the latest camera frame."""
    cache = {
        1: _cached("front", 1, 97.0),
        2: _cached("rear", 2, 98.0),
        3: _cached("front", 3, 99.0),
    }

    snapshot = _snapshot_fresh_images(cache, 100.0, 6.0)

    assert [(item.source, item.generation) for item in snapshot] == [
        ("front", 1),
        ("rear", 2),
        ("front", 3),
    ]


def test_snapshot_age_limit_can_be_disabled():
    """A non-positive age limit keeps all cached frames eligible."""
    cache = {"front": _cached("front", 1, 1.0)}

    snapshot = _snapshot_fresh_images(
        cache,
        now_monotonic=100.0,
        max_age_seconds=0.0,
    )

    assert snapshot == [cache["front"]]


def test_consuming_snapshot_preserves_newer_replacement_and_new_source():
    """Consuming an old snapshot cannot erase frames that arrived later."""
    old_front = _cached("front", 1, 10.0)
    rear = _cached("rear", 2, 10.0)
    cache = {"front": old_front, "rear": rear}
    snapshot = [old_front, rear]

    new_front = _cached("front", 3, 11.0)
    side = _cached("side", 4, 11.0)
    cache["front"] = new_front
    cache["side"] = side

    _consume_image_snapshot(cache, snapshot)

    assert cache == {"front": new_front, "side": side}


def test_image_state_is_ignored_when_vision_is_disabled():
    """A camera cache cannot initiate an LLM request with vision disabled."""
    class Logger:
        def __init__(self):
            self.messages = []

        def debug(self, message):
            self.messages.append(message)

    logger = Logger()
    fake_node = SimpleNamespace(
        queue=deque(),
        _queue_lock=threading.Lock(),
        _error_capture_lock=threading.Lock(),
        _error_capture_timer=None,
        cache_max_age_seconds=30.0,
        image_queue={"front": _cached("front", 1, 1.0)},
        _inference_trigger_generation=0,
        _processed_inference_trigger_generation=0,
        get_logger=lambda: logger,
    )

    AnomalyDetectionNode.llm_callback(fake_node)

    assert logger.messages
    assert "no queued text" in logger.messages[0]


def test_info_only_context_does_not_schedule_inference():
    """INFO context waits for a warning, error, or explicit trigger."""
    class Logger:
        def __init__(self):
            self.messages = []

        def debug(self, message):
            self.messages.append(message)

    logger = Logger()
    fake_node = SimpleNamespace(
        queue=deque([_text("healthy context", 1.0)]),
        _queue_lock=threading.Lock(),
        _error_capture_lock=threading.Lock(),
        _error_capture_timer=None,
        cache_max_age_seconds=0.0,
        _inference_trigger_generation=0,
        _processed_inference_trigger_generation=0,
        get_logger=lambda: logger,
    )

    AnomalyDetectionNode.llm_callback(fake_node)

    assert logger.messages
    assert "Retaining INFO" in logger.messages[0]


def test_actionable_context_schedules_one_worker():
    """A new actionable generation submits a single worker."""
    submitted = []
    worker = object()
    fake_node = SimpleNamespace(
        queue=deque([_text("warning context", 1.0, trigger_generation=1)]),
        _queue_lock=threading.Lock(),
        _error_capture_lock=threading.Lock(),
        _error_capture_timer=None,
        cache_max_age_seconds=0.0,
        _llm_state_lock=threading.Lock(),
        _llm_worker_shutdown=False,
        _llm_worker_running=False,
        _llm_rerun_requested=False,
        _inference_trigger_generation=1,
        _processed_inference_trigger_generation=0,
        _llm_worker_loop=worker,
        _llm_executor=SimpleNamespace(
            submit=lambda callback: submitted.append(callback)
        ),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )

    AnomalyDetectionNode.llm_callback(fake_node)

    assert submitted == [worker]
    assert fake_node._llm_worker_running is True


def test_stale_text_is_removed_and_cannot_leave_a_ghost_trigger():
    """Expired actionable text cannot initiate an INFO-only inference."""
    queue = deque(
        [
            _text("old warning", 90.0, trigger_generation=1),
            _text("fresh info", 99.0),
        ]
    )

    removed = _discard_stale_text(queue, 100.0, 5.0)

    assert removed == 1
    assert list(queue) == [_text("fresh info", 99.0)]
    assert _pending_trigger_generation(list(queue), 0) is None


def test_pending_trigger_generation_tracks_only_unprocessed_entries():
    """The newest unprocessed token wins without relying on a global flag."""
    items = [
        _text("info", 1.0),
        _text("old warning", 2.0, trigger_generation=2),
        _text("new error", 3.0, trigger_generation=4),
    ]

    assert _pending_trigger_generation(items, processed_generation=2) == 4
    assert _pending_trigger_generation(items, processed_generation=4) is None


def _fake_processing_node(chat_callback):
    event = _text("warning context", 1.0, trigger_generation=1)
    artifacts = []
    logger = SimpleNamespace(
        debug=lambda message: None,
        info=lambda message: None,
        warn=lambda message: None,
        error=lambda message: None,
    )
    node = SimpleNamespace(
        queue=deque([event]),
        _queue_lock=threading.Lock(),
        cache_max_age_seconds=0.0,
        _processed_inference_trigger_generation=0,
        vision_enabled=False,
        _image_queue_lock=threading.Lock(),
        image_queue={},
        image_max_age_seconds=3.0,
        llm=SimpleNamespace(local_chat=chat_callback, chat=chat_callback),
        llm_local=True,
        llm_called_pub=SimpleNamespace(publish=lambda message: None),
        decision_pub=object(),
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=123)
        ),
        _publish_text=lambda publisher, message: None,
        _publish_decision=lambda decision: None,
        _queue_items_match=(
            lambda current, expected: AnomalyDetectionNode._queue_items_match(
                None,
                current,
                expected,
            )
        ),
        _write_api_artifact=lambda *args, **kwargs: artifacts.append(
            (args, kwargs)
        ),
    )
    node._remove_queue_snapshot = (
        lambda queue, lock, snapshot: AnomalyDetectionNode._remove_queue_snapshot(
            node,
            queue,
            lock,
            snapshot,
        )
    )
    return node, event, artifacts


def test_backend_failure_preserves_claimed_event_context():
    """A timeout does not discard the event, and it cannot hot-loop."""
    def fail_chat(text, images=None):
        raise TimeoutError("backend timed out")

    node, event, artifacts = _fake_processing_node(fail_chat)

    AnomalyDetectionNode._process_llm_queue(node)

    assert list(node.queue) == [event]
    assert node._processed_inference_trigger_generation == 1
    assert _pending_trigger_generation(list(node.queue), 1) is None
    assert len(artifacts) == 1


def test_successful_backend_call_consumes_only_its_text_snapshot():
    """A successful response removes the exact claimed text snapshot."""
    def successful_chat(text, images=None):
        return '{"anomaly": false, "action": "none", "summary": "normal"}'

    node, _, artifacts = _fake_processing_node(successful_chat)

    AnomalyDetectionNode._process_llm_queue(node)

    assert list(node.queue) == []
    assert node._processed_inference_trigger_generation == 1
    assert len(artifacts) == 1


def test_vision_enabled_without_a_frame_falls_back_to_text():
    """A missing camera frame must not block an actionable LLM request."""
    model_calls = []

    def successful_chat(text, images=None):
        model_calls.append((text, images))
        return '{"anomaly": false, "action": "none", "summary": "normal"}'

    node, _, _ = _fake_processing_node(successful_chat)
    node.vision_enabled = True

    AnomalyDetectionNode._process_llm_queue(node)

    assert len(model_calls) == 1
    assert model_calls[0][1] == []
    assert list(node.queue) == []


def test_pre_event_images_reach_model_with_an_actionable_event(monkeypatch):
    """An actionable event receives the richer pre-event image history."""
    model_calls = []
    artifacts = []
    logger = SimpleNamespace(
        debug=lambda message: None,
        info=lambda message: None,
        warn=lambda message: None,
        error=lambda message: None,
    )
    prepared = SimpleNamespace(
        original_width=640,
        original_height=480,
        width=640,
        height=480,
        mime_type="image/jpeg",
        byte_size=100,
        sha256="test",
    )
    node = SimpleNamespace(
        queue=deque(),
        _queue_lock=threading.Lock(),
        _error_capture_lock=threading.Lock(),
        _error_capture_timer=None,
        cache_max_age_seconds=0.0,
        _inference_trigger_generation=0,
        _processed_inference_trigger_generation=0,
        vision_enabled=True,
        _image_queue_lock=threading.Lock(),
        image_queue={
            1: _cached("front", 1, 97.0),
            2: _cached("rear", 2, 98.0),
            3: _cached("front", 3, 99.0),
        },
        image_max_age_seconds=6.0,
        llm=SimpleNamespace(
            prepare_images=lambda images: [prepared],
            local_chat=lambda text, images=None: (
                model_calls.append((text, images))
                or '{"anomaly": false, "action": "none", "summary": "normal"}'
            ),
        ),
        llm_local=True,
        llm_called_pub=SimpleNamespace(publish=lambda message: None),
        decision_pub=object(),
        get_logger=lambda: logger,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=123)
        ),
        _publish_text=lambda publisher, message: None,
        _publish_decision=lambda decision: None,
        _write_api_artifact=lambda *args, **kwargs: artifacts.append(
            (args, kwargs)
        ),
    )
    node._queue_items_match = (
        lambda current, expected: AnomalyDetectionNode._queue_items_match(
            None, current, expected
        )
    )
    node._remove_queue_snapshot = (
        lambda queue, lock, snapshot: AnomalyDetectionNode._remove_queue_snapshot(
            node, queue, lock, snapshot
        )
    )

    # This small unit fixture omits enabled vision scheduling; image history
    # remains available for the later actionable processing pass.
    AnomalyDetectionNode.llm_callback(node)
    assert model_calls == []
    assert len(node.image_queue) == 3

    # A later warning is the trigger that makes the pre-event sequence useful.
    node.queue.append(_text("warning: obstacle nearby", 100.0, trigger_generation=1))
    node._inference_trigger_generation = 1
    monkeypatch.setattr(
        "anomaly_detection.anomaly_detection_node.time.monotonic",
        lambda: 100.0,
    )
    AnomalyDetectionNode._process_llm_queue(node)

    assert len(model_calls) == 1
    prompt, images = model_calls[0]
    assert prompt.startswith("warning: obstacle nearby")
    assert [item for item in images] == [prepared, prepared, prepared]
    assert "Image 1: source=front captured_at=stamp-1" in prompt
    assert "Image 2: source=rear captured_at=stamp-2" in prompt
    assert "Image 3: source=front captured_at=stamp-3" in prompt
    # Successful inference retains the latest ZED context for the next request.
    assert sorted(node.image_queue) == [1, 2, 3]
    assert artifacts[0][1]["image_metadata"][0]["source"] == "front"


def test_periodic_context_pass_sends_only_the_configured_small_sample(monkeypatch):
    """Routine visual context uses the newest frame instead of full history."""
    model_calls = []
    prepared = SimpleNamespace(
        original_width=640,
        original_height=480,
        width=640,
        height=480,
        mime_type="image/jpeg",
        byte_size=100,
        sha256="test",
    )
    context = _text(
        "Periodic camera context observation.",
        100.0,
        trigger_generation=1,
        anomaly_candidate=False,
    )
    node = SimpleNamespace(
        queue=deque([context]),
        _queue_lock=threading.Lock(),
        cache_max_age_seconds=0.0,
        _processed_inference_trigger_generation=0,
        vision_enabled=True,
        _image_queue_lock=threading.Lock(),
        image_queue={
            1: _cached("front", 1, 97.0),
            2: _cached("rear", 2, 98.0),
            3: _cached("front", 3, 99.0),
        },
        image_max_age_seconds=6.0,
        image_context_max_frames=1,
        llm=SimpleNamespace(
            prepare_images=lambda images: [prepared],
            local_chat=lambda text, images=None: (
                model_calls.append((text, images))
                or '{"anomaly": false, "action": "none", "summary": "normal"}'
            ),
        ),
        llm_local=True,
        llm_called_pub=SimpleNamespace(publish=lambda message: None),
        decision_pub=object(),
        get_logger=lambda: SimpleNamespace(
            debug=lambda message: None,
            info=lambda message: None,
            warn=lambda message: None,
            error=lambda message: None,
        ),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=123)
        ),
        _publish_text=lambda publisher, message: None,
        _publish_decision=lambda decision: None,
        _write_api_artifact=lambda *args, **kwargs: None,
    )
    node._queue_items_match = (
        lambda current, expected: AnomalyDetectionNode._queue_items_match(
            None, current, expected
        )
    )
    node._remove_queue_snapshot = (
        lambda queue, lock, snapshot: AnomalyDetectionNode._remove_queue_snapshot(
            node, queue, lock, snapshot
        )
    )
    monkeypatch.setattr(
        "anomaly_detection.anomaly_detection_node.time.monotonic",
        lambda: 100.0,
    )

    AnomalyDetectionNode._process_llm_queue(node)

    assert len(model_calls) == 1
    prompt, images = model_calls[0]
    assert images == [prepared]
    assert "Image 1: source=front captured_at=stamp-3" in prompt
    # Routine observations also retain their frame for subsequent requests.
    assert sorted(node.image_queue) == [1, 2, 3]


def test_trigger_importance_config_accepts_names_and_safe_defaults():
    """Trigger thresholds support readable names and reject bad values."""
    assert _parse_trigger_importance("info") == 0
    assert _parse_trigger_importance("WARNING") == 1
    assert _parse_trigger_importance(2) == 2
    assert _parse_trigger_importance("invalid") == 1
    assert _parse_trigger_importance(99) == 1


def test_artifact_payload_contains_metadata_but_no_image_payload():
    """Artifacts record image metadata without image data or base64."""
    payload = _build_artifact_payload(
        artifact_id="api_artifact_1",
        created_at="2026-07-20T00:00:00+00:00",
        timestamp_ns=1,
        cached_data=["warning text"],
        image_metadata=[
            {
                "source": "camera:front",
                "encoded_bytes": 1234,
                "sha256": "abc",
            }
        ],
        api_response='{"anomaly": false}',
    )

    serialized = json.dumps(payload)
    assert payload["cached_data"] == ["warning text"]
    assert payload["images"][0]["source"] == "camera:front"
    assert "base64" not in serialized
    assert "data:image" not in serialized


def test_artifact_pruning_is_bounded_and_preserves_unrelated_files(tmp_path):
    """Retention removes only the oldest generated artifacts."""
    artifacts = []
    for index in range(4):
        path = tmp_path / f"api_artifact_{index}.json"
        path.write_text("{}", encoding="utf-8")
        timestamp = 100 + index
        os.utime(path, ns=(timestamp, timestamp))
        artifacts.append(path)

    unrelated = tmp_path / "other.json"
    unrelated.write_text("{}", encoding="utf-8")

    removed = _prune_api_artifacts(
        str(tmp_path),
        max_files=2,
        keep_path=str(artifacts[0]),
    )

    remaining = sorted(tmp_path.glob("api_artifact_*.json"))
    assert len(remaining) == 2
    assert artifacts[0] in remaining
    assert artifacts[3] in remaining
    assert len(removed) == 2
    assert unrelated.exists()


def test_artifact_pruning_can_be_disabled(tmp_path):
    """A zero retention limit disables pruning."""
    artifact = tmp_path / "api_artifact_1.json"
    artifact.write_text("{}", encoding="utf-8")

    assert _prune_api_artifacts(str(tmp_path), 0, str(artifact)) == []
    assert artifact.exists()
