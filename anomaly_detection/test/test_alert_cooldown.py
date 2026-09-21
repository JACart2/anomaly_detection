"""Tests for incident-level immediate alert suppression."""

import threading
from types import SimpleNamespace

from anomaly_detection.anomaly_detection_node import AnomalyDetectionNode


def _node():
    published = []
    logger = SimpleNamespace(
        debug=lambda message: None,
        info=lambda message: None,
    )
    node = SimpleNamespace(
        immediate_alert_min_period_sec=10.0,
        _alert_state_lock=threading.Lock(),
        _last_immediate_alert_time=None,
        decision_pub="decision",
        alert_pub="alert",
        get_logger=lambda: logger,
        _publish_text=lambda publisher, text: published.append((publisher, text)),
    )
    node._immediate_alert_cooldown_active = (
        lambda now=None: AnomalyDetectionNode._immediate_alert_cooldown_active(
            node,
            now,
        )
    )
    return node, published


def test_repeated_immediate_stop_is_published_once(monkeypatch):
    """A burst of ERROR telemetry produces one immediate incident alert."""
    node, published = _node()
    times = iter((100.0, 100.1))
    monkeypatch.setattr(
        "anomaly_detection.anomaly_detection_node.time.monotonic",
        lambda: next(times),
    )

    AnomalyDetectionNode._publish_immediate_stop(node, "braking active")
    AnomalyDetectionNode._publish_immediate_stop(node, "braking active update")

    assert len(published) == 2
    assert [publisher for publisher, _ in published] == ["decision", "alert"]


def test_llm_stop_alert_is_suppressed_after_immediate_alert():
    """The LLM decision remains published without duplicating the stop alert."""
    node, published = _node()
    node._last_immediate_alert_time = 100.0
    decision = SimpleNamespace(
        anomaly=True,
        severity="high",
        action="stop_cart",
        summary="confirmed braking incident",
    )
    node._immediate_alert_cooldown_active = lambda now=None: True

    AnomalyDetectionNode._publish_decision(node, decision)

    assert len(published) == 1
    assert published[0][0] == "decision"
    assert "confirmed braking incident" in published[0][1]
