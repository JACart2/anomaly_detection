#!/usr/bin/env python3
"""Compare human anomaly labels with detector results across ROS 2 bags.

The script is intentionally standalone: discovery, deserialization, timestamp
parsing, global bag pairing, event matching, metrics, and report generation all
live in this file. It never replays a bag or starts the anomaly detector.

Detector artifacts use two distinct clocks which must not be conflated:
source messages contain strings such as ``[t=1784050861.875419090 ...]`` while
the artifact ``timestamp_ns`` records when the response was returned. The first
is used to match an event; the second is used to calculate processing latency.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # Keep --help useful outside a sourced ROS shell.
    rosbag2_py = None  # type: ignore[assignment]
    deserialize_message = None  # type: ignore[assignment]
    get_message = None  # type: ignore[assignment]
    ROS_IMPORT_ERROR: ImportError | None = exc
else:
    ROS_IMPORT_ERROR = None


NS_PER_SECOND = 1_000_000_000
REPORT_SCHEMA_VERSION = 1

DEFAULT_DETECTOR_TOPICS = (
    "/aad/decisions",
    "/aad/api_response",
    "/aad/cache_snapshot",
    "/aad/api_artifact",
)
DEFAULT_CONTEXT_TOPICS = (
    "/aad/formatted_messages",
)

# Automatic label discovery is deliberately conservative. Generic anomaly log
# topics are not human ground truth and must never be guessed to be labels.
DEFAULT_LABEL_TOPIC_PATTERN = (
    r"(?:^|/)(?:human[_-]?(?:anomaly[_-]?)?labels?|"
    r"ground[_-]?truth|annotations?)(?:$|/)"
)

EMBEDDED_T_PATTERN = re.compile(
    r"\[\s*t\s*=\s*(?P<seconds>-?\d+)"
    r"(?:\.(?P<fraction>\d{1,9}))?(?=\s|\])",
    re.IGNORECASE,
)
NAMED_SECONDS_PATTERN = re.compile(
    r"\b(?:event|label|source)?_?timestamp\s*[:=]\s*"
    r"(?P<seconds>-?\d+)(?:\.(?P<fraction>\d{1,9}))?\b",
    re.IGNORECASE,
)
ANOMALY_PATTERN = re.compile(
    r"\banomaly\s*[:=]\s*(?P<value>true|false|yes|no|1|0)\b",
    re.IGNORECASE,
)

TRUE_STRINGS = {"true", "yes", "1", "anomaly", "positive"}
FALSE_STRINGS = {"false", "no", "0", "normal", "negative"}
ANOMALY_KEYS = (
    "anomaly",
    "is_anomaly",
    "human_label",
    "ground_truth",
    "label",
)
EVENT_NS_KEYS = (
    "event_timestamp_ns",
    "label_timestamp_ns",
    "source_timestamp_ns",
    "timestamp_ns",
)
EVENT_TIME_KEYS = (
    "event_timestamp",
    "label_timestamp",
    "source_timestamp",
    "timestamp",
)


class EvaluationError(RuntimeError):
    """Raised for a fatal input or environment error."""


@dataclass(frozen=True)
class TimestampEvidence:
    """A timestamp plus the source used to obtain it."""

    ns: int
    source: str
    raw: str


@dataclass
class HumanLabel:
    """One positive human ground-truth marker."""

    identifier: str
    event_time: TimestampEvidence
    topic: str
    record_timestamp_ns: int
    raw: str


@dataclass
class DetectorResult:
    """One detector response and all possible source events in its context."""

    identifier: str
    anomaly: bool | None
    event_times: list[TimestampEvidence]
    return_time: TimestampEvidence
    topic: str
    record_timestamp_ns: int
    raw_context: list[str]
    raw_response: str
    attribution_method: str


@dataclass
class ContextMessage:
    """A formatted source message recorded before a plain decision."""

    record_timestamp_ns: int
    event_times: list[TimestampEvidence]
    raw: str


@dataclass
class ArtifactAccumulator:
    """Collect artifact pieces that may arrive on multiple ROS topics."""

    identifier: str
    topic: str
    first_record_timestamp_ns: int
    contexts: list[str] = field(default_factory=list)
    response_values: list[Any] = field(default_factory=list)
    return_time: TimestampEvidence | None = None
    response_record_timestamp_ns: int | None = None
    raw_payloads: list[str] = field(default_factory=list)


@dataclass
class BagProfile:
    """All information extracted from one logical ROS bag."""

    path: Path
    topic_types: dict[str, str] = field(default_factory=dict)
    labels: list[HumanLabel] = field(default_factory=list)
    detector_results: list[DetectorResult] = field(default_factory=list)
    source_timestamps_ns: list[int] = field(default_factory=list)
    header_timestamps_ns: list[int] = field(default_factory=list)
    record_start_ns: int | None = None
    record_end_ns: int | None = None
    role: str = "unknown"
    label_topics_present: list[str] = field(default_factory=list)
    detector_topics_present: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def source_interval(self) -> tuple[int, int] | None:
        """Return source-clock coverage, preferring embedded over headers."""
        if self.source_timestamps_ns:
            return min(self.source_timestamps_ns), max(self.source_timestamps_ns)
        if self.header_timestamps_ns:
            return min(self.header_timestamps_ns), max(self.header_timestamps_ns)
        return None

    def record_interval(self) -> tuple[int, int] | None:
        """Return rosbag record-clock coverage."""
        if self.record_start_ns is None or self.record_end_ns is None:
            return None
        return self.record_start_ns, self.record_end_ns


@dataclass(frozen=True)
class PairCandidate:
    """A possible human/detector bag pairing and its deterministic cost."""

    human_index: int
    detector_index: int
    cost: int
    time_domain: str
    human_start_ns: int
    human_end_ns: int
    detector_start_ns: int
    detector_end_ns: int
    overlap_ns: int
    gap_ns: int


@dataclass
class FlowEdge:
    """Mutable residual edge used by the dependency-free matching solver."""

    to_node: int
    reverse_index: int
    capacity: int
    cost: int


def seconds_to_ns(seconds: float) -> int:
    """Convert CLI seconds to integer nanoseconds."""
    return int(round(seconds * NS_PER_SECOND))


def decimal_seconds_to_ns(seconds: str, fraction: str | None) -> int:
    """Convert a decimal-seconds regex match without using a float."""
    whole = int(seconds)
    fractional_ns = int((fraction or "").ljust(9, "0") or "0")
    if whole < 0:
        return whole * NS_PER_SECOND - fractional_ns
    return whole * NS_PER_SECOND + fractional_ns


def unique_timestamps(values: Iterable[TimestampEvidence]) -> list[TimestampEvidence]:
    """Deduplicate equal times while preserving the first provenance record."""
    result: list[TimestampEvidence] = []
    seen: set[int] = set()
    for value in values:
        if value.ns in seen:
            continue
        seen.add(value.ns)
        result.append(value)
    return result


def embedded_timestamps(text: str, source: str) -> list[TimestampEvidence]:
    """Extract every ``[t=sec.nsec ...]`` timestamp from a string."""
    values = []
    for match in EMBEDDED_T_PATTERN.finditer(text):
        raw = match.group(0)
        values.append(
            TimestampEvidence(
                ns=decimal_seconds_to_ns(
                    match.group("seconds"), match.group("fraction")
                ),
                source=source,
                raw=raw,
            )
        )
    return unique_timestamps(values)


def named_timestamp(text: str, source: str) -> TimestampEvidence | None:
    """Extract a named decimal timestamp used by some label strings."""
    match = NAMED_SECONDS_PATTERN.search(text)
    if match is None:
        return None
    return TimestampEvidence(
        ns=decimal_seconds_to_ns(match.group("seconds"), match.group("fraction")),
        source=source,
        raw=match.group(0),
    )


def coerce_ns(value: Any, explicitly_ns: bool) -> int | None:
    """Parse JSON timestamp values, inferring units only for generic fields."""
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, int):
        if explicitly_ns or abs(value) >= 100_000_000_000:
            return value
        return value * NS_PER_SECOND
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        if explicitly_ns or abs(value) >= 100_000_000_000:
            return int(round(value))
        return int(round(value * NS_PER_SECOND))
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            if explicitly_ns:
                return int(stripped)
            if "." in stripped:
                seconds, fraction = stripped.split(".", 1)
                if seconds.lstrip("-").isdigit() and fraction.isdigit():
                    return decimal_seconds_to_ns(seconds, fraction[:9])
            integer = int(stripped)
            if abs(integer) >= 100_000_000_000:
                return integer
            return integer * NS_PER_SECOND
        except ValueError:
            return None
    return None


def timestamp_from_mapping(
    payload: dict[str, Any], source_prefix: str
) -> TimestampEvidence | None:
    """Read the most specific event timestamp from a decoded label object."""
    for key in EVENT_NS_KEYS:
        if key in payload:
            value = coerce_ns(payload[key], explicitly_ns=True)
            if value is not None:
                return TimestampEvidence(value, f"{source_prefix}.{key}", str(payload[key]))
    for key in EVENT_TIME_KEYS:
        if key in payload:
            value = coerce_ns(payload[key], explicitly_ns=False)
            if value is not None:
                return TimestampEvidence(value, f"{source_prefix}.{key}", str(payload[key]))
    return None


def parse_json_object(text: str) -> dict[str, Any] | None:
    """Decode a JSON object, including a JSON string wrapped once in JSON."""
    stripped = text.strip()
    if not stripped:
        return None
    try:
        value: Any = json.loads(stripped)
        if isinstance(value, str):
            value = json.loads(value)
    except (json.JSONDecodeError, TypeError):
        return None
    return value if isinstance(value, dict) else None


def scalar_message_fields(message: Any) -> dict[str, Any]:
    """Extract safe scalar fields without expanding image or byte payloads."""
    getter = getattr(message, "get_fields_and_field_types", None)
    if not callable(getter):
        return {}
    values: dict[str, Any] = {}
    for name in getter():
        try:
            value = getattr(message, name)
        except (AttributeError, RuntimeError):
            continue
        if isinstance(value, (str, bool, int, float)):
            values[name] = value
    return values


def message_text(message: Any, scalar_fields: dict[str, Any]) -> str:
    """Select the human-readable payload from common ROS message fields."""
    for field_name in ("data", "msg", "message", "payload", "json", "label"):
        value = scalar_fields.get(field_name)
        if isinstance(value, str) and value.strip():
            return value
    if scalar_fields:
        return json.dumps(scalar_fields, sort_keys=True, separators=(",", ":"))
    return ""


def header_timestamp(message: Any) -> TimestampEvidence | None:
    """Read a ROS Header stamp when it is present and nonzero."""
    try:
        stamp = message.header.stamp
        value = int(stamp.sec) * NS_PER_SECOND + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if value == 0:
        return None
    return TimestampEvidence(value, "message_header", f"{stamp.sec}.{stamp.nanosec:09d}")


def coerce_bool(value: Any) -> bool | None:
    """Normalize supported boolean label/decision representations."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in TRUE_STRINGS:
            return True
        if normalized in FALSE_STRINGS:
            return False
    return None


def extract_anomaly_bool(value: Any) -> bool | None:
    """Find an explicit anomaly boolean in mappings, JSON, or key-value text."""
    if isinstance(value, dict):
        for key in ANOMALY_KEYS:
            if key in value:
                parsed = coerce_bool(value[key])
                if parsed is not None:
                    return parsed
        # Structured response wrappers are common in recorded artifact topics.
        for key in ("decision", "api_response", "response", "result"):
            if key in value:
                parsed = extract_anomaly_bool(value[key])
                if parsed is not None:
                    return parsed
        return None
    if isinstance(value, str):
        payload = parse_json_object(value)
        if payload is not None:
            parsed = extract_anomaly_bool(payload)
            if parsed is not None:
                return parsed
        match = ANOMALY_PATTERN.search(value)
        return coerce_bool(match.group("value")) if match else None
    return coerce_bool(value)


def looks_like_artifact(payload: dict[str, Any]) -> bool:
    """Return whether a JSON object is a complete or partial API artifact."""
    artifact_fields = {
        "cached_data",
        "cache",
        "api_response",
        "response",
        "decision",
    }
    return (
        "artifact_id" in payload and bool(artifact_fields.intersection(payload))
    ) or ("cached_data" in payload and "api_response" in payload)


def stringify_payload(value: Any) -> str:
    """Create stable raw report text from structured or scalar payloads."""
    if isinstance(value, str):
        return value
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"))
    except (TypeError, ValueError):
        return str(value)


def artifact_contexts(payload: dict[str, Any]) -> list[str]:
    """Extract cached/formatted message strings from artifact variants."""
    for key in ("cached_data", "cache", "cached_messages", "context"):
        value = payload.get(key)
        if isinstance(value, list):
            return [stringify_payload(item) for item in value]
    return []


def artifact_responses(payload: dict[str, Any]) -> list[Any]:
    """Extract response values in order of structured reliability."""
    result = []
    for key in ("decision", "api_response", "response", "result"):
        if key in payload:
            result.append(payload[key])
    return result


def explicit_return_timestamp(
    payload: dict[str, Any], source_prefix: str
) -> TimestampEvidence | None:
    """Extract an explicit response time from an artifact payload."""
    for key in ("response_timestamp_ns", "return_timestamp_ns", "timestamp_ns"):
        if key in payload:
            value = coerce_ns(payload[key], explicitly_ns=True)
            if value is not None:
                return TimestampEvidence(value, f"{source_prefix}.{key}", str(payload[key]))
    for key in ("response_timestamp", "return_timestamp"):
        if key in payload:
            value = coerce_ns(payload[key], explicitly_ns=False)
            if value is not None:
                return TimestampEvidence(value, f"{source_prefix}.{key}", str(payload[key]))
    return None


def discover_bags(root: Path) -> list[Path]:
    """Discover logical bags without double-counting their split MCAP files."""
    if not root.exists():
        raise EvaluationError(f"Bag folder does not exist: {root}")
    if not root.is_dir():
        raise EvaluationError(f"--bag-dir must be a directory: {root}")

    bag_directories = sorted(
        {path.parent.resolve() for path in root.rglob("metadata.yaml")}
    )
    standalone: list[Path] = []
    for mcap_path in root.rglob("*.mcap"):
        resolved = mcap_path.resolve()
        if any(
            directory == resolved.parent or directory in resolved.parents
            for directory in bag_directories
        ):
            continue
        standalone.append(resolved)
    return sorted([*bag_directories, *set(standalone)], key=lambda value: str(value))


def topic_matches(topic: str, configured: set[str]) -> bool:
    """Match a ROS topic exactly after normalizing an omitted leading slash."""
    normalized = topic if topic.startswith("/") else f"/{topic}"
    return normalized in configured


def normalize_topics(topics: Sequence[str]) -> set[str]:
    """Normalize CLI topic names to conventional absolute ROS names."""
    return {topic if topic.startswith("/") else f"/{topic}" for topic in topics}


def is_human_named_bag(path: Path) -> bool:
    """Return whether the logical bag name marks it as human-labeled."""
    return "human" in path.name.casefold()


def label_from_message(
    *,
    identifier: str,
    topic: str,
    text: str,
    payload: dict[str, Any] | None,
    scalar_fields: dict[str, Any],
    header_time: TimestampEvidence | None,
    record_timestamp_ns: int,
    positive_pattern: re.Pattern[str] | None,
    require_explicit_positive: bool,
) -> HumanLabel | None:
    """Convert one message on a human-label topic into a positive label."""
    if positive_pattern is not None and positive_pattern.search(text) is None:
        return None

    structured: Any = payload if payload is not None else scalar_fields
    explicit_value = extract_anomaly_bool(structured)
    if explicit_value is None:
        explicit_value = extract_anomaly_bool(text)
    if explicit_value is False:
        return None
    if explicit_value is None and positive_pattern is None and require_explicit_positive:
        return None

    event_time = timestamp_from_mapping(payload, "label_json") if payload else None
    if event_time is None:
        embedded = embedded_timestamps(text, "label_message[t]")
        event_time = embedded[0] if embedded else None
    if event_time is None:
        event_time = named_timestamp(text, "label_message.timestamp")
    if event_time is None:
        event_time = header_time
    if event_time is None:
        event_time = TimestampEvidence(
            record_timestamp_ns,
            "label_bag_record_fallback",
            str(record_timestamp_ns),
        )

    return HumanLabel(
        identifier=identifier,
        event_time=event_time,
        topic=topic,
        record_timestamp_ns=record_timestamp_ns,
        raw=text or stringify_payload(scalar_fields),
    )


def ingest_artifact(
    accumulator: ArtifactAccumulator,
    payload: dict[str, Any],
    raw: str,
    record_timestamp_ns: int,
) -> None:
    """Merge one complete/partial artifact message by artifact ID."""
    for context in artifact_contexts(payload):
        if context not in accumulator.contexts:
            accumulator.contexts.append(context)

    responses = artifact_responses(payload)
    for response in responses:
        rendered = stringify_payload(response)
        if all(
            stringify_payload(existing) != rendered
            for existing in accumulator.response_values
        ):
            accumulator.response_values.append(response)

    if responses:
        accumulator.response_record_timestamp_ns = record_timestamp_ns
        timestamp = explicit_return_timestamp(payload, "artifact")
        if timestamp is not None:
            accumulator.return_time = timestamp
    accumulator.raw_payloads.append(raw)


def finalize_artifact(
    accumulator: ArtifactAccumulator,
    warning_target: list[str],
) -> DetectorResult | None:
    """Build one detector result after all artifact pieces have been read."""
    response: Any | None = None
    anomaly: bool | None = None
    for candidate in accumulator.response_values:
        parsed = extract_anomaly_bool(candidate)
        if parsed is not None:
            response = candidate
            anomaly = parsed
            break
    if response is None and accumulator.response_values:
        response = accumulator.response_values[0]
    if response is None:
        warning_target.append(
            f"Artifact {accumulator.identifier!r} has context but no response; ignored."
        )
        return None

    record_ns = (
        accumulator.response_record_timestamp_ns
        if accumulator.response_record_timestamp_ns is not None
        else accumulator.first_record_timestamp_ns
    )
    return_time = accumulator.return_time or TimestampEvidence(
        record_ns,
        "detector_bag_record_fallback",
        str(record_ns),
    )
    event_times: list[TimestampEvidence] = []
    for context in accumulator.contexts:
        event_times.extend(embedded_timestamps(context, "artifact.cached_data[t]"))
    event_times = unique_timestamps(event_times)
    attribution = "artifact_cached_candidates"
    if not event_times:
        # A result remains usable, but its zero processing latency is explicitly
        # marked as a fallback rather than presented as a measured source delay.
        event_times = [
            TimestampEvidence(record_ns, "detector_bag_record_fallback", str(record_ns))
        ]
        attribution = "return_record_time_fallback"
        warning_target.append(
            f"Artifact {accumulator.identifier!r} has no embedded source timestamp; "
            "using its response record time as event time."
        )
    if anomaly is None:
        warning_target.append(
            f"Artifact {accumulator.identifier!r} has no parseable anomaly boolean."
        )

    return DetectorResult(
        identifier=accumulator.identifier,
        anomaly=anomaly,
        event_times=event_times,
        return_time=return_time,
        topic=accumulator.topic,
        record_timestamp_ns=record_ns,
        raw_context=list(accumulator.contexts),
        raw_response=stringify_payload(response),
        attribution_method=attribution,
    )


def recent_context_times(
    contexts: Sequence[ContextMessage],
    decision_record_ns: int,
    lookback_ns: int,
) -> tuple[list[TimestampEvidence], list[str]]:
    """Associate a plain decision with recent formatted messages.

    Current production bags do not record the standalone artifact JSON. This
    bounded association is therefore a declared heuristic, not a causal claim.
    """
    selected_times: list[TimestampEvidence] = []
    selected_raw: list[str] = []
    lower_bound = decision_record_ns - lookback_ns
    for context in contexts:
        if lower_bound <= context.record_timestamp_ns <= decision_record_ns:
            selected_times.extend(context.event_times)
            selected_raw.append(context.raw)
    return unique_timestamps(selected_times), selected_raw


def scan_bag(
    path: Path,
    *,
    configured_label_topics: set[str],
    label_topic_pattern: re.Pattern[str],
    label_positive_pattern: re.Pattern[str] | None,
    detector_topics: set[str],
    context_topics: set[str],
    context_lookback_ns: int,
) -> BagProfile:
    """Read and profile one bag while isolating message-level failures."""
    if rosbag2_py is None or deserialize_message is None or get_message is None:
        raise EvaluationError(
            "ROS 2 Python libraries are unavailable. Source the ROS/workspace "
            f"environment before running this script. Import error: {ROS_IMPORT_ERROR}"
        )

    profile = BagProfile(path=path)
    human_named = is_human_named_bag(path)
    try:
        metadata = rosbag2_py.Info().read_metadata(str(path), "mcap")
        profile.record_start_ns = int(metadata.starting_time.nanoseconds)
        profile.record_end_ns = profile.record_start_ns + int(metadata.duration.nanoseconds)
    except Exception as exc:
        profile.warnings.append(f"Could not read metadata directly: {exc}")

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        profile.topic_types = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
    except Exception as exc:
        profile.errors.append(f"Could not open bag: {exc}")
        return profile

    if configured_label_topics:
        declared_label_topics = {
            topic for topic in profile.topic_types if topic_matches(topic, configured_label_topics)
        }
    else:
        declared_label_topics = {
            topic for topic in profile.topic_types if label_topic_pattern.search(topic)
        }
    # A human-named bag is authoritative ground truth. If it lacks a dedicated
    # label topic, inspect all textual messages but accept only explicit positive
    # markers (or the user-supplied positive regex), never ordinary warnings.
    fallback_label_scan = human_named and not declared_label_topics
    label_scan_topics = (
        set(profile.topic_types) if fallback_label_scan else declared_label_topics
    )
    detected_topics = {
        topic for topic in profile.topic_types if topic_matches(topic, detector_topics)
    }
    supporting_topics = {
        topic for topic in profile.topic_types if topic_matches(topic, context_topics)
    }
    profile.label_topics_present = sorted(declared_label_topics)
    profile.detector_topics_present = sorted(detected_topics | supporting_topics)

    type_cache: dict[str, Any] = {}
    unavailable_types: set[str] = set()
    artifact_groups: dict[str, ArtifactAccumulator] = {}
    contexts: list[ContextMessage] = []
    label_counter = 0
    result_counter = 0
    message_counter = 0

    while True:
        try:
            if not reader.has_next():
                break
            topic, serialized, record_ns = reader.read_next()
        except Exception as exc:
            profile.errors.append(f"Bag read stopped after {message_counter} messages: {exc}")
            break

        message_counter += 1
        record_ns = int(record_ns)
        profile.record_start_ns = (
            record_ns
            if profile.record_start_ns is None
            else min(profile.record_start_ns, record_ns)
        )
        profile.record_end_ns = (
            record_ns if profile.record_end_ns is None else max(profile.record_end_ns, record_ns)
        )
        type_name = profile.topic_types.get(topic)
        if not type_name or type_name in unavailable_types:
            continue
        if type_name not in type_cache:
            try:
                type_cache[type_name] = get_message(type_name)
            except Exception as exc:
                unavailable_types.add(type_name)
                profile.warnings.append(
                    f"Cannot load ROS type {type_name!r}; its messages are skipped: {exc}"
                )
                continue
        try:
            message = deserialize_message(serialized, type_cache[type_name])
        except Exception as exc:
            profile.warnings.append(
                f"Could not deserialize message {message_counter} on {topic}: {exc}"
            )
            continue

        scalar_fields = scalar_message_fields(message)
        text = message_text(message, scalar_fields)
        payload = parse_json_object(text)
        header_time = header_timestamp(message)
        if header_time is not None:
            profile.header_timestamps_ns.append(header_time.ns)

        embedded = embedded_timestamps(text, f"{topic}[t]")
        profile.source_timestamps_ns.extend(value.ns for value in embedded)

        if topic in label_scan_topics:
            label_counter += 1
            label = label_from_message(
                identifier=f"{path.name}:label:{label_counter}",
                topic=topic,
                text=text,
                payload=payload,
                scalar_fields=scalar_fields,
                header_time=header_time,
                record_timestamp_ns=record_ns,
                positive_pattern=label_positive_pattern,
                require_explicit_positive=fallback_label_scan,
            )
            if label is not None:
                profile.labels.append(label)
                profile.source_timestamps_ns.append(label.event_time.ns)

        if topic in supporting_topics and embedded:
            contexts.append(ContextMessage(record_ns, embedded, text))

        if payload is not None and looks_like_artifact(payload):
            raw_id = payload.get("artifact_id")
            artifact_id = str(raw_id) if raw_id else f"{path.name}:artifact:{record_ns}"
            accumulator = artifact_groups.setdefault(
                artifact_id,
                ArtifactAccumulator(artifact_id, topic, record_ns),
            )
            ingest_artifact(accumulator, payload, text, record_ns)
            if topic not in profile.detector_topics_present:
                profile.detector_topics_present.append(topic)
            continue

        if topic not in detected_topics:
            continue
        anomaly = extract_anomaly_bool(payload if payload is not None else text)
        if anomaly is None:
            profile.warnings.append(
                f"Detector message on {topic} at {record_ns} has no parseable anomaly boolean."
            )
            continue
        result_counter += 1
        event_times = list(embedded)
        raw_context: list[str] = []
        attribution = "decision_embedded_timestamp"
        if not event_times:
            event_times, raw_context = recent_context_times(
                contexts,
                decision_record_ns=record_ns,
                lookback_ns=context_lookback_ns,
            )
            attribution = "recent_formatted_messages_heuristic"
        if not event_times:
            event_times = [
                TimestampEvidence(record_ns, "detector_bag_record_fallback", str(record_ns))
            ]
            attribution = "return_record_time_fallback"
            profile.warnings.append(
                f"Decision {result_counter} on {topic} has no associated source timestamp; "
                "using its record time as event time."
            )
        profile.detector_results.append(
            DetectorResult(
                identifier=f"{path.name}:decision:{result_counter}",
                anomaly=anomaly,
                event_times=unique_timestamps(event_times),
                return_time=TimestampEvidence(
                    record_ns, "detector_bag_record_timestamp", str(record_ns)
                ),
                topic=topic,
                record_timestamp_ns=record_ns,
                raw_context=raw_context,
                raw_response=text,
                attribution_method=attribution,
            )
        )

    for accumulator in artifact_groups.values():
        result = finalize_artifact(accumulator, profile.warnings)
        if result is not None:
            profile.detector_results.append(result)
            profile.source_timestamps_ns.extend(value.ns for value in result.event_times)

    # The agreed filename convention is authoritative. A human copy may retain
    # detector topics from the original recording, but it remains ground truth.
    if human_named:
        profile.role = "human_labeled"
        if profile.detector_results:
            profile.warnings.append(
                "Bag contains both label and detector data; classified as human_labeled."
            )
    else:
        profile.role = "detector_output"

    if fallback_label_scan:
        profile.warnings.append(
            "No dedicated human-label topic was found; scanned the human-named "
            "bag for explicit positive label markers."
        )

    profile.label_topics_present.sort()
    profile.detector_topics_present.sort()
    profile.source_timestamps_ns = sorted(set(profile.source_timestamps_ns))
    profile.header_timestamps_ns = sorted(set(profile.header_timestamps_ns))
    return profile


def interval_overlap_and_gap(
    first: tuple[int, int], second: tuple[int, int]
) -> tuple[int, int]:
    """Return inclusive overlap length and separation between two intervals."""
    left = max(first[0], second[0])
    right = min(first[1], second[1])
    if left <= right:
        # One nanosecond gives point intervals meaningful containment.
        return max(1, right - left), 0
    return 0, left - right


def pair_candidate(
    human_index: int,
    detector_index: int,
    human: BagProfile,
    detector: BagProfile,
    max_gap_ns: int,
) -> PairCandidate | None:
    """Build a same-clock pairing candidate, falling back record-to-record."""
    human_interval = human.source_interval()
    detector_interval = detector.source_interval()
    time_domain = "source"
    if human_interval is None or detector_interval is None:
        human_interval = human.record_interval()
        detector_interval = detector.record_interval()
        time_domain = "bag_record_fallback"
    if human_interval is None or detector_interval is None:
        return None

    overlap_ns, gap_ns = interval_overlap_and_gap(human_interval, detector_interval)
    if gap_ns > max_gap_ns:
        return None

    human_midpoint = (human_interval[0] + human_interval[1]) // 2
    detector_midpoint = (detector_interval[0] + detector_interval[1]) // 2
    boundary_difference = abs(human_interval[0] - detector_interval[0]) + abs(
        human_interval[1] - detector_interval[1]
    )
    # Maximum-cardinality matching minimizes this cost. The overlap reward
    # favors true shared run coverage; boundary terms make ties deterministic.
    cost = (
        10 * gap_ns
        + abs(human_midpoint - detector_midpoint)
        + boundary_difference
        - 3 * overlap_ns
    )
    return PairCandidate(
        human_index=human_index,
        detector_index=detector_index,
        cost=cost,
        time_domain=time_domain,
        human_start_ns=human_interval[0],
        human_end_ns=human_interval[1],
        detector_start_ns=detector_interval[0],
        detector_end_ns=detector_interval[1],
        overlap_ns=overlap_ns,
        gap_ns=gap_ns,
    )


def add_flow_edge(
    graph: list[list[FlowEdge]], from_node: int, to_node: int, cost: int
) -> FlowEdge:
    """Add a unit-capacity edge and its residual reverse edge."""
    forward = FlowEdge(to_node, len(graph[to_node]), 1, cost)
    reverse = FlowEdge(from_node, len(graph[from_node]), 0, -cost)
    graph[from_node].append(forward)
    graph[to_node].append(reverse)
    return forward


def minimum_cost_maximum_matching(
    left_count: int,
    right_count: int,
    edge_costs: dict[tuple[int, int], int],
    excluded_edges: set[tuple[int, int]] | None = None,
) -> tuple[dict[int, int], int]:
    """Find maximum-cardinality then minimum-cost one-to-one matches.

    Successive shortest augmenting paths use Bellman-Ford because residual
    edges may have negative costs. Bag folders are small enough that avoiding a
    SciPy dependency is preferable to a more elaborate optimized solver.
    """
    excluded = excluded_edges or set()
    source = 0
    left_offset = 1
    right_offset = left_offset + left_count
    sink = right_offset + right_count
    node_count = sink + 1
    graph: list[list[FlowEdge]] = [[] for _ in range(node_count)]
    candidate_edges: dict[tuple[int, int], FlowEdge] = {}

    for left in range(left_count):
        add_flow_edge(graph, source, left_offset + left, 0)
    for right in range(right_count):
        add_flow_edge(graph, right_offset + right, sink, 0)
    for (left, right), cost in sorted(edge_costs.items()):
        if (left, right) in excluded:
            continue
        candidate_edges[(left, right)] = add_flow_edge(
            graph, left_offset + left, right_offset + right, cost
        )

    total_cost = 0
    while True:
        infinity = 10**100
        distances = [infinity] * node_count
        previous: list[tuple[int, int] | None] = [None] * node_count
        distances[source] = 0

        for _ in range(node_count - 1):
            changed = False
            for node, edges in enumerate(graph):
                if distances[node] == infinity:
                    continue
                for edge_index, edge in enumerate(edges):
                    if edge.capacity <= 0:
                        continue
                    candidate_distance = distances[node] + edge.cost
                    if candidate_distance < distances[edge.to_node]:
                        distances[edge.to_node] = candidate_distance
                        previous[edge.to_node] = (node, edge_index)
                        changed = True
            if not changed:
                break
        if previous[sink] is None:
            break

        node = sink
        while node != source:
            prior_node, edge_index = previous[node]  # type: ignore[misc]
            edge = graph[prior_node][edge_index]
            edge.capacity -= 1
            graph[node][edge.reverse_index].capacity += 1
            node = prior_node
        total_cost += distances[sink]

    matches = {
        left: right
        for (left, right), edge in candidate_edges.items()
        if edge.capacity == 0
    }
    return matches, total_cost


def select_bag_pairs(
    humans: Sequence[BagProfile],
    detectors: Sequence[BagProfile],
    *,
    max_gap_ns: int,
    ambiguity_margin_ns: int,
) -> tuple[list[tuple[BagProfile, BagProfile, PairCandidate]], list[dict[str, Any]]]:
    """Globally pair bags and reject equal or near-equal alternate optima."""
    candidates: dict[tuple[int, int], PairCandidate] = {}
    for human_index, human in enumerate(humans):
        for detector_index, detector in enumerate(detectors):
            candidate = pair_candidate(
                human_index,
                detector_index,
                human,
                detector,
                max_gap_ns,
            )
            if candidate is not None:
                candidates[(human_index, detector_index)] = candidate

    edge_costs = {key: value.cost for key, value in candidates.items()}
    matches, best_cost = minimum_cost_maximum_matching(
        len(humans), len(detectors), edge_costs
    )
    ambiguous_humans: set[int] = set()
    ambiguous_detectors: set[int] = set()
    issues: list[dict[str, Any]] = []

    # Removing each chosen edge detects alternate global assignments, including
    # swaps that a local "top two" check would miss.
    for human_index, detector_index in sorted(matches.items()):
        alternate, alternate_cost = minimum_cost_maximum_matching(
            len(humans),
            len(detectors),
            edge_costs,
            excluded_edges={(human_index, detector_index)},
        )
        if len(alternate) != len(matches):
            continue
        if alternate_cost - best_cost > ambiguity_margin_ns:
            continue
        changed_humans = {
            index
            for index in set(matches) | set(alternate)
            if matches.get(index) != alternate.get(index)
        }
        changed_detectors = {
            value for index, value in matches.items() if index in changed_humans
        } | {
            value for index, value in alternate.items() if index in changed_humans
        }
        ambiguous_humans.update(changed_humans)
        ambiguous_detectors.update(changed_detectors)

    if ambiguous_humans:
        issues.append(
            {
                "type": "ambiguous_pairing",
                "message": "Near-equivalent global bag assignments were found.",
                "human_bags": [str(humans[index].path) for index in sorted(ambiguous_humans)],
                "detector_bags": [
                    str(detectors[index].path) for index in sorted(ambiguous_detectors)
                ],
            }
        )
        remaining_edges = {
            key: cost
            for key, cost in edge_costs.items()
            if key[0] not in ambiguous_humans and key[1] not in ambiguous_detectors
        }
        matches, _ = minimum_cost_maximum_matching(
            len(humans), len(detectors), remaining_edges
        )

    selected = []
    used_detectors: set[int] = set()
    for human_index, detector_index in sorted(matches.items()):
        selected.append(
            (
                humans[human_index],
                detectors[detector_index],
                candidates[(human_index, detector_index)],
            )
        )
        used_detectors.add(detector_index)

    for index, human in enumerate(humans):
        if index not in matches and index not in ambiguous_humans:
            issues.append(
                {
                    "type": "unpaired_human_bag",
                    "bag": str(human.path),
                    "message": "No eligible, unambiguous detector counterpart was found.",
                }
            )
    for index, detector in enumerate(detectors):
        if index not in used_detectors and index not in ambiguous_detectors:
            issues.append(
                {
                    "type": "unpaired_detector_bag",
                    "bag": str(detector.path),
                    "message": "No eligible, unambiguous human-labeled counterpart was found.",
                }
            )
    return selected, issues


def choose_event_timestamp(
    label_time_ns: int, result: DetectorResult
) -> TimestampEvidence:
    """Choose the candidate nearest the human label with deterministic ties."""
    return min(result.event_times, key=lambda value: (abs(value.ns - label_time_ns), value.ns))


def match_events(
    labels: Sequence[HumanLabel],
    results: Sequence[DetectorResult],
    buffer_ns: int,
) -> tuple[dict[int, tuple[int, TimestampEvidence]], list[int], list[int]]:
    """Maximum-cardinality, minimum-time-distance event matching."""
    positives = [index for index, result in enumerate(results) if result.anomaly is True]
    edge_costs: dict[tuple[int, int], int] = {}
    selected_times: dict[tuple[int, int], TimestampEvidence] = {}
    for label_index, label in enumerate(labels):
        for positive_index, result_index in enumerate(positives):
            selected = choose_event_timestamp(label.event_time.ns, results[result_index])
            distance = abs(selected.ns - label.event_time.ns)
            if distance <= buffer_ns:  # The five-second boundary is inclusive.
                edge_costs[(label_index, positive_index)] = distance
                selected_times[(label_index, positive_index)] = selected

    compact_matches, _ = minimum_cost_maximum_matching(
        len(labels), len(positives), edge_costs
    )
    matches = {
        label_index: (
            positives[positive_index],
            selected_times[(label_index, positive_index)],
        )
        for label_index, positive_index in compact_matches.items()
    }
    matched_results = {result_index for result_index, _ in matches.values()}
    unmatched_labels = [index for index in range(len(labels)) if index not in matches]
    unmatched_results = [index for index in positives if index not in matched_results]
    return matches, unmatched_labels, unmatched_results


def safe_ratio(numerator: int, denominator: int) -> float | None:
    """Return a JSON-safe ratio or None when it is undefined."""
    return numerator / denominator if denominator else None


def latency_statistics(values: Sequence[float]) -> dict[str, float | int | None]:
    """Summarize latency values without emitting NaN or infinity."""
    if not values:
        return {
            "count": 0,
            "mean_seconds": None,
            "median_seconds": None,
            "minimum_seconds": None,
            "maximum_seconds": None,
            "p95_seconds": None,
        }
    ordered = sorted(values)
    rank = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "count": len(ordered),
        "mean_seconds": statistics.fmean(ordered),
        "median_seconds": statistics.median(ordered),
        "minimum_seconds": ordered[0],
        "maximum_seconds": ordered[-1],
        "p95_seconds": ordered[rank] if len(ordered) >= 2 else None,
    }


def compute_metrics(
    true_positives: int,
    false_negatives: int,
    false_positives: int,
) -> dict[str, Any]:
    """Calculate event metrics using micro-aggregated counts."""
    accuracy = safe_ratio(true_positives, true_positives + false_negatives)
    precision = safe_ratio(true_positives, true_positives + false_positives)
    recall = accuracy
    f1 = safe_ratio(
        2 * true_positives,
        2 * true_positives + false_positives + false_negatives,
    )
    return {
        "true_positives": true_positives,
        "false_negatives": false_negatives,
        "false_positives": false_positives,
        "overall_detection_accuracy": accuracy,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "accuracy_definition": (
            "true_positives / (true_positives + false_negatives); "
            "equivalent to event recall"
        ),
    }


def evaluate_pair(
    human: BagProfile,
    detector: BagProfile,
    candidate: PairCandidate,
    buffer_ns: int,
) -> dict[str, Any]:
    """Evaluate one bag pair and retain full timestamp provenance."""
    labels = sorted(human.labels, key=lambda item: item.event_time.ns)
    results = sorted(detector.detector_results, key=lambda item: item.return_time.ns)
    matches, unmatched_labels, unmatched_results = match_events(labels, results, buffer_ns)

    event_rows: list[dict[str, Any]] = []
    processing_latencies: list[float] = []
    label_latencies: list[float] = []
    warnings = [*human.warnings, *detector.warnings]

    for label_index, label in enumerate(labels):
        window_start = label.event_time.ns - buffer_ns
        window_end = label.event_time.ns + buffer_ns
        if label_index not in matches:
            event_rows.append(
                {
                    "result": "false_negative",
                    "human_label_id": label.identifier,
                    "human_label_timestamp_ns": label.event_time.ns,
                    "human_timestamp_source": label.event_time.source,
                    "window_start_ns": window_start,
                    "window_end_ns": window_end,
                    "matched": False,
                    "detector_result_id": None,
                    "detector_event_timestamp_ns": None,
                    "detector_return_timestamp_ns": None,
                    "event_time_difference_seconds": None,
                    "processing_latency_seconds": None,
                    "label_relative_latency_seconds": None,
                    "raw_label": label.raw,
                    "raw_detector_context": None,
                    "raw_detector_response": None,
                }
            )
            continue

        result_index, selected_time = matches[label_index]
        result = results[result_index]
        event_difference = (selected_time.ns - label.event_time.ns) / NS_PER_SECOND
        processing_latency = (result.return_time.ns - selected_time.ns) / NS_PER_SECOND
        label_latency = (result.return_time.ns - label.event_time.ns) / NS_PER_SECOND
        processing_latencies.append(processing_latency)
        label_latencies.append(label_latency)
        if processing_latency < 0:
            warnings.append(
                f"Negative processing latency for {label.identifier} and "
                f"{result.identifier}: {processing_latency:.9f}s"
            )
        event_rows.append(
            {
                "result": "true_positive",
                "human_label_id": label.identifier,
                "human_label_timestamp_ns": label.event_time.ns,
                "human_timestamp_source": label.event_time.source,
                "window_start_ns": window_start,
                "window_end_ns": window_end,
                "matched": True,
                "detector_result_id": result.identifier,
                "detector_event_timestamp_ns": selected_time.ns,
                "detector_event_timestamp_source": selected_time.source,
                "detector_return_timestamp_ns": result.return_time.ns,
                "detector_return_timestamp_source": result.return_time.source,
                "event_time_difference_seconds": event_difference,
                "processing_latency_seconds": processing_latency,
                "label_relative_latency_seconds": label_latency,
                "attribution_method": (
                    "closest_to_human_label_from_" + result.attribution_method
                ),
                "all_detector_event_candidates_ns": [
                    item.ns for item in result.event_times
                ],
                "raw_label": label.raw,
                "raw_detector_context": result.raw_context,
                "raw_detector_response": result.raw_response,
            }
        )

    duplicate_count = 0
    for result_index in unmatched_results:
        result = results[result_index]
        nearest_label: HumanLabel | None = None
        nearest_time: TimestampEvidence | None = None
        nearest_distance: int | None = None
        for label in labels:
            selected = choose_event_timestamp(label.event_time.ns, result)
            distance = abs(selected.ns - label.event_time.ns)
            if distance <= buffer_ns and (
                nearest_distance is None or distance < nearest_distance
            ):
                nearest_label = label
                nearest_time = selected
                nearest_distance = distance
        subtype = "duplicate" if nearest_label is not None else "unmatched_positive"
        if subtype == "duplicate":
            duplicate_count += 1
        event_rows.append(
            {
                "result": "false_positive",
                "false_positive_subtype": subtype,
                "human_label_id": nearest_label.identifier if nearest_label else None,
                "human_label_timestamp_ns": (
                    nearest_label.event_time.ns if nearest_label else None
                ),
                "matched": False,
                "detector_result_id": result.identifier,
                "detector_event_timestamp_ns": nearest_time.ns if nearest_time else None,
                "detector_return_timestamp_ns": result.return_time.ns,
                "detector_return_timestamp_source": result.return_time.source,
                "raw_label": nearest_label.raw if nearest_label else None,
                "raw_detector_context": result.raw_context,
                "raw_detector_response": result.raw_response,
            }
        )

    true_positives = len(matches)
    false_negatives = len(unmatched_labels)
    # Duplicates are a subtype of false positive so repeated alerts reduce
    # precision instead of being rewarded as cost-free behavior.
    false_positives = len(unmatched_results)
    pair_metrics = compute_metrics(true_positives, false_negatives, false_positives)
    pair_metrics.update(
        {
            "human_label_count": len(labels),
            "positive_detector_result_count": sum(
                result.anomaly is True for result in results
            ),
            "negative_detector_result_count": sum(
                result.anomaly is False for result in results
            ),
            "unparseable_detector_result_count": sum(
                result.anomaly is None for result in results
            ),
            "duplicate_count": duplicate_count,
            "clean_pair": not labels
            and not any(result.anomaly is True for result in results),
        }
    )

    return {
        "human_bag": str(human.path),
        "detector_bag": str(detector.path),
        "pairing": {
            "time_domain": candidate.time_domain,
            "human_interval_ns": [candidate.human_start_ns, candidate.human_end_ns],
            "detector_interval_ns": [
                candidate.detector_start_ns,
                candidate.detector_end_ns,
            ],
            "overlap_seconds": candidate.overlap_ns / NS_PER_SECOND,
            "gap_seconds": candidate.gap_ns / NS_PER_SECOND,
            "assignment_cost": candidate.cost,
        },
        "metrics": pair_metrics,
        "processing_latency": latency_statistics(processing_latencies),
        "label_relative_latency": latency_statistics(label_latencies),
        "events": event_rows,
        "warnings": sorted(set(warnings)),
        "errors": [*human.errors, *detector.errors],
    }


def profile_summary(profile: BagProfile) -> dict[str, Any]:
    """Serialize non-event bag diagnostics for the final report."""
    source_interval = profile.source_interval()
    record_interval = profile.record_interval()
    return {
        "path": str(profile.path),
        "role": profile.role,
        "topics": profile.topic_types,
        "label_topics_present": profile.label_topics_present,
        "detector_topics_present": profile.detector_topics_present,
        "human_label_count": len(profile.labels),
        "detector_result_count": len(profile.detector_results),
        "source_interval_ns": list(source_interval) if source_interval else None,
        "record_interval_ns": list(record_interval) if record_interval else None,
        "warnings": profile.warnings,
        "errors": profile.errors,
    }


def aggregate_report(
    profiles: Sequence[BagProfile],
    pair_reports: Sequence[dict[str, Any]],
    pairing_issues: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Micro-aggregate pair counts and latency samples for the folder."""
    true_positives = sum(pair["metrics"]["true_positives"] for pair in pair_reports)
    false_negatives = sum(pair["metrics"]["false_negatives"] for pair in pair_reports)
    false_positives = sum(pair["metrics"]["false_positives"] for pair in pair_reports)
    metrics = compute_metrics(true_positives, false_negatives, false_positives)
    metrics.update(
        {
            "bags_discovered": len(profiles),
            "human_labeled_bags": sum(
                profile.role == "human_labeled" for profile in profiles
            ),
            "detector_output_bags": sum(
                profile.role == "detector_output" for profile in profiles
            ),
            "unknown_bags": sum(profile.role == "unknown" for profile in profiles),
            "bag_pairs_evaluated": len(pair_reports),
            "human_label_count": sum(
                pair["metrics"]["human_label_count"] for pair in pair_reports
            ),
            "positive_detector_result_count": sum(
                pair["metrics"]["positive_detector_result_count"]
                for pair in pair_reports
            ),
            "duplicate_count": sum(
                pair["metrics"]["duplicate_count"] for pair in pair_reports
            ),
            "evaluation_complete": not pairing_issues
            and all(not profile.errors for profile in profiles)
            and all(profile.role != "unknown" for profile in profiles),
        }
    )

    processing_values = [
        event["processing_latency_seconds"]
        for pair in pair_reports
        for event in pair["events"]
        if event["result"] == "true_positive"
    ]
    label_values = [
        event["label_relative_latency_seconds"]
        for pair in pair_reports
        for event in pair["events"]
        if event["result"] == "true_positive"
    ]
    return {
        "metrics": metrics,
        "processing_latency": latency_statistics(processing_values),
        "label_relative_latency": latency_statistics(label_values),
    }


def collision_safe_report_path(output_directory: Path, started_at: datetime) -> Path:
    """Choose a timestamped output path without overwriting prior results."""
    stem = f"anomaly_evaluation_{started_at:%Y%m%d_%H%M%S}"
    candidate = output_directory / f"{stem}.json"
    suffix = 1
    while candidate.exists():
        candidate = output_directory / f"{stem}_{suffix}.json"
        suffix += 1
    return candidate


def percentage(value: float | None) -> str:
    """Format a report ratio for terminal output."""
    return "N/A" if value is None else f"{100.0 * value:.1f}%"


def print_summary(report: dict[str, Any], report_path: Path) -> None:
    """Print a compact human-readable summary after writing the report."""
    overall = report["overall"]["metrics"]
    latency = report["overall"]["processing_latency"]
    print("\nEvaluation summary")
    print(f"  Bag pairs evaluated:  {overall['bag_pairs_evaluated']}")
    print(f"  Human anomalies:      {overall['human_label_count']}")
    print(f"  Correctly detected:   {overall['true_positives']}")
    print(f"  Missed:               {overall['false_negatives']}")
    print(f"  False positives:      {overall['false_positives']}")
    print(f"  Overall accuracy:     {percentage(overall['overall_detection_accuracy'])}")
    print(f"  Detection precision:  {percentage(overall['precision'])}")
    median = latency["median_seconds"]
    print(f"  Median processing:    {'N/A' if median is None else f'{median:.3f}s'}")
    print(f"  Evaluation complete:  {overall['evaluation_complete']}")
    print(f"  Report:               {report_path}")


def compile_pattern(value: str, option_name: str) -> re.Pattern[str]:
    """Compile a CLI regular expression with an actionable error."""
    try:
        return re.compile(value, re.IGNORECASE)
    except re.error as exc:
        raise EvaluationError(f"Invalid {option_name} regular expression: {exc}") from exc


def build_argument_parser() -> argparse.ArgumentParser:
    """Define the command-line contract in one place."""
    parser = argparse.ArgumentParser(
        description=(
            "Pair human-labeled and detector-output ROS 2 bags by timestamp, "
            "then evaluate boolean anomaly detections."
        )
    )
    parser.add_argument("--bag-dir", required=True, type=Path, help="Folder to scan recursively.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report folder (default: <bag-dir>/evaluation_results).",
    )
    parser.add_argument(
        "--buffer-seconds",
        type=float,
        default=5.0,
        help="Inclusive event matching buffer before/after each label (default: 5).",
    )
    parser.add_argument(
        "--human-label-topic",
        action="append",
        default=[],
        help="Exact human-label topic; repeat for multiple topics. Auto-detected if omitted.",
    )
    parser.add_argument(
        "--human-label-topic-regex",
        default=DEFAULT_LABEL_TOPIC_PATTERN,
        help="Conservative topic-name regex used only when no exact label topic is supplied.",
    )
    parser.add_argument(
        "--human-label-positive-regex",
        default=None,
        help=(
            "Optional regex that a label message must match. Otherwise explicit "
            "false is skipped and all other label-topic messages are positive."
        ),
    )
    parser.add_argument(
        "--detector-topic",
        action="append",
        default=[],
        help="Detector result/artifact topic; repeat to replace built-in defaults.",
    )
    parser.add_argument(
        "--context-topic",
        action="append",
        default=[],
        help="Formatted source-message topic; repeat to replace built-in defaults.",
    )
    parser.add_argument(
        "--context-lookback-seconds",
        type=float,
        default=30.0,
        help="Context associated with plain decisions lacking artifact data (default: 30).",
    )
    parser.add_argument(
        "--max-pair-gap-seconds",
        type=float,
        default=5.0,
        help="Maximum separation allowed between counterpart bag ranges (default: 5).",
    )
    parser.add_argument(
        "--pair-ambiguity-seconds",
        type=float,
        default=1.0,
        help="Maximum alternate-assignment cost difference treated as ambiguous (default: 1).",
    )
    return parser


def validate_nonnegative(name: str, value: float) -> None:
    """Reject NaN, infinity, and negative duration arguments."""
    if not math.isfinite(value) or value < 0:
        raise EvaluationError(f"{name} must be a finite non-negative number.")


def run(args: argparse.Namespace) -> Path:
    """Execute folder discovery, pairing, evaluation, and report writing."""
    validate_nonnegative("--buffer-seconds", args.buffer_seconds)
    validate_nonnegative("--context-lookback-seconds", args.context_lookback_seconds)
    validate_nonnegative("--max-pair-gap-seconds", args.max_pair_gap_seconds)
    validate_nonnegative("--pair-ambiguity-seconds", args.pair_ambiguity_seconds)

    started_at = datetime.now(timezone.utc)
    root = args.bag_dir.resolve()
    output_directory = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else root / "evaluation_results"
    )
    bags = discover_bags(root)
    if not bags:
        raise EvaluationError(f"No ROS 2 bags were found beneath {root}")

    label_pattern = compile_pattern(
        args.human_label_topic_regex, "--human-label-topic-regex"
    )
    positive_pattern = (
        compile_pattern(args.human_label_positive_regex, "--human-label-positive-regex")
        if args.human_label_positive_regex
        else None
    )
    label_topics = normalize_topics(args.human_label_topic)
    detector_topics = normalize_topics(args.detector_topic or DEFAULT_DETECTOR_TOPICS)
    context_topics = normalize_topics(args.context_topic or DEFAULT_CONTEXT_TOPICS)

    print(f"Discovered {len(bags)} logical bag(s) beneath {root}")
    profiles: list[BagProfile] = []
    for index, bag in enumerate(bags, start=1):
        print(f"[{index}/{len(bags)}] Scanning {bag}")
        profiles.append(
            scan_bag(
                bag,
                configured_label_topics=label_topics,
                label_topic_pattern=label_pattern,
                label_positive_pattern=positive_pattern,
                detector_topics=detector_topics,
                context_topics=context_topics,
                context_lookback_ns=seconds_to_ns(args.context_lookback_seconds),
            )
        )

    humans = [profile for profile in profiles if profile.role == "human_labeled"]
    detectors = [profile for profile in profiles if profile.role == "detector_output"]
    selected_pairs, pairing_issues = select_bag_pairs(
        humans,
        detectors,
        max_gap_ns=seconds_to_ns(args.max_pair_gap_seconds),
        ambiguity_margin_ns=seconds_to_ns(args.pair_ambiguity_seconds),
    )
    pair_reports = []
    for human, detector, candidate in selected_pairs:
        print(
            f"PAIR {human.path.name} <-> {detector.path.name} "
            f"overlap={candidate.overlap_ns / NS_PER_SECOND:.3f}s "
            f"domain={candidate.time_domain}"
        )
        pair_reports.append(
            evaluate_pair(
                human,
                detector,
                candidate,
                buffer_ns=seconds_to_ns(args.buffer_seconds),
            )
        )
    for issue in pairing_issues:
        print(f"PAIRING {issue['type']}: {issue.get('bag', issue.get('message', ''))}")

    overall = aggregate_report(profiles, pair_reports, pairing_issues)
    report = {
        "schema_version": REPORT_SCHEMA_VERSION,
        "generated_at_utc": started_at.isoformat(),
        "input_folder": str(root),
        "configuration": {
            "buffer_seconds": args.buffer_seconds,
            "human_label_topics": sorted(label_topics),
            "human_label_topic_regex": args.human_label_topic_regex,
            "human_label_positive_regex": args.human_label_positive_regex,
            "detector_topics": sorted(detector_topics),
            "context_topics": sorted(context_topics),
            "context_lookback_seconds": args.context_lookback_seconds,
            "max_pair_gap_seconds": args.max_pair_gap_seconds,
            "pair_ambiguity_seconds": args.pair_ambiguity_seconds,
        },
        "overall": overall,
        "bag_pairs": pair_reports,
        "pairing_issues": pairing_issues,
        "bags": [profile_summary(profile) for profile in profiles],
    }

    output_directory.mkdir(parents=True, exist_ok=True)
    report_path = collision_safe_report_path(output_directory, started_at)
    try:
        with report_path.open("x", encoding="utf-8") as report_file:
            json.dump(report, report_file, indent=2, sort_keys=False, allow_nan=False)
            report_file.write("\n")
    except OSError as exc:
        raise EvaluationError(f"Could not write report {report_path}: {exc}") from exc

    print_summary(report, report_path)
    return report_path


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point with concise expected-error handling."""
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        run(args)
    except EvaluationError as exc:
        parser.exit(2, f"error: {exc}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
