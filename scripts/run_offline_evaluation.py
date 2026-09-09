#!/usr/bin/env python3
"""
Batch-replay ROS 2 bags through configurable anomaly-detection runs.

The runner supports two workflows:

* Pass paths and mode on the command line.
* Edit QUICK RUN SETTINGS below and click Run in an IDE.

Experiment options live in YAML. For every bag/configuration/trial combination,
the script creates an isolated resolved AAD configuration, starts a clean node,
replays the bag, collects decisions, and writes a detailed Markdown report.
"""

from __future__ import annotations

import argparse
from collections import deque
import copy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Iterable, Sequence
from urllib.parse import unquote, urlparse

import yaml

try:
    import rclpy
    import rosbag2_py
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Bool, String
except ImportError as exc:
    ROS_IMPORT_ERROR: ImportError | None = exc
else:
    ROS_IMPORT_ERROR = None


# ---------------------------------------------------------------------------
# QUICK RUN SETTINGS
# Paste local paths (or file:// links) here, then run this file from an IDE.
# Command-line values override YAML runner values, which override these values.
# Experiment/model/image/prompt options belong in the YAML file, not here.
# ---------------------------------------------------------------------------
QUICK_RUN_BAG_LOCATION = '/root/dev_ws/src/anomaly_detection/system_evaluation/bags'
QUICK_RUN_CONFIG = (
    '/root/dev_ws/src/anomaly_detection/system_evaluation/offline_evaluation.yaml'
)
QUICK_RUN_MODE = 'unlabeled'  # 'unlabeled' or 'human-labeled'
QUICK_RUN_OUTPUT_DIRECTORY = (
    '/root/dev_ws/src/anomaly_detection/system_evaluation/results'
)


NS_PER_SECOND = 1_000_000_000
SCHEMA_VERSION = 5
DEFAULT_DECISION_TOPIC = '/aad/decisions'
DEFAULT_ALERT_TOPIC = '/aad/alerts'
DEFAULT_LLM_CALLED_TOPIC = '/aad/llm_called'
DEFAULT_FORMATTED_TOPIC = '/aad/formatted_messages'

TIMESTAMP_PATTERN = re.compile(
    r'\[\s*t\s*=\s*(?P<seconds>-?\d+)'
    r'(?:\.(?P<fraction>\d{1,9}))?(?=\s|\])',
    re.IGNORECASE,
)
ANOMALY_PATTERN = re.compile(
    r'\banomaly\s*[:=]\s*(?P<value>true|false|yes|no|1|0)\b',
    re.IGNORECASE,
)
FIELD_PATTERN = re.compile(
    r'\b(?P<key>severity|action)\s*=\s*(?P<value>[^\s]+)',
    re.IGNORECASE,
)
SUMMARY_PATTERN = re.compile(
    r'\bsummary\s*=\s*(?P<value>.*)$',
    re.IGNORECASE | re.DOTALL,
)


class EvaluationError(RuntimeError):
    """Expected configuration, environment, or execution failure."""


class EvaluationInterrupted(RuntimeError):
    """A user interruption that occurred after a partial report was saved."""

    def __init__(self, report_path: Path) -> None:
        """Record the path containing results completed before interruption."""
        super().__init__(f'Partial report saved to {report_path}')
        self.report_path = report_path


@dataclass(frozen=True)
class TimestampEvidence:
    """Integer timestamp with auditable provenance."""

    timestamp_ns: int
    source: str


@dataclass(frozen=True)
class BagInfo:
    """One logical ROS 2 bag and its static metadata."""

    path: Path
    topics: dict[str, str]
    start_ns: int | None
    end_ns: int | None
    duration_seconds: float | None


@dataclass(frozen=True)
class Experiment:
    """One resolved anomaly-detector configuration."""

    name: str
    aad_config: dict[str, Any]
    config_hash: str
    trials: int


@dataclass(frozen=True)
class RunnerSettings:
    """Validated orchestration settings loaded from YAML and CLI."""

    mode: str
    bags_path: Path
    output_directory: Path
    playback_rate: float
    startup_timeout_seconds: float
    startup_grace_seconds: float
    playback_timeout_padding_seconds: float
    inference_drain_timeout_seconds: float
    post_playback_grace_seconds: float
    shutdown_timeout_seconds: float
    context_lookback_seconds: float
    label_buffer_seconds: float
    comparison_window_seconds: float
    continue_on_error: bool
    detector_command: tuple[str, ...]
    decision_topic: str
    alert_topic: str
    llm_called_topic: str
    formatted_topic: str
    human_label_topic: str | None
    human_label_positive_regex: str | None
    dry_run: bool


@dataclass
class FlowEdge:
    """Residual edge for dependency-free bipartite matching."""

    target: int
    reverse: int
    capacity: int
    cost: int


@dataclass
class CollectedContext:
    """A formatted input message observed during replay."""

    received_monotonic_ns: int
    source_timestamps_ns: list[int]
    raw: str


@dataclass
class CollectorState:
    """Mutable callback state protected by a condition lock."""

    decisions: list[dict[str, Any]] = field(default_factory=list)
    alerts: list[dict[str, Any]] = field(default_factory=list)
    llm_calls: list[dict[str, Any]] = field(default_factory=list)
    pending_llm_calls_ns: deque[int] = field(default_factory=deque)
    contexts: deque[CollectedContext] = field(default_factory=deque)
    last_activity_ns: int = field(default_factory=time.monotonic_ns)


def decimal_timestamp_ns(seconds: str, fraction: str | None) -> int:
    """Convert decimal seconds exactly, without float precision loss."""
    whole = int(seconds)
    fractional = int((fraction or '').ljust(9, '0') or '0')
    return whole * NS_PER_SECOND - fractional if whole < 0 else whole * NS_PER_SECOND + fractional


def embedded_timestamps(text: str) -> list[int]:
    """Extract unique formatted-message timestamps in encounter order."""
    result: list[int] = []
    seen: set[int] = set()
    for match in TIMESTAMP_PATTERN.finditer(text):
        value = decimal_timestamp_ns(match.group('seconds'), match.group('fraction'))
        if value not in seen:
            result.append(value)
            seen.add(value)
    return result


def parse_json_object(text: str) -> dict[str, Any] | None:
    """Decode a JSON mapping, allowing one JSON-string wrapper."""
    try:
        value: Any = json.loads(text.strip())
        if isinstance(value, str):
            value = json.loads(value)
    except (json.JSONDecodeError, TypeError):
        return None
    return value if isinstance(value, dict) else None


def coerce_bool(value: Any) -> bool | None:
    """Normalize common boolean representations."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {'true', 'yes', '1', 'positive', 'anomaly'}:
            return True
        if normalized in {'false', 'no', '0', 'negative', 'normal'}:
            return False
    return None


def anomaly_value(value: Any) -> bool | None:
    """Find an anomaly boolean in mappings, JSON strings, or key-value text."""
    if isinstance(value, dict):
        for key in ('anomaly', 'is_anomaly', 'human_label', 'ground_truth', 'label'):
            if key in value:
                parsed = coerce_bool(value[key])
                if parsed is not None:
                    return parsed
        for key in ('decision', 'api_response', 'response', 'result'):
            if key in value:
                parsed = anomaly_value(value[key])
                if parsed is not None:
                    return parsed
        return None
    if isinstance(value, str):
        payload = parse_json_object(value)
        if payload is not None:
            parsed = anomaly_value(payload)
            if parsed is not None:
                return parsed
        match = ANOMALY_PATTERN.search(value)
        return coerce_bool(match.group('value')) if match else None
    return coerce_bool(value)


def parse_decision(text: str) -> dict[str, Any]:
    """Parse the stable decision fields while preserving the raw response."""
    payload = parse_json_object(text)
    anomaly = anomaly_value(payload if payload is not None else text)
    result: dict[str, Any] = {
        'raw': text,
        'anomaly': anomaly,
        'severity': None,
        'action': None,
        'summary': None,
    }
    if payload is not None:
        result['severity'] = payload.get('severity')
        result['action'] = payload.get('action', payload.get('response'))
        result['summary'] = payload.get('summary')
    for match in FIELD_PATTERN.finditer(text):
        result[match.group('key').lower()] = match.group('value')
    if result['summary'] is None:
        summary_match = SUMMARY_PATTERN.search(text)
        if summary_match:
            result['summary'] = summary_match.group('value').strip()
    return result


def normalize_location(value: str | Path, base_directory: Path | None = None) -> Path:
    """Resolve a local path or file:// link and reject implicit downloads."""
    raw = str(value).strip()
    if not raw:
        raise EvaluationError('A required path is empty.')
    parsed = urlparse(raw)
    if parsed.scheme and parsed.scheme != 'file':
        raise EvaluationError(
            f'Unsupported location scheme {parsed.scheme!r}; use a local path or file:// link.'
        )
    if parsed.scheme == 'file':
        if parsed.netloc not in ('', 'localhost'):
            raise EvaluationError(f'Remote file URL hosts are unsupported: {raw}')
        path = Path(unquote(parsed.path))
    else:
        path = Path(raw).expanduser()
    if not path.is_absolute() and base_directory is not None:
        path = base_directory / path
    return path.resolve()


def load_yaml_mapping(path: Path) -> dict[str, Any]:
    """Load a YAML mapping with context-rich validation errors."""
    try:
        with path.open(encoding='utf-8') as stream:
            value = yaml.safe_load(stream) or {}
    except OSError as exc:
        raise EvaluationError(f'Could not read YAML file {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise EvaluationError(f'Invalid YAML in {path}: {exc}') from exc
    if not isinstance(value, dict):
        raise EvaluationError(f'YAML root must be a mapping: {path}')
    return value


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge mappings while replacing scalar and list values."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def set_nested_value(target: dict[str, Any], dotted_key: str, value: Any) -> None:
    """Apply an optional dotted override such as ``llm.vision_enabled``."""
    parts = dotted_key.split('.')
    if not all(parts):
        raise EvaluationError(f'Invalid empty override path: {dotted_key!r}')
    current = target
    for part in parts[:-1]:
        existing = current.get(part)
        if existing is None:
            existing = {}
            current[part] = existing
        if not isinstance(existing, dict):
            raise EvaluationError(
                f'Override {dotted_key!r} crosses non-mapping key {part!r}.'
            )
        current = existing
    current[parts[-1]] = copy.deepcopy(value)


def normalize_overrides(value: Any, experiment_name: str) -> dict[str, Any]:
    """Accept ordinary nested YAML and convenient dotted keys."""
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise EvaluationError(f'Experiment {experiment_name!r} overrides must be a mapping.')
    result: dict[str, Any] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise EvaluationError(f'Experiment {experiment_name!r} has a non-string override key.')
        if '.' in key:
            set_nested_value(result, key, item)
        elif isinstance(item, dict):
            result[key] = normalize_overrides(item, experiment_name)
        else:
            result[key] = copy.deepcopy(item)
    return result


def stable_hash(value: dict[str, Any]) -> str:
    """Hash the fully resolved experiment configuration."""
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def positive_number(name: str, value: Any, allow_zero: bool = False) -> float:
    """Validate finite duration/rate settings."""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(f'{name} must be numeric, got {value!r}.') from exc
    minimum_ok = number >= 0 if allow_zero else number > 0
    if not math.isfinite(number) or not minimum_ok:
        qualifier = 'non-negative' if allow_zero else 'positive'
        raise EvaluationError(f'{name} must be a finite {qualifier} number.')
    return number


def positive_integer(name: str, value: Any) -> int:
    """Validate trial-count settings without accepting booleans."""
    if isinstance(value, bool):
        raise EvaluationError(f'{name} must be a positive integer.')
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(f'{name} must be a positive integer.') from exc
    if parsed <= 0 or parsed != value:
        raise EvaluationError(f'{name} must be a positive integer.')
    return parsed


def resolve_experiments(
    document: dict[str, Any],
    config_path: Path,
    trials_override: int | None = None,
) -> list[Experiment]:
    """Resolve base AAD YAML plus each experiment's overrides."""
    raw_experiments = document.get('experiments')
    if not isinstance(raw_experiments, list) or not raw_experiments:
        raise EvaluationError('Evaluation YAML must contain a non-empty experiments list.')
    base_reference = document.get('base_config')
    default_trials = document.get('runner', {}).get('trials', 1)
    names: set[str] = set()
    resolved: list[Experiment] = []

    for index, raw in enumerate(raw_experiments, start=1):
        if not isinstance(raw, dict):
            raise EvaluationError(f'Experiment #{index} must be a mapping.')
        name = str(raw.get('name', '')).strip()
        if not name:
            raise EvaluationError(f'Experiment #{index} has no name.')
        if name in names:
            raise EvaluationError(f'Duplicate experiment name: {name!r}')
        names.add(name)

        reference = raw.get('config', base_reference)
        if not reference:
            raise EvaluationError(
                f'Experiment {name!r} needs config or top-level base_config.'
            )
        base_path = normalize_location(str(reference), config_path.parent)
        base_config = load_yaml_mapping(base_path)
        overrides = normalize_overrides(raw.get('overrides'), name)
        aad_config = deep_merge(base_config, overrides)
        llm = aad_config.get('llm')
        if not isinstance(llm, dict):
            raise EvaluationError(f'Experiment {name!r} has no valid llm mapping.')
        if not isinstance(llm.get('model'), str) or not llm['model'].strip():
            raise EvaluationError(f'Experiment {name!r} must select llm.model.')
        if 'local' in llm and not isinstance(llm['local'], bool):
            raise EvaluationError(f'Experiment {name!r} llm.local must be boolean.')
        if 'vision_enabled' in llm and not isinstance(llm['vision_enabled'], bool):
            raise EvaluationError(
                f'Experiment {name!r} llm.vision_enabled must be boolean.'
            )
        trial_value = (
            trials_override
            if trials_override is not None
            else raw.get('trials', default_trials)
        )
        trials = positive_integer(f'experiments[{name}].trials', trial_value)
        resolved.append(Experiment(name, aad_config, stable_hash(aad_config), trials))
    return resolved


def runner_value(
    cli_value: Any,
    yaml_runner: dict[str, Any],
    yaml_key: str,
    quick_value: Any,
    fallback: Any,
) -> Any:
    """Apply CLI > YAML > quick-run > built-in precedence."""
    if cli_value is not None:
        return cli_value
    if yaml_key in yaml_runner:
        return yaml_runner[yaml_key]
    if quick_value not in (None, ''):
        return quick_value
    return fallback


def command_value(value: Any) -> tuple[str, ...]:
    """Normalize detector command from a YAML list or shell-like string."""
    if isinstance(value, str):
        result = tuple(shlex.split(value))
    elif isinstance(value, list) and all(isinstance(item, str) for item in value):
        result = tuple(value)
    else:
        raise EvaluationError('runner.detector_command must be a string or list of strings.')
    if not result:
        raise EvaluationError('runner.detector_command cannot be empty.')
    return result


def resolve_settings(
    args: argparse.Namespace,
    document: dict[str, Any],
    config_path: Path,
) -> RunnerSettings:
    """Resolve and validate orchestration values."""
    yaml_runner = document.get('runner', {})
    if not isinstance(yaml_runner, dict):
        raise EvaluationError('runner must be a YAML mapping.')

    if args.bags is not None:
        bags_path = normalize_location(args.bags)
    elif yaml_runner.get('bags'):
        bags_path = normalize_location(str(yaml_runner['bags']), config_path.parent)
    elif QUICK_RUN_BAG_LOCATION:
        bags_path = normalize_location(QUICK_RUN_BAG_LOCATION)
    else:
        raise EvaluationError(
            'No recordings folder set. Use --bags, runner.bags, or '
            'QUICK_RUN_BAG_LOCATION.'
        )
    if args.output_dir is not None:
        output_directory = normalize_location(args.output_dir)
    elif yaml_runner.get('output_directory'):
        output_directory = normalize_location(
            str(yaml_runner['output_directory']), config_path.parent
        )
    elif QUICK_RUN_OUTPUT_DIRECTORY:
        output_directory = normalize_location(QUICK_RUN_OUTPUT_DIRECTORY)
    else:
        output_directory = (config_path.parent / 'evaluation_results').resolve()
    mode = str(
        runner_value(args.mode, yaml_runner, 'mode', QUICK_RUN_MODE, 'unlabeled')
    ).strip()
    if mode not in {'unlabeled', 'human-labeled'}:
        raise EvaluationError('mode must be unlabeled or human-labeled.')

    label_topic_value = yaml_runner.get('human_label_topic')
    label_topic = str(label_topic_value).strip() if label_topic_value else None
    if mode == 'human-labeled' and not label_topic:
        raise EvaluationError(
            'human-labeled mode requires runner.human_label_topic in YAML.'
        )

    detector_command = command_value(
        yaml_runner.get(
            'detector_command',
            ['ros2', 'run', 'anomaly_detection', 'anomaly_detection_node'],
        )
    )
    return RunnerSettings(
        mode=mode,
        bags_path=bags_path,
        output_directory=output_directory,
        playback_rate=positive_number(
            'runner.playback_rate', yaml_runner.get('playback_rate', 1.0)
        ),
        startup_timeout_seconds=positive_number(
            'runner.startup_timeout_seconds',
            yaml_runner.get('startup_timeout_seconds', 15.0),
        ),
        startup_grace_seconds=positive_number(
            'runner.startup_grace_seconds',
            yaml_runner.get('startup_grace_seconds', 3.0),
            allow_zero=True,
        ),
        playback_timeout_padding_seconds=positive_number(
            'runner.playback_timeout_padding_seconds',
            yaml_runner.get('playback_timeout_padding_seconds', 30.0),
            allow_zero=True,
        ),
        inference_drain_timeout_seconds=positive_number(
            'runner.inference_drain_timeout_seconds',
            yaml_runner.get('inference_drain_timeout_seconds', 60.0),
        ),
        post_playback_grace_seconds=positive_number(
            'runner.post_playback_grace_seconds',
            yaml_runner.get('post_playback_grace_seconds', 2.0),
            allow_zero=True,
        ),
        shutdown_timeout_seconds=positive_number(
            'runner.shutdown_timeout_seconds',
            yaml_runner.get('shutdown_timeout_seconds', 20.0),
        ),
        context_lookback_seconds=positive_number(
            'runner.context_lookback_seconds',
            yaml_runner.get('context_lookback_seconds', 30.0),
            allow_zero=True,
        ),
        label_buffer_seconds=positive_number(
            'runner.label_buffer_seconds',
            yaml_runner.get('label_buffer_seconds', 5.0),
            allow_zero=True,
        ),
        comparison_window_seconds=positive_number(
            'runner.comparison_window_seconds',
            yaml_runner.get('comparison_window_seconds', 5.0),
            allow_zero=True,
        ),
        continue_on_error=bool(yaml_runner.get('continue_on_error', True)),
        detector_command=detector_command,
        decision_topic=str(yaml_runner.get('decision_topic', DEFAULT_DECISION_TOPIC)),
        alert_topic=str(yaml_runner.get('alert_topic', DEFAULT_ALERT_TOPIC)),
        llm_called_topic=str(
            yaml_runner.get('llm_called_topic', DEFAULT_LLM_CALLED_TOPIC)
        ),
        formatted_topic=str(
            yaml_runner.get('formatted_topic', DEFAULT_FORMATTED_TOPIC)
        ),
        human_label_topic=label_topic,
        human_label_positive_regex=(
            str(yaml_runner['human_label_positive_regex'])
            if yaml_runner.get('human_label_positive_regex')
            else None
        ),
        dry_run=bool(args.dry_run),
    )


def discover_bags(root: Path) -> list[Path]:
    """Find one selected bag or all bags nested beneath a directory."""
    if not root.is_dir():
        raise EvaluationError(f'Recordings folder does not exist: {root}')
    # A directory containing metadata.yaml is itself a ROS 2 bag. Supporting
    # this form lets a YAML configuration select one recording without moving it.
    if (root / 'metadata.yaml').is_file():
        return [root.resolve()]
    bag_directories = sorted(
        {metadata.parent.resolve() for metadata in root.rglob('metadata.yaml')}
    )
    standalone: set[Path] = set()
    for mcap in root.rglob('*.mcap'):
        resolved = mcap.resolve()
        if any(directory in resolved.parents for directory in bag_directories):
            continue
        standalone.add(resolved)
    bags = sorted([*bag_directories, *standalone], key=lambda item: str(item))
    if not bags:
        raise EvaluationError(f'No ROS 2 bags found beneath {root}')
    return bags


def inspect_bag(path: Path) -> BagInfo:
    """Read bag metadata and topic names without replaying it."""
    if ROS_IMPORT_ERROR is not None:
        raise EvaluationError(
            'ROS 2 Python modules are unavailable. Source the ROS and workspace '
            f'environments first: {ROS_IMPORT_ERROR}'
        )
    try:
        metadata = rosbag2_py.Info().read_metadata(str(path), 'mcap')
        topics = {
            item.topic_metadata.name: item.topic_metadata.type
            for item in metadata.topics_with_message_count
        }
        start = int(metadata.starting_time.nanoseconds)
        duration_ns = int(metadata.duration.nanoseconds)
        return BagInfo(
            path=path,
            topics=topics,
            start_ns=start,
            end_ns=start + duration_ns,
            duration_seconds=duration_ns / NS_PER_SECOND,
        )
    except Exception:
        # Standalone or incomplete MCAP files may not have sidecar metadata.
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr',
                ),
            )
            topics = {
                item.name: item.type for item in reader.get_all_topics_and_types()
            }
            first: int | None = None
            last: int | None = None
            while reader.has_next():
                _, _, timestamp = reader.read_next()
                first = int(timestamp) if first is None else min(first, int(timestamp))
                last = int(timestamp) if last is None else max(last, int(timestamp))
        except Exception as exc:
            raise EvaluationError(f'Could not inspect bag {path}: {exc}') from exc
        return BagInfo(
            path=path,
            topics=topics,
            start_ns=first,
            end_ns=last,
            duration_seconds=(
                (last - first) / NS_PER_SECOND
                if first is not None and last is not None
                else None
            ),
        )


def scalar_message_text(message: Any) -> str:
    """Extract common textual ROS fields without expanding images or bytes."""
    for name in ('data', 'msg', 'message', 'payload', 'label'):
        value = getattr(message, name, None)
        if isinstance(value, str) and value.strip():
            return value
    return ''


def message_header_time(message: Any) -> int | None:
    """Read a nonzero ROS Header timestamp."""
    try:
        value = int(message.header.stamp.sec) * NS_PER_SECOND
        value += int(message.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    return value or None


def mapping_timestamp_ns(payload: dict[str, Any]) -> int | None:
    """Read common label timestamp keys from JSON."""
    for key in (
        'label_timestamp_ns',
        'event_timestamp_ns',
        'source_timestamp_ns',
        'timestamp_ns',
    ):
        value = payload.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            return value
        if isinstance(value, str) and value.strip().lstrip('-').isdigit():
            return int(value)
    for key in ('label_timestamp', 'event_timestamp', 'source_timestamp', 'timestamp'):
        value = payload.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return int(round(float(value) * NS_PER_SECOND))
        if isinstance(value, str):
            match = re.fullmatch(r'(-?\d+)(?:\.(\d{1,9}))?', value.strip())
            if match:
                return decimal_timestamp_ns(match.group(1), match.group(2))
    return None


def extract_human_labels(
    bag: BagInfo,
    topic: str,
    positive_regex: str | None,
) -> tuple[list[dict[str, Any]], list[str]]:
    """Extract timestamped positive truth markers before replay."""
    if topic not in bag.topics:
        return [], [f'Required human-label topic {topic!r} is missing.']
    try:
        message_class = get_message(bag.topics[topic])
    except Exception as exc:
        return [], [f'Cannot load label type {bag.topics[topic]!r}: {exc}']
    try:
        pattern = re.compile(positive_regex, re.IGNORECASE) if positive_regex else None
    except re.error as exc:
        raise EvaluationError(f'Invalid human_label_positive_regex: {exc}') from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.path), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    labels: list[dict[str, Any]] = []
    warnings: list[str] = []
    index = 0
    while reader.has_next():
        _, serialized, record_ns = reader.read_next()
        index += 1
        try:
            message = deserialize_message(serialized, message_class)
        except Exception as exc:
            warnings.append(f'Could not decode label message #{index}: {exc}')
            continue
        text = scalar_message_text(message)
        payload = parse_json_object(text)
        explicit = anomaly_value(payload if payload is not None else text)
        if pattern is not None and pattern.search(text) is None:
            continue
        if explicit is False:
            continue
        # A dedicated label topic treats messages without an explicit boolean
        # as positive markers; a regex may narrow that behavior when required.
        timestamp = mapping_timestamp_ns(payload) if payload is not None else None
        source = 'label_json'
        if timestamp is None:
            embedded = embedded_timestamps(text)
            timestamp = embedded[0] if embedded else None
            source = 'label_message[t]'
        if timestamp is None:
            timestamp = message_header_time(message)
            source = 'label_header'
        if timestamp is None:
            timestamp = int(record_ns)
            source = 'label_bag_record_fallback'
            warnings.append(f'Label #{index} uses its bag-record timestamp.')
        labels.append(
            {
                'id': f'{bag.path.name}:label:{index}',
                'timestamp_ns': timestamp,
                'timestamp_source': source,
                'record_timestamp_ns': int(record_ns),
                'raw': text,
            }
        )
    return labels, warnings


class EvaluationCollector(Node):
    """Collect detector topics and correlate calls, context, and decisions."""

    def __init__(self, settings: RunnerSettings) -> None:
        super().__init__('offline_evaluation_collector')
        self.settings = settings
        self.state = CollectorState()
        self.condition = threading.Condition()
        self.create_subscription(String, settings.decision_topic, self._decision, 50)
        self.create_subscription(String, settings.alert_topic, self._alert, 50)
        self.create_subscription(Bool, settings.llm_called_topic, self._llm_called, 50)
        self.create_subscription(String, settings.formatted_topic, self._formatted, 100)

    def reset(self) -> None:
        """Clear all run-specific state."""
        with self.condition:
            self.state = CollectorState()
            self.condition.notify_all()

    def _touch(self) -> int:
        now = time.monotonic_ns()
        self.state.last_activity_ns = now
        return now

    def _llm_called(self, message: Bool) -> None:
        with self.condition:
            now = self._touch()
            self.state.pending_llm_calls_ns.append(now)
            self.state.llm_calls.append(
                {
                    'received_monotonic_ns': now,
                    'received_at_utc_ns': time.time_ns(),
                    'value': bool(message.data),
                }
            )
            self.condition.notify_all()

    def _formatted(self, message: String) -> None:
        with self.condition:
            now = self._touch()
            self.state.contexts.append(
                CollectedContext(now, embedded_timestamps(message.data), message.data)
            )
            cutoff = now - int(self.settings.context_lookback_seconds * NS_PER_SECOND)
            while self.state.contexts and self.state.contexts[0].received_monotonic_ns < cutoff:
                self.state.contexts.popleft()
            self.condition.notify_all()

    def _decision(self, message: String) -> None:
        with self.condition:
            now = self._touch()
            parsed = parse_decision(message.data)
            call_start = (
                self.state.pending_llm_calls_ns.popleft()
                if self.state.pending_llm_calls_ns
                else None
            )
            cutoff = now - int(self.settings.context_lookback_seconds * NS_PER_SECOND)
            contexts = [
                item for item in self.state.contexts if item.received_monotonic_ns >= cutoff
            ]
            candidates = sorted(
                {
                    timestamp
                    for context in contexts
                    for timestamp in context.source_timestamps_ns
                }
            )
            observations = [
                {
                    'source_timestamp_ns': timestamp,
                    'observed_monotonic_ns': context.received_monotonic_ns,
                }
                for context in contexts
                for timestamp in context.source_timestamps_ns
            ]
            parsed.update(
                {
                    'received_at_utc_ns': time.time_ns(),
                    'received_monotonic_ns': now,
                    'model_latency_seconds': (
                        (now - call_start) / NS_PER_SECOND
                        if call_start is not None
                        else None
                    ),
                    'source_timestamp_candidates_ns': candidates,
                    'source_replay_observations': observations,
                    'source_attribution': (
                        'recent_formatted_messages_heuristic' if candidates else None
                    ),
                    'context_messages': [item.raw for item in contexts],
                }
            )
            self.state.decisions.append(parsed)
            self.condition.notify_all()

    def _alert(self, message: String) -> None:
        with self.condition:
            now = self._touch()
            self.state.alerts.append(
                {
                    'raw': message.data,
                    'received_at_utc_ns': time.time_ns(),
                    'received_monotonic_ns': now,
                }
            )
            self.condition.notify_all()

    def wait_for_drain(self, minimum_wait: float, timeout: float) -> bool:
        """Wait for minimum observation time, no calls, and a short quiet period."""
        started = time.monotonic()
        deadline = started + timeout
        quiet_ns = int(0.5 * NS_PER_SECOND)
        with self.condition:
            while True:
                now = time.monotonic()
                quiet = time.monotonic_ns() - self.state.last_activity_ns >= quiet_ns
                waited = now - started >= minimum_wait
                if waited and quiet and not self.state.pending_llm_calls_ns:
                    return True
                remaining = deadline - now
                if remaining <= 0:
                    return False
                self.condition.wait(timeout=min(0.2, remaining))

    def wait_for_detector_ready(
        self,
        process: subprocess.Popen[Any],
        raw_input_topic: str,
        timeout: float,
    ) -> bool:
        """Wait for both the detector input subscription and decision publisher."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return False
            subscriptions = self.get_subscriptions_info_by_topic(raw_input_topic)
            publishers = self.get_publishers_info_by_topic(self.settings.decision_topic)
            if subscriptions and publishers:
                return True
            time.sleep(0.1)
        return False

    def snapshot(self) -> dict[str, Any]:
        """Copy collected state without exposing callback-owned containers."""
        with self.condition:
            return {
                'decisions': copy.deepcopy(self.state.decisions),
                'alerts': copy.deepcopy(self.state.alerts),
                'llm_calls': copy.deepcopy(self.state.llm_calls),
                'pending_llm_calls': len(self.state.pending_llm_calls_ns),
                'formatted_message_count': len(self.state.contexts),
            }


def add_flow_edge(graph: list[list[FlowEdge]], source: int, target: int, cost: int) -> FlowEdge:
    """Add a unit-capacity residual edge."""
    forward = FlowEdge(target, len(graph[target]), 1, cost)
    reverse = FlowEdge(source, len(graph[source]), 0, -cost)
    graph[source].append(forward)
    graph[target].append(reverse)
    return forward


def minimum_cost_maximum_matching(
    left_count: int,
    right_count: int,
    costs: dict[tuple[int, int], int],
) -> dict[int, int]:
    """Maximize match count, then minimize total edge cost."""
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
    for (left, right), cost in sorted(costs.items()):
        candidate_edges[(left, right)] = add_flow_edge(
            graph, left_offset + left, right_offset + right, cost
        )

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
                    candidate = distances[node] + edge.cost
                    if candidate < distances[edge.target]:
                        distances[edge.target] = candidate
                        previous[edge.target] = (node, edge_index)
                        changed = True
            if not changed:
                break
        if previous[sink] is None:
            break
        node = sink
        while node != source:
            prior, edge_index = previous[node]  # type: ignore[misc]
            edge = graph[prior][edge_index]
            edge.capacity -= 1
            graph[node][edge.reverse].capacity += 1
            node = prior
    return {
        left: right
        for (left, right), edge in candidate_edges.items()
        if edge.capacity == 0
    }


def stop_process(process: subprocess.Popen[Any] | None, timeout: float) -> None:
    """Stop an isolated process group, escalating only after a timeout."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def read_text_tail(path: Path, maximum_characters: int = 20_000) -> str:
    """Keep logs useful internally without retaining unbounded output."""
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return ''
    return text[-maximum_characters:]


def read_artifacts(directory: Path) -> list[dict[str, Any]]:
    """Load generated API artifacts and retain malformed-file diagnostics."""
    artifacts: list[dict[str, Any]] = []
    for path in sorted(directory.glob('api_artifact_*.json')):
        try:
            payload = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(payload, dict):
                raise ValueError('artifact root is not an object')
            payload['_file'] = path.name
            artifacts.append(payload)
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            artifacts.append({'_file': path.name, '_parse_error': str(exc)})
    return artifacts


def enrich_decisions_with_artifacts(
    decisions: list[dict[str, Any]], artifacts: list[dict[str, Any]]
) -> list[str]:
    """Attach artifact context to LLM-backed decisions in chronological order."""
    warnings: list[str] = []
    usable = [artifact for artifact in artifacts if '_parse_error' not in artifact]
    llm_decisions = [
        decision for decision in decisions if decision.get('model_latency_seconds') is not None
    ]
    for decision, artifact in zip(llm_decisions, usable):
        contexts = artifact.get('cached_data', [])
        if isinstance(contexts, list):
            candidates = sorted(
                {
                    timestamp
                    for context in contexts
                    for timestamp in embedded_timestamps(str(context))
                }
            )
            if candidates:
                decision['source_timestamp_candidates_ns'] = candidates
                decision['source_attribution'] = 'api_artifact.cached_data'
        timestamp = artifact.get('timestamp_ns')
        if isinstance(timestamp, int):
            decision['artifact_return_timestamp_ns'] = timestamp
        decision['artifact_id'] = artifact.get('artifact_id')
        decision['artifact_api_response'] = artifact.get('api_response')
    if len(usable) != len(llm_decisions):
        warnings.append(
            'Artifact/LLM-decision counts differ; unmatched records were preserved '
            f'(artifacts={len(usable)}, decisions={len(llm_decisions)}).'
        )
    return warnings


def latency_summary(values: Iterable[float]) -> dict[str, Any]:
    """Return JSON-safe latency statistics including tail behavior."""
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return {
            'count': 0,
            'mean_seconds': None,
            'median_seconds': None,
            'minimum_seconds': None,
            'maximum_seconds': None,
            'p90_seconds': None,
            'p95_seconds': None,
        }

    def percentile(fraction: float) -> float:
        rank = max(0, math.ceil(fraction * len(ordered)) - 1)
        return ordered[rank]

    return {
        'count': len(ordered),
        'mean_seconds': statistics.fmean(ordered),
        'median_seconds': statistics.median(ordered),
        'minimum_seconds': ordered[0],
        'maximum_seconds': ordered[-1],
        'p90_seconds': percentile(0.90),
        'p95_seconds': percentile(0.95),
    }


def safe_ratio(numerator: int, denominator: int) -> float | None:
    """Return None for an undefined metric instead of NaN."""
    return numerator / denominator if denominator else None


def evaluate_ground_truth(
    labels: list[dict[str, Any]],
    decisions: list[dict[str, Any]],
    buffer_seconds: float,
    playback_rate: float,
) -> dict[str, Any]:
    """Match positive decisions to truth labels within an inclusive time window."""
    positives = [decision for decision in decisions if decision.get('anomaly') is True]
    scorable: list[dict[str, Any]] = []
    unscorable: list[dict[str, Any]] = []
    for decision in positives:
        if decision.get('source_timestamp_candidates_ns'):
            scorable.append(decision)
        else:
            unscorable.append(decision)

    buffer_ns = int(round(buffer_seconds * NS_PER_SECOND))
    costs: dict[tuple[int, int], int] = {}
    chosen_sources: dict[tuple[int, int], int] = {}
    for label_index, label in enumerate(labels):
        label_time = int(label['timestamp_ns'])
        for decision_index, decision in enumerate(scorable):
            selected = min(
                decision['source_timestamp_candidates_ns'],
                key=lambda value: (abs(value - label_time), value),
            )
            distance = abs(selected - label_time)
            if distance <= buffer_ns:
                costs[(label_index, decision_index)] = distance
                chosen_sources[(label_index, decision_index)] = selected
    matches = minimum_cost_maximum_matching(len(labels), len(scorable), costs)
    matched_decisions = set(matches.values())

    events: list[dict[str, Any]] = []
    processing_latencies: list[float] = []
    label_latencies: list[float] = []
    warnings: list[str] = []
    for label_index, label in enumerate(labels):
        if label_index not in matches:
            events.append(
                {
                    'result': 'false_negative',
                    'label': label,
                    'decision': None,
                }
            )
            continue
        decision_index = matches[label_index]
        decision = scorable[decision_index]
        source_time = chosen_sources[(label_index, decision_index)]
        observations = [
            item
            for item in decision.get('source_replay_observations', [])
            if item.get('source_timestamp_ns') == source_time
        ]
        processing: float | None = None
        label_relative: float | None = None
        if observations:
            # Original ROS timestamps select the matching event. Monotonic
            # replay-observation times measure latency without mixing the old
            # recording clock with the current machine clock.
            observed_ns = max(item['observed_monotonic_ns'] for item in observations)
            processing = (
                decision['received_monotonic_ns'] - observed_ns
            ) / NS_PER_SECOND
            source_after_label = (
                source_time - int(label['timestamp_ns'])
            ) / NS_PER_SECOND
            label_relative = processing + source_after_label / playback_rate
            processing_latencies.append(processing)
            label_latencies.append(label_relative)
            if processing < 0:
                warnings.append(
                    f'Negative replay processing latency for {label["id"]}: '
                    f'{processing:.9f}s.'
                )
        else:
            warnings.append(
                f'No replay observation time was retained for the selected source '
                f'of {label["id"]}; accuracy is scored but latency is unavailable.'
            )
        events.append(
            {
                'result': 'true_positive',
                'label': label,
                'decision': decision,
                'selected_source_timestamp_ns': source_time,
                'event_difference_seconds': (
                    source_time - int(label['timestamp_ns'])
                )
                / NS_PER_SECOND,
                'processing_latency_seconds': processing,
                'label_relative_replay_latency_seconds': label_relative,
                'attribution_method': 'closest_source_timestamp_to_human_label',
            }
        )
    for decision_index, decision in enumerate(scorable):
        if decision_index not in matched_decisions:
            events.append(
                {
                    'result': 'false_positive',
                    'label': None,
                    'decision': decision,
                }
            )

    true_positives = len(matches)
    false_negatives = len(labels) - true_positives
    false_positives = len(scorable) - true_positives
    metrics = {
        'true_positives': true_positives,
        'false_negatives': false_negatives,
        'false_positives': false_positives,
        'unscorable_positive_decisions': len(unscorable),
        'detection_accuracy': safe_ratio(true_positives, len(labels)),
        'precision': safe_ratio(true_positives, true_positives + false_positives),
        'recall': safe_ratio(true_positives, true_positives + false_negatives),
        'f1': safe_ratio(
            2 * true_positives,
            2 * true_positives + false_positives + false_negatives,
        ),
        'complete': not unscorable,
    }
    return {
        'metrics': metrics,
        'processing_latency': latency_summary(processing_latencies),
        'label_relative_latency': latency_summary(label_latencies),
        'events': events,
        'unscorable_decisions': unscorable,
        'warnings': warnings,
    }


def execution_metrics(snapshot: dict[str, Any], elapsed_seconds: float) -> dict[str, Any]:
    """Summarize one replay independently of ground truth."""
    decisions = snapshot['decisions']
    latencies = [
        decision['model_latency_seconds']
        for decision in decisions
        if decision.get('model_latency_seconds') is not None
    ]
    positive_count = sum(decision.get('anomaly') is True for decision in decisions)
    negative_count = sum(decision.get('anomaly') is False for decision in decisions)
    unparseable = len(decisions) - positive_count - negative_count
    return {
        'elapsed_seconds': elapsed_seconds,
        'model_response_latency': latency_summary(latencies),
        'behavior': {
            'decision_count': len(decisions),
            'positive_decisions': positive_count,
            'negative_decisions': negative_count,
            'unparseable_decisions': unparseable,
            'positive_decision_rate': safe_ratio(positive_count, len(decisions)),
            'alert_count': len(snapshot['alerts']),
            'severity_distribution': dict(
                sorted(
                    {
                        severity: sum(
                            decision.get('severity') == severity for decision in decisions
                        )
                        for severity in {
                            decision.get('severity')
                            for decision in decisions
                            if decision.get('severity')
                        }
                    }.items()
                )
            ),
            'action_distribution': dict(
                sorted(
                    {
                        action: sum(
                            decision.get('action') == action for decision in decisions
                        )
                        for action in {
                            decision.get('action')
                            for decision in decisions
                            if decision.get('action')
                        }
                    }.items()
                )
            ),
        },
        'reliability': {
            'llm_calls': len(snapshot['llm_calls']),
            'pending_llm_calls_at_timeout': snapshot['pending_llm_calls'],
        },
    }


def playback_timeout(bag: BagInfo, settings: RunnerSettings) -> float | None:
    """Derive a bounded timeout from recorded duration and playback rate."""
    if bag.duration_seconds is None:
        return None
    return (
        bag.duration_seconds / settings.playback_rate
        + settings.playback_timeout_padding_seconds
    )


def execute_one(
    bag: BagInfo,
    experiment: Experiment,
    trial: int,
    settings: RunnerSettings,
    collector: EvaluationCollector,
    labels: list[dict[str, Any]],
    label_warnings: list[str],
) -> dict[str, Any]:
    """Run one isolated bag/configuration/trial execution."""
    execution_id = f'{experiment.name}:{bag.path.name}:trial-{trial}'
    started_at = datetime.now(timezone.utc)
    result: dict[str, Any] = {
        'execution_id': execution_id,
        'bag': str(bag.path),
        'experiment': experiment.name,
        'trial': trial,
        'status': 'starting',
        'started_at_utc': started_at.isoformat(),
        'bag_topics': bag.topics,
        'bag_duration_seconds': bag.duration_seconds,
        'warnings': list(label_warnings),
        'errors': [],
    }
    raw_topic = str(experiment.aad_config.get('raw_input_topic', '/ai_anomaly_logging'))
    if raw_topic not in bag.topics:
        result['status'] = 'invalid_bag'
        result['errors'].append(f'Required detector input topic {raw_topic!r} is missing.')
        return result
    if (
        settings.mode == 'human-labeled'
        and settings.human_label_topic not in bag.topics
    ):
        result['status'] = 'invalid_bag'
        result['errors'].append(
            f'Required human-label topic {settings.human_label_topic!r} is missing.'
        )
        return result
    if settings.dry_run:
        result['status'] = 'dry_run_validated'
        result['ground_truth'] = (
            {'label_count': len(labels)} if settings.mode == 'human-labeled' else None
        )
        return result

    collector.reset()
    detector_process: subprocess.Popen[Any] | None = None
    player_process: subprocess.Popen[Any] | None = None
    start_monotonic = time.monotonic()

    with tempfile.TemporaryDirectory(prefix='aad_offline_eval_') as temp_name:
        temp_directory = Path(temp_name)
        artifact_directory = temp_directory / 'artifacts'
        detector_log = temp_directory / 'detector.log'
        player_log = temp_directory / 'player.log'
        runtime_config = copy.deepcopy(experiment.aad_config)
        runtime_config['api_artifact_output_dir'] = str(artifact_directory)
        runtime_config['api_artifact_max_files'] = 0
        runtime_config_path = temp_directory / 'aad_config.yaml'
        runtime_config_path.write_text(
            yaml.safe_dump(runtime_config, sort_keys=False), encoding='utf-8'
        )
        environment = {**os.environ, 'AAD_CONFIG_PATH': str(runtime_config_path)}

        try:
            with detector_log.open('w', encoding='utf-8') as detector_stream:
                detector_process = subprocess.Popen(
                    settings.detector_command,
                    env=environment,
                    stdout=detector_stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                )
            if not collector.wait_for_detector_ready(
                detector_process,
                raw_topic,
                settings.startup_timeout_seconds,
            ):
                if detector_process.poll() is not None:
                    raise EvaluationError(
                        f'Detector exited during startup with code '
                        f'{detector_process.returncode}.'
                    )
                raise EvaluationError(
                    'Detector readiness timed out before its input subscription '
                    'and decision publisher appeared.'
                )
            if settings.startup_grace_seconds:
                time.sleep(settings.startup_grace_seconds)

            playback_command = [
                'ros2',
                'bag',
                'play',
                str(bag.path),
                '--rate',
                str(settings.playback_rate),
            ]
            with player_log.open('w', encoding='utf-8') as player_stream:
                player_process = subprocess.Popen(
                    playback_command,
                    stdout=player_stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                )
            try:
                player_process.wait(timeout=playback_timeout(bag, settings))
            except subprocess.TimeoutExpired as exc:
                stop_process(player_process, settings.shutdown_timeout_seconds)
                raise EvaluationError('Bag playback exceeded its derived timeout.') from exc
            if player_process.returncode != 0:
                raise EvaluationError(
                    f'Bag playback exited with code {player_process.returncode}.'
                )

            drained = collector.wait_for_drain(
                settings.post_playback_grace_seconds,
                settings.inference_drain_timeout_seconds,
            )
            if not drained:
                result['warnings'].append(
                    'Inference drain timed out; pending work is recorded in reliability metrics.'
                )
            snapshot = collector.snapshot()
            artifacts = read_artifacts(artifact_directory)
            result['warnings'].extend(
                enrich_decisions_with_artifacts(snapshot['decisions'], artifacts)
            )
            elapsed = time.monotonic() - start_monotonic
            result.update(
                {
                    'status': 'completed' if drained else 'completed_with_timeout',
                    'finished_at_utc': datetime.now(timezone.utc).isoformat(),
                    'metrics': execution_metrics(snapshot, elapsed),
                    'decisions': snapshot['decisions'],
                    'alerts': snapshot['alerts'],
                    'llm_calls': snapshot['llm_calls'],
                    'artifacts': artifacts,
                }
            )
            if settings.mode == 'human-labeled':
                result['ground_truth'] = evaluate_ground_truth(
                    labels,
                    snapshot['decisions'],
                    settings.label_buffer_seconds,
                    settings.playback_rate,
                )
            else:
                result['ground_truth'] = None
        except EvaluationError as exc:
            result['status'] = 'failed'
            result['errors'].append(str(exc))
            result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        except Exception as exc:
            result['status'] = 'failed'
            result['errors'].append(f'Unexpected execution error: {type(exc).__name__}: {exc}')
            result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        finally:
            stop_process(player_process, settings.shutdown_timeout_seconds)
            stop_process(detector_process, settings.shutdown_timeout_seconds)
            result['detector_log_tail'] = read_text_tail(detector_log)
            result['player_log_tail'] = read_text_tail(player_log)
    return result


def aggregate_executions(
    executions: Sequence[dict[str, Any]],
    mode: str,
    expected: int | None = None,
) -> dict[str, Any]:
    """Aggregate performance, behavior, reliability, and optional truth metrics."""
    completed = [item for item in executions if item['status'].startswith('completed')]
    model_latencies = [
        decision['model_latency_seconds']
        for item in completed
        for decision in item.get('decisions', [])
        if decision.get('model_latency_seconds') is not None
    ]
    positive = sum(
        decision.get('anomaly') is True
        for item in completed
        for decision in item.get('decisions', [])
    )
    negative = sum(
        decision.get('anomaly') is False
        for item in completed
        for decision in item.get('decisions', [])
    )
    calls = sum(len(item.get('llm_calls', [])) for item in completed)
    result: dict[str, Any] = {
        'executions_expected': len(executions) if expected is None else expected,
        'executions_completed': len(completed),
        'executions_failed': sum(item['status'] == 'failed' for item in executions),
        'executions_invalid': sum(item['status'] == 'invalid_bag' for item in executions),
        'model_response_latency': latency_summary(model_latencies),
        'behavior': {
            'positive_decisions': positive,
            'negative_decisions': negative,
            'positive_decision_rate': safe_ratio(positive, positive + negative),
            'alerts': sum(len(item.get('alerts', [])) for item in completed),
        },
        'reliability': {
            'llm_calls': calls,
            'pending_llm_calls_at_timeout': sum(
                item.get('metrics', {})
                .get('reliability', {})
                .get('pending_llm_calls_at_timeout', 0)
                for item in completed
            ),
        },
    }
    if mode == 'unlabeled':
        result['ground_truth_metrics'] = None
        result['ground_truth_note'] = (
            'Unavailable in unlabeled mode; behavior and agreement are not accuracy.'
        )
        return result

    truth_runs = [item['ground_truth'] for item in completed if item.get('ground_truth')]
    true_positives = sum(item['metrics']['true_positives'] for item in truth_runs)
    false_negatives = sum(item['metrics']['false_negatives'] for item in truth_runs)
    false_positives = sum(item['metrics']['false_positives'] for item in truth_runs)
    unscorable = sum(
        item['metrics']['unscorable_positive_decisions'] for item in truth_runs
    )
    result['ground_truth_metrics'] = {
        'true_positives': true_positives,
        'false_negatives': false_negatives,
        'false_positives': false_positives,
        'unscorable_positive_decisions': unscorable,
        'detection_accuracy': safe_ratio(
            true_positives, true_positives + false_negatives
        ),
        'precision': safe_ratio(true_positives, true_positives + false_positives),
        'recall': safe_ratio(true_positives, true_positives + false_negatives),
        'f1': safe_ratio(
            2 * true_positives,
            2 * true_positives + false_positives + false_negatives,
        ),
        'complete': not unscorable and len(truth_runs) == len(completed),
    }
    return result


def numeric_summary(values: Iterable[Any]) -> dict[str, Any]:
    """Summarize finite per-run values, including sample variability."""
    numbers = [
        float(value)
        for value in values
        if isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    ]
    if not numbers:
        return {
            'count': 0,
            'mean': None,
            'standard_deviation': None,
            'minimum': None,
            'maximum': None,
        }
    return {
        'count': len(numbers),
        'mean': statistics.fmean(numbers),
        'standard_deviation': (
            statistics.stdev(numbers) if len(numbers) > 1 else None
        ),
        'minimum': min(numbers),
        'maximum': max(numbers),
    }


def repeatability_summary(
    executions: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Measure per-run variation and bag-level outcome consistency."""
    completed = [item for item in executions if item['status'].startswith('completed')]

    def nested_value(execution: dict[str, Any], *keys: str) -> Any:
        value: Any = execution
        for key in keys:
            if not isinstance(value, dict):
                return None
            value = value.get(key)
        return value

    metric_paths = {
        'mean_response_latency_seconds': (
            'metrics',
            'model_response_latency',
            'mean_seconds',
        ),
        'elapsed_seconds': ('metrics', 'elapsed_seconds'),
        'positive_decisions': ('metrics', 'behavior', 'positive_decisions'),
        'negative_decisions': ('metrics', 'behavior', 'negative_decisions'),
        'alerts': ('metrics', 'behavior', 'alert_count'),
        'llm_calls': ('metrics', 'reliability', 'llm_calls'),
    }
    summaries = {
        name: numeric_summary(nested_value(execution, *path) for execution in completed)
        for name, path in metric_paths.items()
    }

    by_bag: dict[str, list[dict[str, Any]]] = {}
    for execution in completed:
        by_bag.setdefault(execution['bag'], []).append(execution)
    bag_rows = []
    for bag, bag_executions in sorted(by_bag.items()):
        positive_counts = [
            int(nested_value(execution, 'metrics', 'behavior', 'positive_decisions') or 0)
            for execution in bag_executions
        ]
        positive_outcomes = [count > 0 for count in positive_counts]
        bag_rows.append(
            {
                'bag': bag,
                'completed_trials': len(bag_executions),
                'trials_with_positive_decisions': sum(positive_outcomes),
                'positive_outcome_rate': safe_ratio(
                    sum(positive_outcomes), len(positive_outcomes)
                ),
                'unanimous_positive_outcome': len(set(positive_outcomes)) <= 1,
                'identical_positive_decision_counts': len(set(positive_counts)) <= 1,
                'positive_decision_counts': positive_counts,
            }
        )
    return {
        'completed_executions': len(completed),
        'metric_variability': summaries,
        'bag_consistency': bag_rows,
    }


def configuration_comparisons(
    grouped: dict[str, list[dict[str, Any]]]
) -> list[dict[str, Any]]:
    """Compare bag-level positive behavior between every experiment pair."""
    comparisons: list[dict[str, Any]] = []
    by_name_and_key: dict[str, dict[tuple[str, int], dict[str, Any]]] = {}
    for name, executions in grouped.items():
        by_name_and_key[name] = {
            (item['bag'], item['trial']): item
            for item in executions
            if item['status'].startswith('completed')
        }
    for left_name, right_name in itertools.combinations(sorted(grouped), 2):
        left = by_name_and_key[left_name]
        right = by_name_and_key[right_name]
        shared = sorted(set(left).intersection(right))
        rows = []
        agreements = 0
        for key in shared:
            left_positive = any(
                decision.get('anomaly') is True for decision in left[key]['decisions']
            )
            right_positive = any(
                decision.get('anomaly') is True for decision in right[key]['decisions']
            )
            agrees = left_positive == right_positive
            agreements += agrees
            rows.append(
                {
                    'bag': key[0],
                    'trial': key[1],
                    'left_positive': left_positive,
                    'right_positive': right_positive,
                    'agrees': agrees,
                    'left_positive_count': sum(
                        decision.get('anomaly') is True
                        for decision in left[key]['decisions']
                    ),
                    'right_positive_count': sum(
                        decision.get('anomaly') is True
                        for decision in right[key]['decisions']
                    ),
                }
            )
        comparisons.append(
            {
                'left_experiment': left_name,
                'right_experiment': right_name,
                'shared_executions': len(shared),
                'bag_level_positive_agreement': safe_ratio(agreements, len(shared)),
                'note': 'Agreement is not ground-truth accuracy.',
                'executions': rows,
            }
        )
    return comparisons


def reserve_report_path(directory: Path, started: datetime) -> Path:
    """Atomically reserve a timestamped report path without overwriting."""
    directory.mkdir(parents=True, exist_ok=True)
    stem = f'offline_evaluation_{started:%Y%m%d_%H%M%S}'
    suffix = 0
    while True:
        postfix = '' if suffix == 0 else f'_{suffix}'
        candidate = directory / f'{stem}{postfix}.md'
        try:
            # Exclusive creation prevents concurrent evaluators started in the
            # same second from selecting the same output file.
            descriptor = os.open(
                candidate, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644
            )
        except FileExistsError:
            suffix += 1
            continue
        except OSError as exc:
            raise EvaluationError(
                f'Could not reserve report path {candidate}: {exc}'
            ) from exc
        os.close(descriptor)
        return candidate


def source_revision() -> str | None:
    """Record a Git revision when the workspace is inside a repository."""
    try:
        result = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            timeout=5,
            check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def markdown_value(value: Any) -> str:
    """Format and escape a value for use inside a Markdown table."""
    if value is None:
        return '—'
    if isinstance(value, bool):
        return 'Yes' if value else 'No'
    return str(value).replace('|', '\\|').replace('\n', ' ')


def markdown_table(
    headers: Sequence[str], rows: Sequence[Sequence[Any]]
) -> list[str]:
    """Return a GitHub-flavored Markdown table."""
    output = [
        '| ' + ' | '.join(headers) + ' |',
        '| ' + ' | '.join('---' for _ in headers) + ' |',
    ]
    output.extend(
        '| ' + ' | '.join(markdown_value(value) for value in row) + ' |'
        for row in rows
    )
    return output


def seconds_text(value: Any) -> str:
    """Format an optional duration consistently."""
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return '—'
    return f'{value:.3f} s'


def number_text(value: Any) -> str:
    """Format an optional numeric statistic without excessive precision."""
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return '—'
    return f'{value:.3f}'


def ratio_text(value: Any) -> str:
    """Format an optional ratio as a percentage."""
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return '—'
    return f'{value:.1%}'


def distribution_text(value: Any) -> str:
    """Format a distribution mapping as a compact readable list."""
    if not isinstance(value, dict) or not value:
        return 'None'
    return ', '.join(f'{key}: {count}' for key, count in sorted(value.items()))


def metric_rows(metrics: Any) -> list[tuple[str, Any]]:
    """Flatten common execution metrics into human-readable rows."""
    if not isinstance(metrics, dict):
        metrics = {}
    latency = metrics.get('model_response_latency', {})
    behavior = metrics.get('behavior', {})
    reliability = metrics.get('reliability', {})
    rows: list[tuple[str, Any]] = []
    if 'elapsed_seconds' in metrics:
        rows.append(('Elapsed time', seconds_text(metrics.get('elapsed_seconds'))))
    rows.extend(
        [
            ('Decisions measured for latency', latency.get('count', 0)),
            ('Mean response latency', seconds_text(latency.get('mean_seconds'))),
            ('Median response latency', seconds_text(latency.get('median_seconds'))),
            ('Minimum response latency', seconds_text(latency.get('minimum_seconds'))),
            ('Maximum response latency', seconds_text(latency.get('maximum_seconds'))),
            ('P90 response latency', seconds_text(latency.get('p90_seconds'))),
            ('P95 response latency', seconds_text(latency.get('p95_seconds'))),
            ('Positive decisions', behavior.get('positive_decisions', 0)),
            ('Negative decisions', behavior.get('negative_decisions', 0)),
            ('Unparseable decisions', behavior.get('unparseable_decisions', 0)),
            (
                'Positive decision rate',
                ratio_text(behavior.get('positive_decision_rate')),
            ),
            ('Alerts', behavior.get('alert_count', behavior.get('alerts', 0))),
            (
                'Severity distribution',
                distribution_text(behavior.get('severity_distribution')),
            ),
            (
                'Action distribution',
                distribution_text(behavior.get('action_distribution')),
            ),
            ('LLM calls', reliability.get('llm_calls', 0)),
            (
                'Pending LLM calls at timeout',
                reliability.get('pending_llm_calls_at_timeout', 0),
            ),
        ]
    )
    return rows


def append_messages(lines: list[str], heading: str, messages: Any) -> None:
    """Append a warning or error list when it contains messages."""
    if not isinstance(messages, list) or not messages:
        return
    lines.extend(['', f'#### {heading}', ''])
    lines.extend(f'- {message}' for message in messages)


def render_ground_truth(lines: list[str], truth: Any) -> None:
    """Append human-label metrics without exposing raw cached messages."""
    if not isinstance(truth, dict):
        return
    lines.extend(['', '#### Human-label results', ''])
    metrics = truth.get('metrics', {})
    rows = []
    if isinstance(metrics, dict):
        for key, value in metrics.items():
            label = key.replace('_', ' ').capitalize()
            if key in {'detection_accuracy', 'precision', 'recall', 'f1'}:
                value = ratio_text(value)
            rows.append((label, value))
    lines.extend(markdown_table(('Metric', 'Result'), rows))

    for key, label in (
        ('processing_latency', 'Processing latency'),
        ('label_relative_latency', 'Label-relative latency'),
    ):
        summary = truth.get(key)
        if not isinstance(summary, dict):
            continue
        latency_rows = [
            ('Count', summary.get('count', 0)),
            ('Mean', seconds_text(summary.get('mean_seconds'))),
            ('Median', seconds_text(summary.get('median_seconds'))),
            ('Minimum', seconds_text(summary.get('minimum_seconds'))),
            ('Maximum', seconds_text(summary.get('maximum_seconds'))),
            ('P90', seconds_text(summary.get('p90_seconds'))),
            ('P95', seconds_text(summary.get('p95_seconds'))),
        ]
        lines.extend(['', f'**{label}**', ''])
        lines.extend(markdown_table(('Metric', 'Result'), latency_rows))

    events = truth.get('events', [])
    event_rows = []
    if isinstance(events, list):
        for event in events:
            if not isinstance(event, dict):
                continue
            label = event.get('label')
            decision = event.get('decision')
            event_rows.append(
                (
                    event.get('result'),
                    label.get('id') if isinstance(label, dict) else None,
                    decision.get('severity') if isinstance(decision, dict) else None,
                    decision.get('action') if isinstance(decision, dict) else None,
                    seconds_text(event.get('event_difference_seconds')),
                    seconds_text(event.get('processing_latency_seconds')),
                )
            )
    if event_rows:
        lines.extend(['', '**Matched events**', ''])
        lines.extend(
            markdown_table(
                (
                    'Result',
                    'Label',
                    'Severity',
                    'Action',
                    'Event offset',
                    'Processing latency',
                ),
                event_rows,
            )
        )
    append_messages(lines, 'Ground-truth warnings', truth.get('warnings'))


def render_report_markdown(report: dict[str, Any]) -> str:
    """Render a detailed Markdown report from structured evaluation results."""
    experiment = report['experiment']
    overall = report['overall']
    status = str(experiment['run_status']).replace('_', ' ').title()
    lines = [
        '# Offline Anomaly-Detection Evaluation',
        '',
        f'**Status:** {status}',
        '',
        '## Experiment summary',
        '',
    ]
    execution_count = (
        f'{experiment["attempted_executions"]} attempted / '
        f'{experiment["expected_executions"]} expected'
    )
    summary_rows = [
        ('Mode', experiment['evaluation_mode']),
        ('Started (UTC)', experiment['started_at_utc']),
        ('Finished (UTC)', experiment.get('finished_at_utc')),
        ('Playback rate', f'{experiment["playback_rate"]}×'),
        ('Executions', execution_count),
        ('Configuration file', experiment['evaluation_yaml']),
        ('Recordings location', experiment['recordings_folder']),
        ('Source revision', experiment.get('source_revision')),
        ('Report schema', report['schema_version']),
    ]
    lines.extend(markdown_table(('Item', 'Value'), summary_rows))
    lines.extend(['', '**Recordings**', ''])
    lines.extend(
        markdown_table(
            ('Bag', 'Recorded duration'),
            [
                (bag['path'], seconds_text(bag.get('duration_seconds')))
                for bag in experiment['bags']
            ],
        )
    )
    if experiment['evaluation_mode'] == 'unlabeled':
        lines.extend(
            [
                '',
                '> This was an unlabeled evaluation. Detection counts and '
                'configuration agreement are not accuracy measurements.',
            ]
        )

    lines.extend(['', '## Overall results', ''])
    execution_rows = [
        ('Expected executions', overall['executions_expected']),
        ('Completed executions', overall['executions_completed']),
        ('Failed executions', overall['executions_failed']),
        ('Invalid executions', overall['executions_invalid']),
    ]
    lines.extend(markdown_table(('Metric', 'Result'), execution_rows))
    lines.extend(['', '### Overall behavior and latency', ''])
    lines.extend(markdown_table(('Metric', 'Result'), metric_rows(overall)))

    comparisons = report.get('configuration_comparisons', [])
    lines.extend(['', '## Configuration comparisons', ''])
    if not comparisons:
        lines.append('No pairwise configuration comparison was available.')
    for comparison in comparisons:
        comparison_title = (
            f'{comparison["left_experiment"]} vs. '
            f'{comparison["right_experiment"]}'
        )
        agreement = ratio_text(comparison.get('bag_level_positive_agreement'))
        shared = comparison.get('shared_executions', 0)
        lines.extend(
            [
                f'### {comparison_title}',
                '',
                f'Bag-level positive agreement: {agreement} across '
                f'{shared} shared run(s).',
                '',
            ]
        )
        comparison_rows = [
            (
                Path(item['bag']).name,
                item['trial'],
                item['left_positive_count'],
                item['right_positive_count'],
                item['agrees'],
            )
            for item in comparison.get('executions', [])
        ]
        lines.extend(
            markdown_table(
                (
                    'Bag',
                    'Trial',
                    'Left positives',
                    'Right positives',
                    'Agrees',
                ),
                comparison_rows,
            )
        )

    lines.extend(['', '## Configuration details'])
    for configuration in report['configurations']:
        lines.extend(['', f'### {configuration["name"]}', ''])
        parameters = configuration['evaluated_parameters']
        llm = parameters.get('llm', {})
        parameter_rows = [
            ('Configuration hash', configuration['configuration_hash']),
            ('API frequency', seconds_text(parameters.get('api_frequency_seconds'))),
            ('Maximum cached messages', parameters.get('cache_max_items')),
            ('Maximum cache age', seconds_text(parameters.get('cache_max_age_seconds'))),
            (
                'Minimum trigger importance',
                parameters.get('llm_min_trigger_importance'),
            ),
            ('Model provider', llm.get('model_provider')),
            ('Model', llm.get('model')),
            ('Local model', llm.get('local')),
            ('Model token context', llm.get('num_ctx')),
            ('Vision enabled', llm.get('vision_enabled')),
            ('Image context enabled', llm.get('image_context_enabled')),
            ('Routine image frames', llm.get('image_context_max_frames')),
            ('Maximum image frames', llm.get('image_max_frames')),
        ]
        lines.extend(markdown_table(('Parameter', 'Value'), parameter_rows))
        lines.extend(['', '#### Aggregate results', ''])
        lines.extend(
            markdown_table(
                ('Metric', 'Result'), metric_rows(configuration['aggregate'])
            )
        )
        repeatability = configuration.get('repeatability', {})
        variability = repeatability.get('metric_variability', {})
        lines.extend(['', '#### Repeatability across runs', ''])
        if not variability:
            lines.append('No completed runs are available for variability analysis.')
        else:
            variability_rows = []
            labels = {
                'mean_response_latency_seconds': 'Mean response latency (seconds)',
                'elapsed_seconds': 'Elapsed time (seconds)',
                'positive_decisions': 'Positive decisions',
                'negative_decisions': 'Negative decisions',
                'alerts': 'Alerts',
                'llm_calls': 'LLM calls',
            }
            for key, summary in variability.items():
                variability_rows.append(
                    (
                        labels.get(key, key.replace('_', ' ').capitalize()),
                        summary.get('count', 0),
                        number_text(summary.get('mean')),
                        number_text(summary.get('standard_deviation')),
                        number_text(summary.get('minimum')),
                        number_text(summary.get('maximum')),
                    )
                )
            lines.extend(
                markdown_table(
                    ('Metric', 'Runs', 'Mean', 'Std. dev.', 'Min', 'Max'),
                    variability_rows,
                )
            )
        consistency = repeatability.get('bag_consistency', [])
        if consistency:
            consistency_rows = [
                (
                    Path(item['bag']).name,
                    item['completed_trials'],
                    item['trials_with_positive_decisions'],
                    ratio_text(item.get('positive_outcome_rate')),
                    item['unanimous_positive_outcome'],
                    item['identical_positive_decision_counts'],
                    ', '.join(
                        str(count) for count in item['positive_decision_counts']
                    ),
                )
                for item in consistency
            ]
            lines.extend(['', '**Bag-level trial consistency**', ''])
            lines.extend(
                markdown_table(
                    (
                        'Bag',
                        'Completed trials',
                        'Trials with positives',
                        'Positive outcome rate',
                        'Same outcome',
                        'Same positive count',
                        'Positive counts by trial',
                    ),
                    consistency_rows,
                )
            )
        for execution in configuration['executions']:
            bag_name = Path(execution['bag']).name
            lines.extend(
                [
                    '',
                    f'#### Run: {bag_name} (trial {execution["trial"]})',
                    '',
                ]
            )
            run_rows = [
                ('Status', execution['status']),
                ('Started (UTC)', execution['started_at_utc']),
                ('Finished (UTC)', execution.get('finished_at_utc')),
                (
                    'Recorded bag duration',
                    seconds_text(execution.get('bag_duration_seconds')),
                ),
            ]
            lines.extend(markdown_table(('Item', 'Value'), run_rows))
            lines.extend(['', '**Run metrics**', ''])
            lines.extend(
                markdown_table(
                    ('Metric', 'Result'),
                    metric_rows(execution.get('metrics', {})),
                )
            )
            decisions = execution.get('final_decisions', [])
            if isinstance(decisions, list) and decisions:
                decision_rows = [
                    (
                        index,
                        decision.get('anomaly'),
                        decision.get('severity'),
                        decision.get('action'),
                        seconds_text(decision.get('model_latency_seconds')),
                        decision.get('summary'),
                    )
                    for index, decision in enumerate(decisions, start=1)
                    if isinstance(decision, dict)
                ]
                lines.extend(['', '**Final decisions**', ''])
                lines.extend(
                    markdown_table(
                        (
                            '#',
                            'Anomaly',
                            'Severity',
                            'Action',
                            'Model latency',
                            'Summary',
                        ),
                        decision_rows,
                    )
                )
            append_messages(lines, 'Warnings', execution.get('warnings'))
            append_messages(lines, 'Errors', execution.get('errors'))
            render_ground_truth(lines, execution.get('ground_truth'))

    lines.extend(
        [
            '',
            '---',
            '',
            'Raw periodic messages, API artifacts, and process logs are omitted '
            'from this report.',
            '',
        ]
    )
    return '\n'.join(lines)


def write_report(report: dict[str, Any], path: Path) -> None:
    """Atomically replace a report so interruption cannot corrupt a checkpoint."""
    temporary_path: Path | None = None
    try:
        rendered = render_report_markdown(report)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent
        )
        temporary_path = Path(temporary_name)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        temporary_path = None
    except (OSError, TypeError, ValueError) as exc:
        raise EvaluationError(f'Could not write report {path}: {exc}') from exc
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def evaluated_parameters(configuration: dict[str, Any]) -> dict[str, Any]:
    """Retain comparison inputs without copying the full runtime configuration."""
    llm_value = configuration.get('llm', {})
    llm = llm_value if isinstance(llm_value, dict) else {}
    keys = (
        'model_provider',
        'model',
        'local',
        'vision_enabled',
        'image_context_enabled',
        'image_context_max_frames',
        'image_max_frames',
        'num_ctx',
    )
    return {
        'api_frequency_seconds': configuration.get('api_frequency_seconds'),
        'cache_max_items': configuration.get('cache_max_items'),
        'cache_max_age_seconds': configuration.get('cache_max_age_seconds'),
        'llm_min_trigger_importance': configuration.get(
            'llm_min_trigger_importance'
        ),
        'llm': {key: llm.get(key) for key in keys},
    }


def compact_decision(decision: dict[str, Any]) -> dict[str, Any]:
    """Retain the final decision without its raw prompt context."""
    return {
        'anomaly': decision.get('anomaly'),
        'severity': decision.get('severity'),
        'action': decision.get('action'),
        'summary': decision.get('summary'),
        'model_latency_seconds': decision.get('model_latency_seconds'),
    }


def compact_ground_truth(truth: Any) -> dict[str, Any] | None:
    """Retain label results while removing raw decisions and label payloads."""
    if not isinstance(truth, dict):
        return None
    compact_events = []
    events = truth.get('events', [])
    if isinstance(events, list):
        for event in events:
            if not isinstance(event, dict):
                continue
            label = event.get('label')
            decision = event.get('decision')
            compact_events.append(
                {
                    'result': event.get('result'),
                    'label': (
                        {
                            'id': label.get('id'),
                            'timestamp_ns': label.get('timestamp_ns'),
                            'timestamp_source': label.get('timestamp_source'),
                        }
                        if isinstance(label, dict)
                        else None
                    ),
                    'decision': (
                        compact_decision(decision)
                        if isinstance(decision, dict)
                        else None
                    ),
                    'event_difference_seconds': event.get(
                        'event_difference_seconds'
                    ),
                    'processing_latency_seconds': event.get(
                        'processing_latency_seconds'
                    ),
                    'label_relative_replay_latency_seconds': event.get(
                        'label_relative_replay_latency_seconds'
                    ),
                }
            )
    unscorable = truth.get('unscorable_decisions', [])
    return {
        'metrics': truth.get('metrics'),
        'processing_latency': truth.get('processing_latency'),
        'label_relative_latency': truth.get('label_relative_latency'),
        'events': compact_events,
        'unscorable_decision_count': (
            len(unscorable) if isinstance(unscorable, list) else 0
        ),
        'warnings': truth.get('warnings', []),
    }


def compact_execution(execution: dict[str, Any]) -> dict[str, Any]:
    """Keep final per-run results while omitting raw periodic and debug records."""
    keys = (
        'execution_id',
        'bag',
        'experiment',
        'trial',
        'status',
        'started_at_utc',
        'finished_at_utc',
        'bag_duration_seconds',
        'warnings',
        'errors',
        'metrics',
        'ground_truth',
    )
    compact = {key: execution.get(key) for key in keys if key != 'ground_truth'}
    decisions = execution.get('decisions', [])
    compact['final_decisions'] = [
        compact_decision(decision)
        for decision in decisions
        if isinstance(decision, dict)
    ]
    compact['ground_truth'] = compact_ground_truth(execution.get('ground_truth'))
    return compact


def build_report(
    *,
    settings: RunnerSettings,
    config_path: Path,
    bags: Sequence[BagInfo],
    experiments: Sequence[Experiment],
    grouped: dict[str, list[dict[str, Any]]],
    started: datetime,
    expected: int,
    revision: str | None,
    run_status: str,
) -> dict[str, Any]:
    """Build a report for a running, completed, aborted, or interrupted matrix."""
    configuration_reports: list[dict[str, Any]] = []
    all_executions: list[dict[str, Any]] = []
    expected_per_experiment = {
        experiment.name: len(bags) * experiment.trials
        for experiment in experiments
    }
    for experiment in experiments:
        executions = grouped[experiment.name]
        all_executions.extend(executions)
        configuration_reports.append(
            {
                'name': experiment.name,
                'configuration_hash': experiment.config_hash,
                'evaluated_parameters': evaluated_parameters(
                    experiment.aad_config
                ),
                'aggregate': aggregate_executions(
                    executions,
                    settings.mode,
                    expected_per_experiment[experiment.name],
                ),
                'repeatability': repeatability_summary(executions),
                'executions': [
                    compact_execution(execution) for execution in executions
                ],
            }
        )

    updated = datetime.now(timezone.utc)
    return {
        'schema_version': SCHEMA_VERSION,
        'experiment': {
            'started_at_utc': started.isoformat(),
            'updated_at_utc': updated.isoformat(),
            'finished_at_utc': (
                None if run_status == 'running' else updated.isoformat()
            ),
            'run_status': run_status,
            'evaluation_mode': settings.mode,
            'dry_run': settings.dry_run,
            'evaluation_yaml': str(config_path),
            'recordings_folder': str(settings.bags_path),
            'playback_rate': settings.playback_rate,
            'source_revision': revision,
            'expected_executions': expected,
            'attempted_executions': len(all_executions),
            'aborted_early': run_status in {'aborted', 'interrupted'},
            'bags': [
                {
                    'path': str(bag.path),
                    'duration_seconds': bag.duration_seconds,
                }
                for bag in bags
            ],
        },
        'overall': aggregate_executions(
            all_executions, settings.mode, expected
        ),
        'configuration_comparisons': configuration_comparisons(grouped),
        'configurations': configuration_reports,
    }


def example_configuration() -> dict[str, Any]:
    """Return a runnable template using the repository's current AAD config."""
    base_config = (
        Path(__file__).resolve().parents[1]
        / 'anomaly_detection'
        / 'anomaly_detection'
        / 'config.yaml'
    )
    return {
        'runner': {
            'bags': '/path/to/recordings',
            'output_directory': './evaluation_results',
            'mode': 'unlabeled',
            'trials': 1,
            'playback_rate': 1.0,
            'continue_on_error': True,
            'startup_timeout_seconds': 15.0,
            'startup_grace_seconds': 3.0,
            'post_playback_grace_seconds': 2.0,
            'inference_drain_timeout_seconds': 60.0,
            'shutdown_timeout_seconds': 20.0,
            'context_lookback_seconds': 30.0,
            'label_buffer_seconds': 5.0,
            'human_label_topic': None,
        },
        'base_config': str(base_config),
        'experiments': [
            {
                'name': 'local_text_only',
                'overrides': {'llm': {'vision_enabled': False}},
            },
            {
                'name': 'local_vision',
                'overrides': {
                    'llm': {
                        'vision_enabled': True,
                        'image_context_enabled': True,
                        'image_max_frames': 2,
                    }
                },
            },
        ],
    }


def write_example_configuration(path: Path) -> None:
    """Materialize the embedded YAML template on request."""
    path = path.expanduser().resolve()
    if path.exists():
        raise EvaluationError(f'Refusing to overwrite existing file: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        path.write_text(
            yaml.safe_dump(example_configuration(), sort_keys=False), encoding='utf-8'
        )
    except OSError as exc:
        raise EvaluationError(f'Could not write example config {path}: {exc}') from exc
    print(f'Example configuration written to {path}')


def build_parser() -> argparse.ArgumentParser:
    """Define CLI overrides while allowing zero-argument quick runs."""
    parser = argparse.ArgumentParser(
        description=(
            'Replay a folder of ROS 2 bags through multiple YAML-defined '
            'anomaly-detection configurations.'
        )
    )
    parser.add_argument('--bags', help='Recordings folder or file:// link.')
    parser.add_argument('--config', help='Evaluation YAML path or file:// link.')
    parser.add_argument('--output-dir', help='Override the report directory.')
    parser.add_argument(
        '--trials',
        type=int,
        help='Override the number of repetitions for every configuration and bag.',
    )
    parser.add_argument(
        '--mode', choices=('unlabeled', 'human-labeled'), help='Override YAML mode.'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate bags/configurations and write a report without replaying.',
    )
    parser.add_argument(
        '--write-example-config',
        type=Path,
        metavar='PATH',
        help='Write a starter YAML file and exit without running evaluation.',
    )
    return parser


def run(args: argparse.Namespace) -> Path | None:
    """Resolve inputs, execute the matrix, aggregate, and write a report."""
    if args.write_example_config is not None:
        write_example_configuration(args.write_example_config)
        return None

    config_raw = args.config or QUICK_RUN_CONFIG
    if not config_raw:
        raise EvaluationError(
            'No evaluation YAML selected. Use --config or set QUICK_RUN_CONFIG. '
            'Use --write-example-config PATH to create a starter file.'
        )
    config_path = normalize_location(config_raw)
    document = load_yaml_mapping(config_path)
    settings = resolve_settings(args, document, config_path)
    experiments = resolve_experiments(document, config_path, args.trials)
    bag_paths = discover_bags(settings.bags_path)
    bags = [inspect_bag(path) for path in bag_paths]

    label_cache: dict[Path, tuple[list[dict[str, Any]], list[str]]] = {}
    if settings.mode == 'human-labeled':
        assert settings.human_label_topic is not None
        for bag in bags:
            label_cache[bag.path] = extract_human_labels(
                bag,
                settings.human_label_topic,
                settings.human_label_positive_regex,
            )

    expected = sum(experiment.trials for experiment in experiments) * len(bags)
    print(
        f'Preparing {expected} execution(s): {len(bags)} bag(s) × '
        f'{len(experiments)} configuration(s), mode={settings.mode}'
    )
    started = datetime.now(timezone.utc)
    revision = source_revision()
    report_path = reserve_report_path(settings.output_directory, started)
    grouped: dict[str, list[dict[str, Any]]] = {
        experiment.name: [] for experiment in experiments
    }
    collector: EvaluationCollector | None = None
    executor: SingleThreadedExecutor | None = None
    spin_thread: threading.Thread | None = None
    spin_stop = threading.Event()

    if not settings.dry_run:
        if ROS_IMPORT_ERROR is not None:
            raise EvaluationError(f'ROS 2 Python modules are unavailable: {ROS_IMPORT_ERROR}')
        rclpy.init()
        collector = EvaluationCollector(settings)
        executor = SingleThreadedExecutor()
        executor.add_node(collector)

        def spin() -> None:
            while not spin_stop.is_set():
                executor.spin_once(timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin, daemon=True)
        spin_thread.start()

    aborted = False
    interrupted = False

    def save_checkpoint(run_status: str) -> dict[str, Any]:
        """Persist every execution completed up to this point."""
        report = build_report(
            settings=settings,
            config_path=config_path,
            bags=bags,
            experiments=experiments,
            grouped=grouped,
            started=started,
            expected=expected,
            revision=revision,
            run_status=run_status,
        )
        write_report(report, report_path)
        return report

    # Write valid JSON before the first replay. Subsequent checkpoints replace
    # the same file atomically, so Ctrl+C leaves the last complete checkpoint.
    save_checkpoint('running')
    try:
        for experiment in experiments:
            print(f'Configuration: {experiment.name} ({experiment.config_hash[:12]})')
            for bag in bags:
                labels, label_warnings = label_cache.get(bag.path, ([], []))
                for trial in range(1, experiment.trials + 1):
                    print(f'  [{trial}/{experiment.trials}] {bag.path.name}')
                    if settings.dry_run:
                        # execute_one only uses collector after the dry-run return.
                        dummy_collector = None
                        execution = execute_one(
                            bag,
                            experiment,
                            trial,
                            settings,
                            dummy_collector,  # type: ignore[arg-type]
                            labels,
                            label_warnings,
                        )
                    else:
                        assert collector is not None
                        execution = execute_one(
                            bag,
                            experiment,
                            trial,
                            settings,
                            collector,
                            labels,
                            label_warnings,
                        )
                    grouped[experiment.name].append(execution)
                    save_checkpoint('running')
                    if execution['status'] in {'failed', 'invalid_bag'}:
                        print(f"    {execution['status']}: {execution['errors']}")
                        if not settings.continue_on_error:
                            aborted = True
                            break
                if aborted:
                    break
            if aborted:
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if collector is not None and executor is not None:
            spin_stop.set()
            if spin_thread is not None:
                spin_thread.join(timeout=5)
            executor.shutdown(timeout_sec=5)
            collector.destroy_node()
            rclpy.shutdown()

    if interrupted:
        run_status = 'interrupted'
    elif aborted:
        run_status = 'aborted'
    else:
        run_status = 'completed'
    report = save_checkpoint(run_status)
    all_executions = [
        execution
        for experiment in experiments
        for execution in grouped[experiment.name]
    ]
    overall = report['overall']
    print(f'\nOffline evaluation {run_status}')
    print(f'  Attempted: {len(all_executions)}/{expected}')
    print(f'  Completed: {overall["executions_completed"]}')
    print(f'  Failed:    {overall["executions_failed"]}')
    print(f'  Report:    {report_path}')
    if interrupted:
        raise EvaluationInterrupted(report_path)
    return report_path


def main(argv: Sequence[str] | None = None) -> int:
    """Command-line entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        run(args)
    except EvaluationInterrupted as exc:
        print(f'Interrupted. Partial report: {exc.report_path}', file=sys.stderr)
        return 130
    except EvaluationError as exc:
        parser.exit(2, f'error: {exc}\n')
    except KeyboardInterrupt:
        parser.exit(130, 'Interrupted.\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
