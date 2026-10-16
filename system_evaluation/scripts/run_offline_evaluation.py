#!/usr/bin/env python3
"""
Batch-replay ROS 2 bags through configurable anomaly-detection runs.

The runner supports two workflows:

* Pass paths and mode on the command line.
* Edit QUICK RUN SETTINGS below and click Run in an IDE.

Experiment options live in YAML. For every bag/configuration/trial combination,
the script creates an isolated resolved AAD configuration, starts a clean node,
replays the bag, collects decisions, and writes a detailed Markdown report.
Human-labeled mode reads schema-v1 ``*.annotations.json`` files stored beside
their bags; it does not require a label topic in the recording.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import os
import random
import re
import secrets
import shlex
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlparse
from urllib.request import Request, urlopen

import yaml


AUTO_SOURCE_MARKER = 'AAD_OFFLINE_EVALUATION_AUTOSOURCED'


def find_ros_setup() -> Path | None:
    """Find the active or locally installed ROS distribution setup script."""
    candidates: list[Path] = []
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        candidates.append(Path('/opt/ros') / ros_distro / 'setup.bash')
    candidates.append(Path('/opt/ros/jazzy/setup.bash'))
    candidates.extend(sorted(Path('/opt/ros').glob('*/setup.bash')))
    return next((path for path in candidates if path.is_file()), None)


def find_workspace_setup() -> Path | None:
    """Find the nearest workspace install setup above this source file."""
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'install' / 'setup.bash'
        if candidate.is_file():
            return candidate
    return None


def auto_source_environment() -> None:
    """Re-execute this CLI once inside its ROS and workspace environments."""
    if os.environ.get(AUTO_SOURCE_MARKER) == '1':
        return
    ros_setup = find_ros_setup()
    workspace_setup = find_workspace_setup()
    if ros_setup is None and workspace_setup is None:
        return
    command = (
        'set -e\n'
        'if [[ -n "$1" ]]; then source "$1"; fi\n'
        'if [[ -n "$2" ]]; then source "$2"; fi\n'
        f'export {AUTO_SOURCE_MARKER}=1\n'
        'exec "$3" "$4" "${@:5}"'
    )
    os.execv(
        '/bin/bash',
        [
            '/bin/bash',
            '-c',
            command,
            'run_offline_evaluation',
            str(ros_setup or ''),
            str(workspace_setup or ''),
            sys.executable,
            str(Path(__file__).resolve()),
            *sys.argv[1:],
        ],
    )


if __name__ == '__main__':
    auto_source_environment()

try:
    import rclpy
    import rosbag2_py
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError as exc:
    ROS_IMPORT_ERROR: ImportError | None = exc
    # Keep imports usable for configuration-only tests and provide a clear
    # error if auto-sourcing could not locate a working ROS environment.
    Node = object  # type: ignore[misc,assignment]
else:
    ROS_IMPORT_ERROR = None

try:
    from anomaly_detection.response_handler import (
        parse_llm_response as runtime_parse_llm_response,
    )
except ImportError:
    from response_handler import parse_llm_response as runtime_parse_llm_response


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
QUICK_RUN_MODE = 'human-labeled'  # Uses adjacent *.annotations.json sidecars.
QUICK_RUN_OUTPUT_DIRECTORY = (
    '/root/dev_ws/src/anomaly_detection/system_evaluation/results'
)


NS_PER_SECOND = 1_000_000_000
SCHEMA_VERSION = 13
ANNOTATION_SCHEMA_VERSION = 1
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
ARTIFACT_FILE_PATTERN = re.compile(r'^api_artifact_(?P<timestamp>\d+)\.json$')


class EvaluationError(RuntimeError):
    """Expected configuration, environment, or execution failure."""


class EvaluationStageError(EvaluationError):
    """Expected failure attributed to one evaluator lifecycle stage."""

    def __init__(
        self,
        message: str,
        *,
        kind: str,
        limit_seconds: float | None = None,
    ) -> None:
        super().__init__(message)
        self.kind = kind
        self.limit_seconds = limit_seconds


class EvaluationInterrupted(RuntimeError):
    """A user interruption that occurred after a partial report was saved."""

    def __init__(self, report_path: Path) -> None:
        """Record the path containing results completed before interruption."""
        super().__init__(f'Partial report saved to {report_path}')
        self.report_path = report_path


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
    randomization: dict[str, Any] | None = None


@dataclass(frozen=True)
class RunnerSettings:
    """Validated orchestration settings loaded from YAML and CLI."""

    mode: str
    bags_path: Path
    output_directory: Path
    provider_config_path: Path | None
    playback_rate: float
    ros_domain_id: int
    startup_timeout_seconds: float
    startup_grace_seconds: float
    playback_timeout_padding_seconds: float
    inference_drain_timeout_seconds: float
    inference_drain_model_passes: int
    post_playback_grace_seconds: float
    shutdown_timeout_seconds: float
    graph_clean_timeout_seconds: float
    model_preflight_enabled: bool
    model_preflight_timeout_seconds: float
    label_buffer_seconds: float
    semantic_judge_enabled: bool
    continue_on_error: bool
    detector_command: tuple[str, ...]
    decision_topic: str
    alert_topic: str
    llm_called_topic: str
    formatted_topic: str
    dry_run: bool


@dataclass
class FlowEdge:
    """Residual edge for dependency-free bipartite matching."""

    target: int
    reverse: int
    capacity: int
    cost: int


@dataclass
class CollectorState:
    """Mutable callback state protected by a condition lock."""

    decision_messages: list[str] = field(default_factory=list)
    alert_count: int = 0
    llm_calls: list[dict[str, Any]] = field(default_factory=list)
    last_activity_ns: int = field(default_factory=time.monotonic_ns)


def decision_from_llm_response(response: Any) -> dict[str, Any]:
    """Convert the detector's shared response type into a report mapping."""
    decision = runtime_parse_llm_response(response)
    return {
        'anomaly': decision.anomaly,
        'severity': decision.severity,
        'action': decision.action,
        'summary': decision.summary,
    }


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


def ros_domain_id(value: Any) -> int:
    """Validate a ROS 2 domain ID accepted by common DDS implementations."""
    if isinstance(value, bool):
        raise EvaluationError('runner.ros_domain_id must be an integer from 0 to 232.')
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(
            'runner.ros_domain_id must be an integer from 0 to 232.'
        ) from exc
    if parsed != value or not 0 <= parsed <= 232:
        raise EvaluationError('runner.ros_domain_id must be an integer from 0 to 232.')
    return parsed


def random_seed(name: str, value: Any) -> int:
    """Validate an optional randomization seed or generate a secure one."""
    if value is None:
        return secrets.randbits(63)
    if isinstance(value, bool):
        raise EvaluationError(f'{name} must be an integer.')
    try:
        seed = int(value)
    except (TypeError, ValueError) as exc:
        raise EvaluationError(f'{name} must be an integer.') from exc
    if seed != value:
        raise EvaluationError(f'{name} must be an integer.')
    return seed


def randomized_llm_value(
    name: str, specification: Any, generator: random.Random
) -> Any:
    """Select one LLM value from a choices list or a numeric range."""
    if isinstance(specification, list):
        choices = specification
        range_specification = None
    elif isinstance(specification, dict):
        choices = specification.get('choices')
        range_specification = specification.get('range')
        unknown = set(specification).difference({'choices', 'range', 'integer'})
        if unknown:
            raise EvaluationError(
                f'{name} has unsupported randomization field(s): '
                f'{", ".join(sorted(unknown))}.'
            )
        if choices is not None and range_specification is not None:
            raise EvaluationError(f'{name} cannot specify both choices and range.')
    else:
        raise EvaluationError(
            f'{name} must be a choices list or a mapping with choices or range.'
        )

    if choices is not None:
        if not isinstance(choices, list) or not choices:
            raise EvaluationError(f'{name}.choices must be a non-empty list.')
        return copy.deepcopy(generator.choice(choices))

    if not isinstance(range_specification, list) or len(range_specification) != 2:
        raise EvaluationError(f'{name}.range must contain [minimum, maximum].')
    minimum, maximum = range_specification
    if (
        not isinstance(minimum, (int, float))
        or isinstance(minimum, bool)
        or not isinstance(maximum, (int, float))
        or isinstance(maximum, bool)
        or not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or minimum > maximum
    ):
        raise EvaluationError(f'{name}.range must be finite numbers in ascending order.')
    integer = specification.get('integer') if isinstance(specification, dict) else None
    if integer is None:
        integer = isinstance(minimum, int) and isinstance(maximum, int)
    if not isinstance(integer, bool):
        raise EvaluationError(f'{name}.integer must be boolean when provided.')
    if integer:
        if int(minimum) != minimum or int(maximum) != maximum:
            raise EvaluationError(f'{name}.range must use integer endpoints.')
        return generator.randint(int(minimum), int(maximum))
    return generator.uniform(float(minimum), float(maximum))


def validate_llm_configuration(
    experiment_name: str, aad_config: dict[str, Any]
) -> None:
    """Check the LLM fields required by the evaluator before a replay."""
    llm = aad_config.get('llm')
    if not isinstance(llm, dict):
        raise EvaluationError(
            f'Experiment {experiment_name!r} has no valid llm mapping.'
        )
    if not isinstance(llm.get('model'), str) or not llm['model'].strip():
        raise EvaluationError(
            f'Experiment {experiment_name!r} must select llm.model.'
        )
    if 'local' in llm and not isinstance(llm['local'], bool):
        raise EvaluationError(
            f'Experiment {experiment_name!r} llm.local must be boolean.'
        )
    if 'vision_enabled' in llm and not isinstance(llm['vision_enabled'], bool):
        raise EvaluationError(
            f'Experiment {experiment_name!r} llm.vision_enabled must be boolean.'
        )


def randomized_experiments(
    *,
    name: str,
    aad_config: dict[str, Any],
    trials: int,
    specification: Any,
) -> list[Experiment]:
    """Expand one randomized experiment into reproducible sampled variants."""
    if not isinstance(specification, dict):
        raise EvaluationError(f'Experiment {name!r} randomize must be a mapping.')
    unknown = set(specification).difference({'samples', 'seed', 'llm'})
    if unknown:
        raise EvaluationError(
            f'Experiment {name!r} randomize has unsupported field(s): '
            f'{", ".join(sorted(unknown))}.'
        )
    samples = positive_integer(
        f'experiments[{name}].randomize.samples', specification.get('samples', 1)
    )
    llm_specification = specification.get('llm')
    if not isinstance(llm_specification, dict) or not llm_specification:
        raise EvaluationError(
            f'Experiment {name!r} randomize.llm must be a non-empty mapping.'
        )
    if not all(
        isinstance(parameter, str) and parameter for parameter in llm_specification
    ):
        raise EvaluationError(
            f'Experiment {name!r} randomize.llm parameter names must be strings.'
        )

    seed = random_seed(
        f'experiments[{name}].randomize.seed', specification.get('seed')
    )
    generator = random.Random(seed)
    resolved: list[Experiment] = []
    hashes: set[str] = set()
    attempts = 0
    maximum_attempts = max(samples * 100, 100)
    while len(resolved) < samples and attempts < maximum_attempts:
        attempts += 1
        sampled = {
            parameter: randomized_llm_value(
                f'experiments[{name}].randomize.llm.{parameter}', value, generator
            )
            for parameter, value in llm_specification.items()
        }
        candidate = copy.deepcopy(aad_config)
        candidate_llm = candidate.get('llm')
        assert isinstance(candidate_llm, dict)
        candidate_llm.update(sampled)
        validate_llm_configuration(name, candidate)
        config_hash = stable_hash(candidate)
        if config_hash in hashes:
            continue
        sample_index = len(resolved) + 1
        resolved.append(
            Experiment(
                name=f'{name}__sample_{sample_index}',
                aad_config=candidate,
                config_hash=config_hash,
                trials=trials,
                randomization={
                    'source_experiment': name,
                    'sample_index': sample_index,
                    'sample_count': samples,
                    'seed': seed,
                    'sampled_llm_parameters': sampled,
                },
            )
        )
        hashes.add(config_hash)
    if len(resolved) != samples:
        raise EvaluationError(
            f'Experiment {name!r} could not generate {samples} unique randomized '
            'LLM configurations. Add more choices, widen a range, or request fewer '
            'samples.'
        )
    return resolved


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
    declared_names = {
        str(raw.get('name', '')).strip()
        for raw in raw_experiments
        if isinstance(raw, dict)
    }
    names: set[str] = set()
    resolved_names: set[str] = set()
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
        validate_llm_configuration(name, aad_config)
        trial_value = (
            trials_override
            if trials_override is not None
            else raw.get('trials', default_trials)
        )
        trials = positive_integer(f'experiments[{name}].trials', trial_value)
        randomize = raw.get('randomize')
        if randomize is None:
            experiments = [Experiment(name, aad_config, stable_hash(aad_config), trials)]
        else:
            experiments = randomized_experiments(
                name=name,
                aad_config=aad_config,
                trials=trials,
                specification=randomize,
            )
        for experiment in experiments:
            if experiment.name in resolved_names or (
                randomize is not None and experiment.name in declared_names
            ):
                raise EvaluationError(
                    f'Randomized experiment name conflicts with another experiment: '
                    f'{experiment.name!r}.'
                )
            resolved_names.add(experiment.name)
            resolved.append(experiment)
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
    provider_config_value = (
        args.provider_config
        if args.provider_config is not None
        else yaml_runner.get('provider_config')
    )
    provider_config_path = (
        normalize_location(str(provider_config_value), config_path.parent)
        if provider_config_value
        else None
    )
    if provider_config_path is not None and not provider_config_path.is_file():
        raise EvaluationError(
            f'LLM provider config does not exist: {provider_config_path}'
        )
    mode = str(
        runner_value(args.mode, yaml_runner, 'mode', QUICK_RUN_MODE, 'unlabeled')
    ).strip()
    if mode not in {'unlabeled', 'human-labeled'}:
        raise EvaluationError('mode must be unlabeled or human-labeled.')

    detector_command = command_value(
        yaml_runner.get(
            'detector_command',
            ['ros2', 'run', 'anomaly_detection', 'anomaly_detection_node'],
        )
    )
    semantic_judge_enabled = coerce_bool(
        yaml_runner.get('semantic_judge_enabled', False)
    )
    if semantic_judge_enabled is None:
        raise EvaluationError('runner.semantic_judge_enabled must be boolean.')
    model_preflight_enabled = coerce_bool(
        yaml_runner.get('model_preflight_enabled', True)
    )
    if model_preflight_enabled is None:
        raise EvaluationError('runner.model_preflight_enabled must be boolean.')
    return RunnerSettings(
        mode=mode,
        bags_path=bags_path,
        output_directory=output_directory,
        provider_config_path=provider_config_path,
        playback_rate=positive_number(
            'runner.playback_rate', yaml_runner.get('playback_rate', 1.0)
        ),
        ros_domain_id=ros_domain_id(yaml_runner.get('ros_domain_id', 42)),
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
            yaml_runner.get('inference_drain_timeout_seconds', 180.0),
        ),
        inference_drain_model_passes=positive_integer(
            'runner.inference_drain_model_passes',
            yaml_runner.get('inference_drain_model_passes', 2),
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
        graph_clean_timeout_seconds=positive_number(
            'runner.graph_clean_timeout_seconds',
            yaml_runner.get('graph_clean_timeout_seconds', 5.0),
            allow_zero=True,
        ),
        model_preflight_enabled=model_preflight_enabled,
        model_preflight_timeout_seconds=positive_number(
            'runner.model_preflight_timeout_seconds',
            yaml_runner.get('model_preflight_timeout_seconds', 120.0),
        ),
        label_buffer_seconds=positive_number(
            'runner.label_buffer_seconds',
            yaml_runner.get('label_buffer_seconds', 5.0),
            allow_zero=True,
        ),
        semantic_judge_enabled=semantic_judge_enabled,
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


def annotation_path_for_bag(path: Path) -> Path:
    """Resolve the annotation sidecar stored beside a standalone or split bag."""
    if path.is_file():
        return path.with_name(f'{path.stem}.annotations.json')

    mcap_candidates = sorted(path.glob('*.mcap'))
    expected = [
        mcap.with_name(f'{mcap.stem}.annotations.json') for mcap in mcap_candidates
    ]
    expected.append(path / f'{path.name}.annotations.json')
    expected.append(path.parent / f'{path.name}.annotations.json')
    matches = list(
        dict.fromkeys(candidate for candidate in expected if candidate.is_file())
    )
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise EvaluationError(
            f'Multiple annotation sidecars match bag {path}: '
            + ', '.join(str(candidate) for candidate in matches)
        )

    direct = sorted(path.glob('*.annotations.json'))
    if len(direct) == 1:
        return direct[0]
    if len(direct) > 1:
        raise EvaluationError(
            f'Multiple annotation sidecars are present in {path}: '
            + ', '.join(str(candidate) for candidate in direct)
        )
    return expected[0] if mcap_candidates else expected[-2]


def annotation_timestamp(
    value: Any, field_name: str, errors: list[str]
) -> int | None:
    """Validate one schema-v1 nanosecond timestamp without losing precision."""
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        errors.append(f'{field_name} must be a non-negative integer.')
        return None
    return value


def annotation_interval(label: dict[str, Any]) -> tuple[int, int]:
    """Return the inclusive time interval represented by a truth label."""
    timestamp = int(label['timestamp_ns'])
    return (
        int(label.get('start_timestamp_ns', timestamp)),
        int(label.get('end_timestamp_ns', timestamp)),
    )


def load_annotation_ground_truth(bag: BagInfo) -> dict[str, Any]:
    """Load and validate the schema-v1 annotation sidecar adjacent to a bag."""
    warnings: list[str] = []
    errors: list[str] = []
    try:
        path = annotation_path_for_bag(bag.path)
    except EvaluationError as exc:
        return {
            'source': None,
            'bag_anomalous': None,
            'labels': [],
            'ignored_intervals': [],
            'warnings': warnings,
            'errors': [str(exc)],
        }
    if not path.is_file():
        return {
            'source': str(path),
            'bag_anomalous': None,
            'labels': [],
            'ignored_intervals': [],
            'warnings': warnings,
            'errors': [f'Required annotation sidecar is missing: {path}'],
        }
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            'source': str(path),
            'bag_anomalous': None,
            'labels': [],
            'ignored_intervals': [],
            'warnings': warnings,
            'errors': [f'Could not read annotation sidecar {path}: {exc}'],
        }
    if not isinstance(document, dict):
        errors.append('Annotation document must be a JSON object.')
        document = {}
    if document.get('schema_version') != ANNOTATION_SCHEMA_VERSION:
        errors.append(
            f'Expected annotation schema_version {ANNOTATION_SCHEMA_VERSION}.'
        )

    bag_metadata = document.get('bag')
    if not isinstance(bag_metadata, dict):
        errors.append('bag must be an object.')
        bag_metadata = {}
    annotated_file = bag_metadata.get('file')
    valid_names = {bag.path.name}
    if bag.path.is_dir():
        valid_names.update(mcap.name for mcap in bag.path.glob('*.mcap'))
    if not isinstance(annotated_file, str) or not annotated_file.strip():
        errors.append('bag.file must be a non-empty string.')
    elif Path(annotated_file).name not in valid_names:
        errors.append(
            f'Annotation bag.file {annotated_file!r} does not match {bag.path}.'
        )
    bag_anomalous = bag_metadata.get('anomalous')
    if not isinstance(bag_anomalous, bool):
        errors.append(
            'bag.anomalous must be true or false before the sidecar can be evaluated.'
        )
        bag_anomalous = None

    ranges = document.get('ranges')
    annotations = document.get('annotations')
    if not isinstance(ranges, list):
        errors.append('ranges must be an array.')
        ranges = []
    if not isinstance(annotations, list):
        errors.append('annotations must be an array.')
        annotations = []

    labels: list[dict[str, Any]] = []
    ignored_intervals: list[dict[str, Any]] = []
    for index, item in enumerate(ranges):
        field = f'ranges[{index}]'
        if not isinstance(item, dict):
            errors.append(f'{field} must be an object.')
            continue
        start = annotation_timestamp(
            item.get('start_timestamp_ns'), f'{field}.start_timestamp_ns', errors
        )
        end = annotation_timestamp(
            item.get('end_timestamp_ns'), f'{field}.end_timestamp_ns', errors
        )
        if start is None or end is None:
            continue
        if end < start:
            errors.append(f'{field} ends before it starts.')
            continue
        label = item.get('label')
        if label not in {'unlabeled', 'normal', 'anomaly', 'uncertain', 'ignore'}:
            errors.append(f'{field}.label is invalid: {label!r}.')
            continue
        record = {
            'id': f'{path.name}:range:{index + 1}',
            'timestamp_ns': start,
            'start_timestamp_ns': start,
            'end_timestamp_ns': end,
            'timestamp_source': 'annotation_range',
            'category': item.get('category', ''),
            'description': item.get('description', ''),
        }
        if label == 'anomaly':
            labels.append(record)
        elif label in {'unlabeled', 'uncertain', 'ignore'}:
            ignored_intervals.append(record)

    anomaly_ranges = [annotation_interval(label) for label in labels]
    for index, item in enumerate(annotations):
        field = f'annotations[{index}]'
        if not isinstance(item, dict):
            errors.append(f'{field} must be an object.')
            continue
        timestamp = annotation_timestamp(
            item.get('recorded_timestamp_ns'),
            f'{field}.recorded_timestamp_ns',
            errors,
        )
        if timestamp is None:
            continue
        label = item.get('label')
        if label not in {'unlabeled', 'normal', 'anomaly', 'uncertain', 'ignore'}:
            errors.append(f'{field}.label is invalid: {label!r}.')
            continue
        record = {
            'id': f'{path.name}:message:{index + 1}',
            'timestamp_ns': timestamp,
            'timestamp_source': 'annotation_message',
            'message_index': item.get('message_index'),
            'topic': item.get('topic'),
            'node_name': item.get('node_name'),
            'category': item.get('category', ''),
            'description': item.get('description', ''),
        }
        if label == 'anomaly':
            if not any(start <= timestamp <= end for start, end in anomaly_ranges):
                labels.append(record)
        elif label in {'uncertain', 'ignore'}:
            ignored_intervals.append(record)

    if bag_anomalous is False and labels:
        errors.append(
            'bag.anomalous is false but the sidecar contains anomaly labels.'
        )
    if bag_anomalous is True and not labels:
        if bag.start_ns is None or bag.end_ns is None:
            errors.append(
                'The anomalous bag has no anomaly ranges/messages and its time '
                'bounds are unavailable.'
            )
        else:
            labels.append(
                {
                    'id': f'{path.name}:bag',
                    'timestamp_ns': bag.start_ns,
                    'start_timestamp_ns': bag.start_ns,
                    'end_timestamp_ns': bag.end_ns,
                    'timestamp_source': 'annotation_bag_fallback',
                    'category': bag_metadata.get('category', ''),
                    'description': bag_metadata.get('description', ''),
                }
            )
            warnings.append(
                'No anomaly ranges or messages were labeled; the full anomalous '
                'bag is treated as one truth interval.'
            )
    return {
        'source': str(path),
        'bag_anomalous': bag_anomalous,
        'bag_metadata': bag_metadata,
        'labels': labels,
        'ignored_intervals': ignored_intervals,
        'warnings': warnings,
        'errors': errors,
    }


class EvaluationCollector(Node):
    """Collect detector calls, published decisions, and alert counts."""

    def __init__(self, settings: RunnerSettings) -> None:
        """Subscribe to the detector topics needed for one evaluation run."""
        super().__init__('offline_evaluation_collector')
        self.settings = settings
        self.state = CollectorState()
        self.condition = threading.Condition()
        self.collection_active = False
        self.create_subscription(String, settings.decision_topic, self._decision, 50)
        self.create_subscription(String, settings.alert_topic, self._alert, 50)
        self.create_subscription(Bool, settings.llm_called_topic, self._llm_called, 50)

    def reset(self) -> None:
        """Clear all run-specific state."""
        with self.condition:
            self.state = CollectorState()
            self.collection_active = False
            self.condition.notify_all()

    def begin_collection(self) -> None:
        """Start a fresh accounting window immediately before bag playback."""
        with self.condition:
            self.state = CollectorState()
            self.collection_active = True
            self.condition.notify_all()

    def end_collection(self) -> None:
        """Ignore late callbacks after the run snapshot has been captured."""
        with self.condition:
            self.collection_active = False
            self.condition.notify_all()

    def _touch(self) -> None:
        self.state.last_activity_ns = time.monotonic_ns()

    def _llm_called(self, _message: Bool) -> None:
        with self.condition:
            if not self.collection_active:
                return
            self._touch()
            self.state.llm_calls.append(
                {'received_at_utc_ns': time.time_ns()}
            )
            self.condition.notify_all()

    def _decision(self, message: String) -> None:
        with self.condition:
            if not self.collection_active:
                return
            self._touch()
            self.state.decision_messages.append(message.data)
            self.condition.notify_all()

    def _alert(self, _message: String) -> None:
        with self.condition:
            if not self.collection_active:
                return
            self._touch()
            self.state.alert_count += 1
            self.condition.notify_all()

    def wait_for_drain(
        self,
        minimum_wait: float,
        timeout: float,
        artifact_directory: Path,
    ) -> bool:
        """Wait until every observed model call has produced an API artifact."""
        started = time.monotonic()
        deadline = started + timeout
        quiet_ns = int(0.5 * NS_PER_SECOND)
        with self.condition:
            while True:
                now = time.monotonic()
                quiet = time.monotonic_ns() - self.state.last_activity_ns >= quiet_ns
                waited = now - started >= minimum_wait
                completed_calls = sum(
                    1 for _ in artifact_directory.glob('api_artifact_*.json')
                )
                pending_calls = max(len(self.state.llm_calls) - completed_calls, 0)
                if waited and quiet and pending_calls == 0:
                    return True
                remaining = deadline - now
                if remaining <= 0:
                    return False
                self.condition.wait(timeout=min(0.2, remaining))

    @staticmethod
    def _graph_endpoint(endpoint: Any, role: str, topic: str) -> dict[str, str]:
        """Convert ROS graph endpoint data into stable diagnostic fields."""
        return {
            'role': role,
            'topic': topic,
            'node_name': str(getattr(endpoint, 'node_name', 'unknown')),
            'node_namespace': str(getattr(endpoint, 'node_namespace', '/')),
        }

    def graph_contamination(self, raw_input_topic: str) -> list[dict[str, str]]:
        """Find detector endpoints that existed before this run starts."""
        endpoints: list[dict[str, str]] = []
        for topic in (
            self.settings.decision_topic,
            self.settings.alert_topic,
            self.settings.llm_called_topic,
            self.settings.formatted_topic,
        ):
            endpoints.extend(
                self._graph_endpoint(endpoint, 'publisher', topic)
                for endpoint in self.get_publishers_info_by_topic(topic)
            )
        endpoints.extend(
            self._graph_endpoint(endpoint, 'subscription', raw_input_topic)
            for endpoint in self.get_subscriptions_info_by_topic(raw_input_topic)
        )
        endpoints.extend(
            self._graph_endpoint(endpoint, 'publisher', raw_input_topic)
            for endpoint in self.get_publishers_info_by_topic(raw_input_topic)
        )
        return endpoints

    def wait_for_clean_graph(
        self,
        raw_input_topic: str,
        timeout: float,
    ) -> list[dict[str, str]]:
        """Allow stale DDS discovery entries to clear, then return contaminants."""
        deadline = time.monotonic() + timeout
        while True:
            endpoints = self.graph_contamination(raw_input_topic)
            if not endpoints or time.monotonic() >= deadline:
                return endpoints
            time.sleep(0.1)

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
                'decision_messages': list(self.state.decision_messages),
                'alert_count': self.state.alert_count,
                'llm_calls': copy.deepcopy(self.state.llm_calls),
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


def stop_process(process: subprocess.Popen[Any] | None, timeout: float) -> bool:
    """Stop a process group and return whether a forced kill was required."""
    if process is None or process.poll() is not None:
        return False
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return False
    try:
        process.wait(timeout=timeout)
        return False
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
        return True


def read_artifacts(directory: Path) -> list[dict[str, Any]]:
    """Load generated API artifacts and retain malformed-file diagnostics."""
    artifacts: list[dict[str, Any]] = []
    for path in sorted(directory.glob('api_artifact_*.json')):
        match = ARTIFACT_FILE_PATTERN.match(path.name)
        file_timestamp = int(match.group('timestamp')) if match else None
        try:
            file_mtime = path.stat().st_mtime_ns
        except OSError:
            file_mtime = None
        try:
            payload = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(payload, dict):
                raise ValueError('artifact root is not an object')
            payload['_file'] = path.name
            payload['_file_timestamp_ns'] = file_timestamp
            payload['_file_mtime_ns'] = file_mtime
            artifacts.append(payload)
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            artifacts.append(
                {
                    '_file': path.name,
                    '_file_timestamp_ns': file_timestamp,
                    '_file_mtime_ns': file_mtime,
                    '_parse_error': str(exc),
                }
            )
    return artifacts


def artifact_timestamp_ns(artifact: dict[str, Any]) -> int | None:
    """Return an artifact completion timestamp from content or filename."""
    for key in ('_file_mtime_ns', 'timestamp_ns', '_file_timestamp_ns'):
        value = artifact.get(key)
        if isinstance(value, int) and not isinstance(value, bool):
            return value
    return None


def match_artifacts_to_calls(
    artifacts: list[dict[str, Any]], llm_calls: list[dict[str, Any]]
) -> dict[int, dict[str, Any]]:
    """Match each artifact to the nearest preceding unmatched model call."""
    unmatched = set(range(len(llm_calls)))
    matches: dict[int, dict[str, Any]] = {}
    artifact_order = sorted(
        range(len(artifacts)),
        key=lambda index: (
            artifact_timestamp_ns(artifacts[index]) is None,
            artifact_timestamp_ns(artifacts[index]) or index,
        ),
    )
    for artifact_index in artifact_order:
        if not unmatched:
            break
        artifact_time = artifact_timestamp_ns(artifacts[artifact_index])
        preceding = [
            call_index
            for call_index in unmatched
            if artifact_time is not None
            and isinstance(llm_calls[call_index].get('received_at_utc_ns'), int)
            and not isinstance(
                llm_calls[call_index].get('received_at_utc_ns'), bool
            )
            and llm_calls[call_index]['received_at_utc_ns'] <= artifact_time
        ]
        if preceding:
            call_index = max(
                preceding,
                key=lambda index: llm_calls[index]['received_at_utc_ns'],
            )
        else:
            call_index = min(
                unmatched,
                key=lambda index: (
                    llm_calls[index].get('received_at_utc_ns') is None,
                    llm_calls[index].get('received_at_utc_ns') or index,
                ),
            )
        unmatched.remove(call_index)
        matches[artifact_index] = llm_calls[call_index]
    return matches


def llm_decisions_from_artifacts(
    artifacts: list[dict[str, Any]], llm_calls: list[dict[str, Any]]
) -> tuple[list[dict[str, Any]], list[str]]:
    """Build exactly one report decision for each nonempty model response."""
    warnings: list[str] = []
    call_matches = match_artifacts_to_calls(artifacts, llm_calls)
    decisions: list[dict[str, Any]] = []
    for index, artifact in enumerate(artifacts):
        if '_parse_error' in artifact:
            warnings.append(
                f"Could not parse LLM artifact {artifact.get('_file', index + 1)}: "
                f"{artifact['_parse_error']}"
            )
            continue
        response = artifact.get('api_response')
        if not isinstance(response, str) or not response.strip():
            warnings.append(
                f"LLM artifact {artifact.get('_file', index + 1)} contains no model "
                'response; it was excluded from the LLM-decision table.'
            )
            continue
        decision = decision_from_llm_response(response)
        contexts = artifact.get('cached_data', [])
        context_messages = (
            [str(context) for context in contexts]
            if isinstance(contexts, list)
            else []
        )
        error_messages = [
            context
            for context in context_messages
            if re.search(r'\bimportance\s*=\s*ERROR\b', context, re.IGNORECASE)
        ]
        decision.update(
            {
                'artifact_api_response': response,
                'error_context_messages': error_messages,
                'source_timestamp_candidates_ns': [],
                'model_latency_seconds': None,
            }
        )
        if context_messages:
            candidates = sorted(
                {
                    timestamp
                    for context in context_messages
                    for timestamp in embedded_timestamps(context)
                }
            )
            if candidates:
                decision['source_timestamp_candidates_ns'] = candidates
        timestamp = artifact_timestamp_ns(artifact)
        if timestamp is not None:
            matched_call = call_matches.get(index)
            if matched_call is not None:
                call_timestamp = matched_call.get('received_at_utc_ns')
                if isinstance(call_timestamp, int) and timestamp >= call_timestamp:
                    decision['model_latency_seconds'] = (
                        timestamp - call_timestamp
                    ) / NS_PER_SECOND
        decisions.append(decision)
    if len(artifacts) != len(llm_calls):
        warnings.append(
            'Artifact-file/LLM-call counts differ; only captured model responses '
            f'were reported (artifacts={len(artifacts)}, calls={len(llm_calls)}).'
        )
    return decisions, warnings


def decision_signature(decision: dict[str, Any]) -> tuple[Any, ...]:
    """Return stable fields used to identify a published model decision."""
    return (
        decision.get('anomaly'),
        str(decision.get('severity') or '').strip().lower(),
        str(decision.get('action') or '').strip().lower(),
        ' '.join(str(decision.get('summary') or '').split()),
    )


def immediate_decisions_from_topic(
    messages: list[str], llm_decisions: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Exclude artifact-backed model decisions from published stop decisions."""
    remaining_llm = Counter(decision_signature(item) for item in llm_decisions)
    immediate: list[dict[str, Any]] = []
    for message in messages:
        parsed = parse_decision(message)
        signature = decision_signature(parsed)
        if remaining_llm[signature]:
            remaining_llm[signature] -= 1
            continue
        if (
            parsed.get('anomaly') is True
            and parsed.get('severity') == 'high'
            and parsed.get('action') == 'stop_cart'
        ):
            immediate.append(parsed)
    return immediate


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
    ignored_intervals: list[dict[str, Any]] | None = None,
    bag_anomalous: bool | None = None,
) -> dict[str, Any]:
    """Match positive decisions to annotated points or inclusive time ranges."""
    positives = [decision for decision in decisions if decision.get('anomaly') is True]
    scorable: list[dict[str, Any]] = []
    unscorable: list[dict[str, Any]] = []
    ignored: list[dict[str, Any]] = []
    excluded = [annotation_interval(item) for item in (ignored_intervals or [])]
    for decision in positives:
        candidates = decision.get('source_timestamp_candidates_ns')
        if not candidates:
            unscorable.append(decision)
        elif excluded and all(
            any(start <= int(candidate) <= end for start, end in excluded)
            for candidate in candidates
        ):
            ignored.append(decision)
        else:
            scorable.append(decision)

    buffer_ns = int(round(buffer_seconds * NS_PER_SECOND))
    costs: dict[tuple[int, int], int] = {}
    chosen_sources: dict[tuple[int, int], int] = {}
    for label_index, label in enumerate(labels):
        start, end = annotation_interval(label)
        for decision_index, decision in enumerate(scorable):
            def distance_from_label(value: int) -> int:
                if value < start:
                    return start - value
                if value > end:
                    return value - end
                return 0

            selected = min(
                decision['source_timestamp_candidates_ns'],
                key=lambda value: (distance_from_label(int(value)), int(value)),
            )
            distance = distance_from_label(int(selected))
            if distance <= buffer_ns:
                costs[(label_index, decision_index)] = distance
                chosen_sources[(label_index, decision_index)] = int(selected)
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
        label_start, label_end = annotation_interval(label)
        if source_time < label_start:
            event_difference_ns = source_time - label_start
        elif source_time > label_end:
            event_difference_ns = source_time - label_end
        else:
            event_difference_ns = 0
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
            source_after_label = (source_time - label_start) / NS_PER_SECOND
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
                'event_difference_seconds': event_difference_ns / NS_PER_SECOND,
                'processing_latency_seconds': processing,
                'label_relative_replay_latency_seconds': label_relative,
                'attribution_method': 'closest_source_timestamp_to_annotation',
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
        'ignored_positive_decisions': len(ignored),
        'detection_accuracy': safe_ratio(true_positives, len(labels)),
        'precision': safe_ratio(true_positives, true_positives + false_positives),
        'recall': safe_ratio(true_positives, true_positives + false_negatives),
        'f1': safe_ratio(
            2 * true_positives,
            2 * true_positives + false_positives + false_negatives,
        ),
        'complete': not unscorable,
    }
    predicted_anomalous = bool(scorable or unscorable)
    bag_outcome = None
    if bag_anomalous is not None:
        bag_outcome = (
            'true_positive'
            if bag_anomalous and predicted_anomalous
            else 'false_negative'
            if bag_anomalous
            else 'false_positive'
            if predicted_anomalous
            else 'true_negative'
        )
    return {
        'source': 'annotation_sidecar',
        'bag_anomalous': bag_anomalous,
        'label_count': len(labels),
        'ignored_interval_count': len(ignored_intervals or []),
        'bag_classification': {
            'expected_anomalous': bag_anomalous,
            'detected_anomalous': predicted_anomalous,
            'result': bag_outcome,
        },
        'metrics': metrics,
        'processing_latency': latency_summary(processing_latencies),
        'label_relative_latency': latency_summary(label_latencies),
        'events': events,
        'unscorable_decisions': unscorable,
        'ignored_decisions': ignored,
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
    severities = Counter(
        decision.get('severity')
        for decision in decisions
        if decision.get('severity')
    )
    actions = Counter(
        decision.get('action')
        for decision in decisions
        if decision.get('action')
    )
    return {
        'elapsed_seconds': elapsed_seconds,
        'model_response_latency': latency_summary(latencies),
        'behavior': {
            'decision_count': len(decisions),
            'immediate_safety_decisions': len(
                snapshot.get('immediate_decisions', [])
            ),
            'positive_decisions': positive_count,
            'negative_decisions': negative_count,
            'unparseable_decisions': unparseable,
            'positive_decision_rate': safe_ratio(positive_count, len(decisions)),
            'alert_count': snapshot['alert_count'],
            'severity_distribution': dict(sorted(severities.items())),
            'action_distribution': dict(sorted(actions.items())),
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


def configured_model_timeout(configuration: dict[str, Any]) -> float:
    """Return the detector request timeout using its runtime-compatible default."""
    llm = configuration.get('llm', {})
    value = llm.get('timeout_seconds', 30.0) if isinstance(llm, dict) else 30.0
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 30.0
    return parsed if math.isfinite(parsed) and parsed > 0 else 30.0


def effective_inference_drain_timeout(
    configuration: dict[str, Any],
    settings: RunnerSettings,
) -> float:
    """Allow the configured number of serial model passes plus shutdown margin."""
    calculated = (
        configured_model_timeout(configuration)
        * settings.inference_drain_model_passes
        + settings.shutdown_timeout_seconds
    )
    return max(settings.inference_drain_timeout_seconds, calculated)


def ollama_candidates(configuration: dict[str, Any]) -> list[str]:
    """Return de-duplicated Ollama endpoints in detector preference order."""
    llm = configuration.get('llm', {})
    if not isinstance(llm, dict):
        return []
    configured_hosts = llm.get('ollama_hosts', [])
    extras = configured_hosts if isinstance(configured_hosts, list) else []
    values = [
        llm.get('ollama_host', 'http://127.0.0.1:11434'),
        *extras,
        os.environ.get('OLLAMA_HOST'),
        'http://127.0.0.1:11434',
        'http://127.0.0.1:21434',
    ]
    result = []
    for value in values:
        host = str(value or '').strip().rstrip('/')
        if host and host not in result:
            result.append(host)
    return result


def ollama_models(host: str, timeout: float = 2.0) -> set[str]:
    """Read model names from one Ollama tags endpoint."""
    request = Request(f'{host}/api/tags', method='GET')
    with urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode('utf-8'))
    models = payload.get('models', []) if isinstance(payload, dict) else []
    names = set()
    for model in models if isinstance(models, list) else []:
        if not isinstance(model, dict):
            continue
        name = model.get('model') or model.get('name')
        if name:
            names.add(str(name))
    return names


def preflight_model(
    configuration: dict[str, Any],
    settings: RunnerSettings,
) -> dict[str, Any]:
    """Verify and warm a local model before detector startup and bag playback."""
    llm = configuration.get('llm', {})
    if not settings.model_preflight_enabled:
        return {'status': 'disabled'}
    if not isinstance(llm, dict) or not bool(llm.get('local', False)):
        return {
            'status': 'skipped',
            'reason': 'Evaluator warm-up currently applies to local Ollama models.',
        }
    model = str(llm.get('model', '')).strip()
    if not model:
        raise EvaluationStageError(
            'Model preflight failed: llm.model is empty.',
            kind='model_preflight',
        )
    failures = []
    selected_host = None
    for host in ollama_candidates(configuration):
        try:
            if model in ollama_models(host):
                selected_host = host
                break
            failures.append(f'{host}: model not installed')
        except (HTTPError, URLError, OSError, TimeoutError, json.JSONDecodeError) as exc:
            failures.append(f'{host}: {type(exc).__name__}: {exc}')
    if selected_host is None:
        raise EvaluationStageError(
            'Model preflight failed; no reachable Ollama endpoint contains '
            f'{model!r}. Checked: {"; ".join(failures)}',
            kind='model_preflight',
        )

    payload = {
        'model': model,
        'messages': [{'role': 'user', 'content': 'ping'}],
        'stream': False,
        'think': False,
        'keep_alive': llm.get('keep_alive', '2m'),
        'options': {'temperature': 0, 'num_predict': 1},
    }
    request = Request(
        f'{selected_host}/api/chat',
        data=json.dumps(payload).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    started = time.monotonic()
    try:
        with urlopen(
            request,
            timeout=settings.model_preflight_timeout_seconds,
        ) as response:
            response.read()
    except (HTTPError, URLError, OSError, TimeoutError) as exc:
        is_timeout = isinstance(exc, (TimeoutError,)) or 'timed out' in str(exc).lower()
        raise EvaluationStageError(
            f'Model preflight warm-up failed at {selected_host}: '
            f'{type(exc).__name__}: {exc}',
            kind='model_preflight_timeout' if is_timeout else 'model_preflight',
            limit_seconds=(
                settings.model_preflight_timeout_seconds if is_timeout else None
            ),
        ) from exc
    return {
        'status': 'passed',
        'provider': 'ollama',
        'model': model,
        'host': selected_host,
        'latency_seconds': time.monotonic() - started,
    }


def add_timeout_event(
    result: dict[str, Any],
    kind: str,
    limit_seconds: float | None,
    **details: Any,
) -> None:
    """Append one de-duplicated structured timeout diagnostic."""
    event = {'kind': kind, 'limit_seconds': limit_seconds, **details}
    events = result.setdefault('timeout_events', [])
    if event not in events:
        events.append(event)


def safe_execution_name(execution_id: str) -> str:
    """Create a portable directory name from an execution identifier."""
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', execution_id).strip('._') or 'execution'


def retain_execution_logs(
    result: dict[str, Any],
    detector_log: Path,
    player_log: Path,
    log_root: Path,
) -> None:
    """Preserve subprocess logs for failed, timed-out, or backend-error runs."""
    detector_text = ''
    try:
        detector_text = detector_log.read_text(encoding='utf-8', errors='replace')
    except OSError:
        pass
    failure_lines = [
        line.strip()
        for line in detector_text.splitlines()
        if '[AAD] LLM call failed.' in line
    ]
    if failure_lines:
        result['backend_failures'] = failure_lines
        model_timeout = configured_model_timeout(
            result.get('_runtime_configuration', {})
        )
        if any('timeout' in line.lower() or 'timed out' in line.lower() for line in failure_lines):
            add_timeout_event(
                result,
                'model_request',
                model_timeout,
                failures=len(failure_lines),
            )

    problematic = (
        result.get('status') in {'failed', 'completed_with_timeout'}
        or bool(failure_lines)
        or any('contains no model response' in str(item) for item in result.get('warnings', []))
    )
    if not problematic:
        result.pop('_runtime_configuration', None)
        return
    sources = [source for source in (detector_log, player_log) if source.is_file()]
    if not sources:
        result.pop('_runtime_configuration', None)
        return
    destination = log_root / safe_execution_name(str(result.get('execution_id', 'run')))
    retained = []
    try:
        destination.mkdir(parents=True, exist_ok=True)
        for source in sources:
            target = destination / source.name
            shutil.copy2(source, target)
            retained.append(str(target))
    except OSError as exc:
        result.setdefault('warnings', []).append(
            f'Could not retain execution logs in {destination}: {exc}'
        )
    if retained:
        result['log_files'] = retained
    result.pop('_runtime_configuration', None)


def execute_one(
    bag: BagInfo,
    experiment: Experiment,
    trial: int,
    settings: RunnerSettings,
    collector: EvaluationCollector | None,
    annotation_truth: dict[str, Any] | None,
    log_root: Path,
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
        'bag_duration_seconds': bag.duration_seconds,
        'annotation_file': (
            annotation_truth.get('source') if annotation_truth is not None else None
        ),
        'warnings': (
            list(annotation_truth.get('warnings', []))
            if annotation_truth is not None
            else []
        ),
        'errors': [],
        'timeout_events': [],
    }
    raw_topic = str(experiment.aad_config.get('raw_input_topic', '/ai_anomaly_logging'))
    if raw_topic not in bag.topics:
        result['status'] = 'invalid_bag'
        result['errors'].append(f'Required detector input topic {raw_topic!r} is missing.')
        return result
    if settings.mode == 'human-labeled' and annotation_truth is None:
        result['status'] = 'invalid_bag'
        result['errors'].append('Annotation ground truth was not loaded for this bag.')
        return result
    if settings.mode == 'human-labeled' and annotation_truth.get('errors'):
        result['status'] = 'invalid_bag'
        result['errors'].extend(annotation_truth['errors'])
        return result
    if settings.dry_run:
        result['status'] = 'dry_run_validated'
        result['ground_truth'] = (
            {
                'source': 'annotation_sidecar',
                'annotation_file': annotation_truth['source'],
                'bag_anomalous': annotation_truth['bag_anomalous'],
                'label_count': len(annotation_truth['labels']),
                'ignored_interval_count': len(
                    annotation_truth['ignored_intervals']
                ),
            }
            if settings.mode == 'human-labeled'
            else None
        )
        return result

    assert collector is not None
    collector.reset()
    contaminants = collector.wait_for_clean_graph(
        raw_topic,
        settings.graph_clean_timeout_seconds,
    )
    if contaminants:
        result['status'] = 'failed'
        result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        result['graph_contamination'] = contaminants
        descriptions = ', '.join(
            f"{item['role']} {item['topic']} from "
            f"{item['node_namespace']}{item['node_name']}"
            for item in contaminants
        )
        result['errors'].append(
            'ROS graph contamination detected before replay: ' + descriptions
        )
        return result
    detector_process: subprocess.Popen[Any] | None = None
    player_process: subprocess.Popen[Any] | None = None
    collection_started = False
    start_monotonic = time.monotonic()

    with tempfile.TemporaryDirectory(prefix='aad_offline_eval_') as temp_name:
        temp_directory = Path(temp_name)
        artifact_directory = temp_directory / 'artifacts'
        detector_log = temp_directory / 'detector.log'
        player_log = temp_directory / 'player.log'
        runtime_config = copy.deepcopy(experiment.aad_config)
        runtime_config_path = temp_directory / 'aad_config.yaml'
        result['_runtime_configuration'] = runtime_config

        try:
            preflight = preflight_model(runtime_config, settings)
            result['model_preflight'] = preflight
            if preflight.get('host'):
                llm = runtime_config.setdefault('llm', {})
                if isinstance(llm, dict):
                    llm['ollama_host'] = preflight['host']
            runtime_config['api_artifact_output_dir'] = str(artifact_directory)
            runtime_config['api_artifact_max_files'] = 0
            runtime_config_path.write_text(
                yaml.safe_dump(runtime_config, sort_keys=False), encoding='utf-8'
            )
            environment = {
                **os.environ,
                'AAD_CONFIG_PATH': str(runtime_config_path),
                'ROS_DOMAIN_ID': str(settings.ros_domain_id),
            }
            if settings.provider_config_path is not None:
                environment['AAD_LLM_PROVIDERS_PATH'] = str(
                    settings.provider_config_path
                )
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
                raise EvaluationStageError(
                    'Detector readiness timed out before its input subscription '
                    'and decision publisher appeared.',
                    kind='detector_startup',
                    limit_seconds=settings.startup_timeout_seconds,
                )
            if settings.startup_grace_seconds:
                time.sleep(settings.startup_grace_seconds)

            collector.begin_collection()
            collection_started = True
            playback_command = [
                'ros2',
                'bag',
                'play',
                str(bag.path),
                '--rate',
                str(settings.playback_rate),
                '--exclude-topics',
                settings.decision_topic,
                settings.alert_topic,
                settings.llm_called_topic,
                settings.formatted_topic,
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
                playback_limit = playback_timeout(bag, settings)
                player_process.wait(timeout=playback_limit)
            except subprocess.TimeoutExpired as exc:
                raise EvaluationStageError(
                    'Bag playback exceeded its derived timeout.',
                    kind='bag_playback',
                    limit_seconds=playback_limit,
                ) from exc
            if player_process.returncode != 0:
                raise EvaluationError(
                    f'Bag playback exited with code {player_process.returncode}.'
                )

            drain_timeout = effective_inference_drain_timeout(
                runtime_config,
                settings,
            )
            result['inference_drain'] = {
                'configured_minimum_seconds': settings.inference_drain_timeout_seconds,
                'model_passes': settings.inference_drain_model_passes,
                'effective_timeout_seconds': drain_timeout,
            }
            drained = collector.wait_for_drain(
                settings.post_playback_grace_seconds,
                drain_timeout,
                artifact_directory,
            )
            snapshot = collector.snapshot()
            collector.end_collection()
            collection_started = False
            artifacts = read_artifacts(artifact_directory)
            llm_decisions, artifact_warnings = llm_decisions_from_artifacts(
                artifacts,
                snapshot['llm_calls'],
            )
            result['warnings'].extend(artifact_warnings)
            snapshot['immediate_decisions'] = immediate_decisions_from_topic(
                snapshot['decision_messages'], llm_decisions
            )
            snapshot['decisions'] = llm_decisions
            snapshot['pending_llm_calls'] = max(
                len(snapshot['llm_calls']) - len(artifacts),
                0,
            )
            if not drained:
                result['warnings'].append(
                    'Inference drain timed out; pending work is recorded in reliability metrics.'
                )
                add_timeout_event(
                    result,
                    'inference_drain',
                    drain_timeout,
                    pending_calls=snapshot['pending_llm_calls'],
                )
            elapsed = time.monotonic() - start_monotonic
            result.update(
                {
                    'status': 'completed' if drained else 'completed_with_timeout',
                    'finished_at_utc': datetime.now(timezone.utc).isoformat(),
                    'metrics': execution_metrics(snapshot, elapsed),
                    'decisions': snapshot['decisions'],
                }
            )
            if settings.mode == 'human-labeled':
                result['ground_truth'] = evaluate_ground_truth(
                    annotation_truth['labels'],
                    snapshot['decisions'],
                    settings.label_buffer_seconds,
                    settings.playback_rate,
                    annotation_truth['ignored_intervals'],
                    annotation_truth['bag_anomalous'],
                )
                result['ground_truth']['annotation_file'] = annotation_truth['source']
            else:
                result['ground_truth'] = None
        except EvaluationStageError as exc:
            result['status'] = 'failed'
            result['errors'].append(str(exc))
            result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            if exc.limit_seconds is not None:
                add_timeout_event(result, exc.kind, exc.limit_seconds)
        except EvaluationError as exc:
            result['status'] = 'failed'
            result['errors'].append(str(exc))
            result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        except Exception as exc:
            result['status'] = 'failed'
            result['errors'].append(f'Unexpected execution error: {type(exc).__name__}: {exc}')
            result['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        finally:
            if collection_started:
                collector.end_collection()
            player_forced = stop_process(
                player_process,
                settings.shutdown_timeout_seconds,
            )
            detector_forced = stop_process(
                detector_process,
                settings.shutdown_timeout_seconds,
            )
            if player_forced:
                add_timeout_event(
                    result,
                    'player_shutdown',
                    settings.shutdown_timeout_seconds,
                )
            if detector_forced:
                add_timeout_event(
                    result,
                    'detector_shutdown',
                    settings.shutdown_timeout_seconds,
                )
            if (player_forced or detector_forced) and result.get('status') == 'completed':
                result['status'] = 'completed_with_timeout'
            retain_execution_logs(result, detector_log, player_log, log_root)
            result.setdefault(
                'finished_at_utc', datetime.now(timezone.utc).isoformat()
            )
    return result


def aggregate_executions(
    executions: Sequence[dict[str, Any]],
    mode: str,
    expected: int | None = None,
) -> dict[str, Any]:
    """Aggregate performance, behavior, reliability, and optional truth metrics."""
    completed = [item for item in executions if item['status'].startswith('completed')]
    decisions = [
        decision
        for item in completed
        for decision in item.get('decisions', [])
        if isinstance(decision, dict)
    ]
    model_latencies = [
        decision['model_latency_seconds']
        for decision in decisions
        if decision.get('model_latency_seconds') is not None
    ]
    positive = sum(decision.get('anomaly') is True for decision in decisions)
    negative = sum(decision.get('anomaly') is False for decision in decisions)
    unparseable = len(decisions) - positive - negative
    severity_distribution = dict(
        sorted(
            Counter(
                decision.get('severity')
                for decision in decisions
                if decision.get('severity')
            ).items()
        )
    )
    action_distribution = dict(
        sorted(
            Counter(
                decision.get('action')
                for decision in decisions
                if decision.get('action')
            ).items()
        )
    )
    calls = sum(
        item.get('metrics', {}).get('reliability', {}).get('llm_calls', 0)
        for item in completed
    )
    immediate_safety_decisions = sum(
        item.get('metrics', {})
        .get('behavior', {})
        .get('immediate_safety_decisions', 0)
        for item in completed
    )
    result: dict[str, Any] = {
        'executions_expected': len(executions) if expected is None else expected,
        'executions_completed': len(completed),
        'executions_failed': sum(item['status'] == 'failed' for item in executions),
        'executions_invalid': sum(item['status'] == 'invalid_bag' for item in executions),
        'model_response_latency': latency_summary(model_latencies),
        'behavior': {
            'immediate_safety_decisions': immediate_safety_decisions,
            'positive_decisions': positive,
            'negative_decisions': negative,
            'unparseable_decisions': unparseable,
            'positive_decision_rate': safe_ratio(positive, positive + negative),
            'alerts': sum(
                item.get('metrics', {}).get('behavior', {}).get('alert_count', 0)
                for item in completed
            ),
            'severity_distribution': severity_distribution,
            'action_distribution': action_distribution,
        },
        'reliability': {
            'llm_calls': calls,
            'pending_llm_calls_at_timeout': sum(
                item.get('metrics', {})
                .get('reliability', {})
                .get('pending_llm_calls_at_timeout', 0)
                for item in completed
            ),
            'timeout_counts': dict(
                sorted(
                    Counter(
                        event.get('kind')
                        for item in executions
                        for event in item.get('timeout_events', [])
                        if isinstance(event, dict) and event.get('kind')
                    ).items()
                )
            ),
            'backend_failures': sum(
                len(item.get('backend_failures', []))
                for item in executions
                if isinstance(item.get('backend_failures', []), list)
            ),
            'runs_with_diagnostic_logs': sum(
                bool(item.get('log_files')) for item in executions
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
    ignored = sum(
        item['metrics'].get('ignored_positive_decisions', 0) for item in truth_runs
    )
    bag_outcomes = Counter(
        item.get('bag_classification', {}).get('result')
        for item in truth_runs
        if item.get('bag_classification', {}).get('result')
    )
    bag_total = sum(bag_outcomes.values())
    result['ground_truth_metrics'] = {
        'true_positives': true_positives,
        'false_negatives': false_negatives,
        'false_positives': false_positives,
        'unscorable_positive_decisions': unscorable,
        'ignored_positive_decisions': ignored,
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
        'bag_true_positives': bag_outcomes['true_positive'],
        'bag_true_negatives': bag_outcomes['true_negative'],
        'bag_false_positives': bag_outcomes['false_positive'],
        'bag_false_negatives': bag_outcomes['false_negative'],
        'bag_accuracy': safe_ratio(
            bag_outcomes['true_positive'] + bag_outcomes['true_negative'],
            bag_total,
        ),
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


def normalized_api_response(value: Any) -> str | None:
    """Normalize an API response so formatting alone does not count as a change."""
    if not isinstance(value, str) or not value.strip():
        return None
    payload = parse_json_object(value)
    if payload is not None:
        return json.dumps(payload, sort_keys=True, separators=(',', ':'))
    return ' '.join(value.split())


def api_responses_by_source(execution: dict[str, Any]) -> dict[tuple[int, int], str]:
    """Index captured API responses by their latest replay-source timestamp."""
    records: dict[tuple[int, int], str] = {}
    occurrences: dict[int, int] = {}
    for decision in execution.get('decisions', []):
        if not isinstance(decision, dict):
            continue
        response = normalized_api_response(decision.get('artifact_api_response'))
        timestamps = decision.get('source_timestamp_candidates_ns', [])
        source_timestamps = [
            timestamp
            for timestamp in timestamps
            if isinstance(timestamp, int) and not isinstance(timestamp, bool)
        ]
        if response is None or not source_timestamps:
            continue
        source_timestamp = max(source_timestamps)
        occurrence = occurrences.get(source_timestamp, 0)
        occurrences[source_timestamp] = occurrence + 1
        records[(source_timestamp, occurrence)] = response
    return records


def ollama_response_judge(
    configuration: dict[str, Any],
) -> Callable[[str, str], dict[str, Any]] | None:
    """Return an LLM judge for semantic API-response comparisons."""
    llm = configuration.get('llm', {})
    if not isinstance(llm, dict) or not llm.get('local'):
        return None
    model = str(llm.get('model', '')).strip()
    host = str(llm.get('ollama_host', 'http://127.0.0.1:11434')).rstrip('/')
    if not model or not host:
        return None
    try:
        timeout = max(60.0, float(llm.get('timeout_seconds', 60.0)))
    except (TypeError, ValueError):
        timeout = 60.0
    cache: dict[tuple[str, str], dict[str, Any]] = {}

    def judge(left: str, right: str) -> dict[str, Any]:
        cache_key = tuple(sorted((left, right)))
        if cache_key in cache:
            return dict(cache[cache_key])
        prompt = (
            'Compare these two anomaly-detection API responses for the same '
            'replayed event. Decide whether they are materially consistent. '
            'Treat wording-only differences as consistent. Mark them inconsistent '
            'when their anomaly decision, severity, action, or stated evidence '
            'materially conflicts. Return only JSON with this shape: '
            '{"consistent": true or false, "rationale": "brief reason"}.\n\n'
            f'Response A:\n{left[:12_000]}\n\nResponse B:\n{right[:12_000]}'
        )
        payload = {
            'model': model,
            'messages': [
                {
                    'role': 'system',
                    'content': (
                        'You are a precise evaluator of anomaly-detection '
                        'responses. Follow the requested JSON schema exactly.'
                    ),
                },
                {'role': 'user', 'content': prompt},
            ],
            'stream': False,
            'format': 'json',
            'think': False,
            'keep_alive': '10m',
            'options': {'temperature': 0, 'num_predict': 96},
        }
        request = Request(
            f'{host}/api/chat',
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urlopen(request, timeout=timeout) as response:
                body = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, OSError, TimeoutError, json.JSONDecodeError) as exc:
            result = {'consistent': None, 'error': f'{type(exc).__name__}: {exc}'}
            cache[cache_key] = result
            return dict(result)

        message = body.get('message', {}) if isinstance(body, dict) else {}
        content = message.get('content') if isinstance(message, dict) else None
        verdict = parse_json_object(str(content or ''))
        consistent = coerce_bool(verdict.get('consistent')) if verdict else None
        if consistent is None:
            result = {
                'consistent': None,
                'error': 'Judge returned no boolean "consistent" field.',
            }
            cache[cache_key] = result
            return dict(result)
        rationale = str(verdict.get('rationale', '')).strip()
        result = {'consistent': consistent, 'rationale': rationale}
        cache[cache_key] = result
        return dict(result)

    return judge


def api_response_comparison(
    left: dict[str, Any],
    right: dict[str, Any],
    response_judge: Callable[[str, str], dict[str, Any]] | None = None,
    response_cache: dict[int, dict[tuple[int, int], str]] | None = None,
) -> dict[str, Any]:
    """Use an LLM to compare API responses for matching replay sources."""
    if response_cache is None:
        left_records = api_responses_by_source(left)
        right_records = api_responses_by_source(right)
    else:
        left_key = id(left)
        right_key = id(right)
        if left_key not in response_cache:
            response_cache[left_key] = api_responses_by_source(left)
        if right_key not in response_cache:
            response_cache[right_key] = api_responses_by_source(right)
        left_records = response_cache[left_key]
        right_records = response_cache[right_key]
    shared = sorted(set(left_records).intersection(right_records))
    judgments = (
        [
            {
                'source_timestamp_ns': key[0],
                'occurrence': key[1],
                **response_judge(left_records[key], right_records[key]),
            }
            for key in shared
        ]
        if response_judge is not None
        else []
    )
    judged = [item for item in judgments if isinstance(item.get('consistent'), bool)]
    consistent = sum(item['consistent'] is True for item in judged)
    return {
        'matched_responses': len(shared),
        'llm_judged_responses': len(judged),
        'consistent_responses': consistent,
        'semantic_response_agreement': safe_ratio(consistent, len(judged)),
        'judge_errors': len(judgments) - len(judged),
        'left_only_responses': len(set(left_records).difference(right_records)),
        'right_only_responses': len(set(right_records).difference(left_records)),
        'judgments': judgments,
    }


def api_response_repeatability(
    executions: Sequence[dict[str, Any]],
    response_judge: Callable[[str, str], dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Use an LLM to measure response consistency across repeated replays."""
    by_bag: dict[str, list[dict[tuple[int, int], str]]] = {}
    for execution in executions:
        by_bag.setdefault(execution['bag'], []).append(api_responses_by_source(execution))
    rows = []
    for bag, record_sets in sorted(by_bag.items()):
        responses_by_source: dict[tuple[int, int], list[str]] = {}
        for records in record_sets:
            for source, response in records.items():
                responses_by_source.setdefault(source, []).append(response)
        response_pairs = [
            (responses[0], response)
            for responses in responses_by_source.values()
            for response in responses[1:]
        ]
        comparisons = (
            [response_judge(left, right) for left, right in response_pairs]
            if response_judge is not None
            else []
        )
        judged = [
            comparison
            for comparison in comparisons
            if isinstance(comparison.get('consistent'), bool)
        ]
        consistent = sum(comparison['consistent'] is True for comparison in judged)
        rows.append(
            {
                'bag': bag,
                'completed_trials': len(record_sets),
                'comparable_responses': len(response_pairs),
                'llm_judged_responses': len(judged),
                'consistent_responses': consistent,
                'semantic_response_agreement': safe_ratio(consistent, len(judged)),
                'judge_errors': len(comparisons) - len(judged),
            }
        )
    return rows


def repeatability_summary(
    executions: Sequence[dict[str, Any]],
    response_judge: Callable[[str, str], dict[str, Any]] | None = None,
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
        'api_response_consistency': api_response_repeatability(
            completed, response_judge
        ),
    }


def configuration_comparisons(
    grouped: dict[str, list[dict[str, Any]]],
    response_judge: Callable[[str, str], dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    """Compare bag-level positive behavior between every experiment pair."""
    comparisons: list[dict[str, Any]] = []
    response_cache: dict[int, dict[tuple[int, int], str]] = {}
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
            response_comparison = api_response_comparison(
                left[key], right[key], response_judge, response_cache
            )
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
                    'api_response_comparison': response_comparison,
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
    stem = f'{started:%Y-%m-%d_%H-%M-%S}'
    suffix = 0
    while True:
        postfix = '' if suffix == 0 else f'-{suffix + 1}'
        candidate = directory / f'{stem}{postfix}.md'
        if candidate.with_suffix('.json').exists():
            suffix += 1
            continue
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
            cwd=Path(__file__).resolve().parents[2],
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
    truth = metrics.get('ground_truth_metrics')
    rows: list[tuple[str, Any]] = []
    if 'elapsed_seconds' in metrics:
        rows.append(('Elapsed time', seconds_text(metrics.get('elapsed_seconds'))))
    rows.extend(
        [
            ('Model responses measured for latency', latency.get('count', 0)),
            ('Mean response latency', seconds_text(latency.get('mean_seconds'))),
            ('Median response latency', seconds_text(latency.get('median_seconds'))),
            ('Minimum response latency', seconds_text(latency.get('minimum_seconds'))),
            ('Maximum response latency', seconds_text(latency.get('maximum_seconds'))),
            (
                'Immediate safety decisions (excluded from LLM table)',
                behavior.get('immediate_safety_decisions', 0),
            ),
            ('Positive LLM decisions', behavior.get('positive_decisions', 0)),
            ('Negative LLM decisions', behavior.get('negative_decisions', 0)),
            ('Unparseable LLM decisions', behavior.get('unparseable_decisions', 0)),
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
    if isinstance(truth, dict):
        rows.extend(
            [
                ('Truth events: true positives', truth.get('true_positives', 0)),
                ('Truth events: false negatives', truth.get('false_negatives', 0)),
                ('Truth events: false positives', truth.get('false_positives', 0)),
                ('Truth event precision', ratio_text(truth.get('precision'))),
                ('Truth event recall', ratio_text(truth.get('recall'))),
                ('Truth event F1', ratio_text(truth.get('f1'))),
                ('Bag-level accuracy', ratio_text(truth.get('bag_accuracy'))),
                ('Bag true positives', truth.get('bag_true_positives', 0)),
                ('Bag true negatives', truth.get('bag_true_negatives', 0)),
                ('Bag false positives', truth.get('bag_false_positives', 0)),
                ('Bag false negatives', truth.get('bag_false_negatives', 0)),
                (
                    'Unscorable positive decisions',
                    truth.get('unscorable_positive_decisions', 0),
                ),
                (
                    'Ignored positive decisions',
                    truth.get('ignored_positive_decisions', 0),
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


def completed_positive_counts(configuration: dict[str, Any]) -> list[int]:
    """Return positive-decision counts for completed runs in report order."""
    counts = []
    for execution in configuration.get('executions', []):
        if not isinstance(execution, dict) or not str(
            execution.get('status', '')
        ).startswith('completed'):
            continue
        metrics = execution.get('metrics', {})
        behavior = metrics.get('behavior', {}) if isinstance(metrics, dict) else {}
        count = behavior.get('positive_decisions', 0)
        counts.append(
            count if isinstance(count, int) and not isinstance(count, bool) else 0
        )
    return counts


def detailed_llm_issue_counts(report: dict[str, Any]) -> tuple[int, int]:
    """Count safe fallbacks and failed calls retained in final decisions."""
    fallbacks = 0
    failures = 0
    for configuration in report.get('configurations', []):
        if not isinstance(configuration, dict):
            continue
        for execution in configuration.get('executions', []):
            if not isinstance(execution, dict):
                continue
            for decision in execution.get('final_decisions', []):
                if not isinstance(decision, dict):
                    continue
                summary = str(decision.get('summary', '')).lower()
                fallbacks += 'invalid/malformed llm response' in summary
                failures += 'llm call failed' in summary
    return fallbacks, failures


def append_report_overview(lines: list[str], report: dict[str, Any]) -> None:
    """Append a concise, data-derived explanation before the full results."""
    experiment = report['experiment']
    overall = report['overall']
    completed = overall.get('executions_completed', 0)
    expected = overall.get('executions_expected', 0)
    failed = overall.get('executions_failed', 0)
    invalid = overall.get('executions_invalid', 0)
    lines.extend(['## At a glance', ''])
    if completed == expected and not failed and not invalid:
        lines.append(
            f'- All {expected} planned replay(s) completed; none was marked '
            'failed or invalid.'
        )
    else:
        lines.append(
            f'- {completed} of {expected} planned replay(s) completed; '
            f'{failed} failed and {invalid} were invalid.'
        )
    if experiment.get('evaluation_mode') == 'unlabeled':
        lines.append(
            '- This is an **unlabeled** replay, so detections are not measures '
            'of accuracy, precision, or recall.'
        )

    configurations = [
        configuration
        for configuration in report.get('configurations', [])
        if isinstance(configuration, dict)
    ]
    latency_rows = []
    for configuration in configurations:
        aggregate = configuration.get('aggregate', {})
        latency = (
            aggregate.get('model_response_latency', {})
            if isinstance(aggregate, dict)
            else {}
        )
        mean = latency.get('mean_seconds') if isinstance(latency, dict) else None
        if isinstance(mean, (int, float)) and not isinstance(mean, bool):
            latency_rows.append((str(configuration.get('name', 'unnamed')), mean))
    if latency_rows:
        latency_rows.sort(key=lambda item: item[1])
        values = '; '.join(
            f'`{name}` {seconds_text(mean)}' for name, mean in latency_rows
        )
        lines.append(f'- Mean model-response latency: {values}.')
        if len(latency_rows) > 1 and latency_rows[1][1] > 0:
            fastest_name, fastest_mean = latency_rows[0]
            next_mean = latency_rows[1][1]
            reduction = next_mean - fastest_mean
            percent = reduction / next_mean
            lines.append(
                f'  `{fastest_name}` was {seconds_text(reduction)} ({percent:.1%}) '
                'faster than the next-fastest configuration.'
            )

    count_rows = [
        (
            str(configuration.get('name', 'unnamed')),
            completed_positive_counts(configuration),
        )
        for configuration in configurations
    ]
    count_rows = [(name, counts) for name, counts in count_rows if counts]
    if count_rows:
        values = '; '.join(
            f'`{name}`: {", ".join(str(count) for count in counts)}'
            for name, counts in count_rows
        )
        lines.append(f'- Positive-decision counts by completed run: {values}.')

    fallbacks, failures = detailed_llm_issue_counts(report)
    if fallbacks or failures:
        issue_parts = []
        if fallbacks:
            issue_parts.append(
                f'{fallbacks} safe fallback(s) after malformed model output'
            )
        if failures:
            issue_parts.append(f'{failures} failed LLM call(s)')
        lines.append(
            '- Review the detailed decisions before treating detection totals as '
            'operational results: they include ' + ' and '.join(issue_parts) + '.'
        )

    lines.extend(
        [
            '',
            '### Reading this report',
            '',
            '- A **positive decision** is an LLM decision marked `Anomaly: Yes`. '
            'In unlabeled mode, it is not a verified anomaly.',
            '- **Model latency** is reported only when a measured model response '
            'was available. An em dash (`—`) means no latency was recorded for '
            'that LLM decision.',
            '- Immediate safety decisions are counted separately and omitted from '
            'the LLM-decision table because they have no original model response.',
            '- When semantic judging is enabled, **LLM-judged API-response '
            'consistency** compares paired original responses for the same replay '
            'source. It permits wording changes but flags material conflicts in '
            'the decision, severity, action, or evidence.',
            '- Aggregate severity and action distributions combine all completed '
            'replays. `None` means no completed decision supplied a value for that '
            'field.',
            '- **Unparseable decisions** are LLM decisions lacking a Yes/No '
            'anomaly value. A safe fallback after malformed model output can still '
            'be a parseable No decision, so it may appear in the details without '
            'increasing this count.',
            '',
        ]
    )


def render_ground_truth(lines: list[str], truth: Any) -> None:
    """Append annotation-sidecar metrics without exposing raw cached messages."""
    if not isinstance(truth, dict):
        return
    lines.extend(['', '#### Annotation ground-truth results', ''])
    source_rows = [
        ('Annotation file', truth.get('annotation_file')),
        ('Bag expected anomalous', truth.get('bag_anomalous')),
        ('Positive truth events', truth.get('label_count')),
        ('Ignored intervals', truth.get('ignored_interval_count')),
        (
            'Bag classification',
            truth.get('bag_classification', {}).get('result')
            if isinstance(truth.get('bag_classification'), dict)
            else None,
        ),
    ]
    lines.extend(markdown_table(('Ground truth', 'Value'), source_rows))
    lines.extend(['', '**Event metrics**', ''])
    metrics = truth.get('metrics', {})
    rows = []
    if isinstance(metrics, dict):
        for key, value in metrics.items():
            label = key.replace('_', ' ').capitalize()
            if key in {
                'detection_accuracy',
                'precision',
                'recall',
                'f1',
                'bag_accuracy',
            }:
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


def append_configuration_comparisons(
    lines: list[str], comparisons: list[dict[str, Any]]
) -> None:
    """Append pairwise experiment comparison tables."""
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
        response_rows = [
            (
                Path(item['bag']).name,
                item['trial'],
                item.get('api_response_comparison', {}).get('matched_responses', 0),
                item.get('api_response_comparison', {}).get(
                    'llm_judged_responses', 0
                ),
                item.get('api_response_comparison', {}).get(
                    'consistent_responses', 0
                ),
                ratio_text(
                    item.get('api_response_comparison', {}).get(
                        'semantic_response_agreement'
                    )
                ),
                item.get('api_response_comparison', {}).get('judge_errors', 0),
                item.get('api_response_comparison', {}).get('left_only_responses', 0),
                item.get('api_response_comparison', {}).get('right_only_responses', 0),
            )
            for item in comparison.get('executions', [])
        ]
        lines.extend(['', '**LLM-judged API-response consistency**', ''])
        lines.extend(
            markdown_table(
                (
                    'Bag',
                    'Trial',
                    'Matched responses',
                    'LLM judgments',
                    'Consistent responses',
                    'Semantic agreement',
                    'Judge errors',
                    'Left only',
                    'Right only',
                ),
                response_rows,
            )
        )
        judgment_rows = [
            (
                Path(item['bag']).name,
                item['trial'],
                judgment.get('source_timestamp_ns'),
                judgment.get('consistent'),
                judgment.get('rationale') or judgment.get('error'),
            )
            for item in comparison.get('executions', [])
            for judgment in item.get('api_response_comparison', {}).get('judgments', [])
            if isinstance(judgment, dict)
        ]
        if judgment_rows:
            lines.extend(['', '**LLM response judgments**', ''])
            lines.extend(
                markdown_table(
                    ('Bag', 'Trial', 'Source timestamp', 'Consistent', 'Rationale'),
                    judgment_rows,
                )
            )


def append_execution_details(
    lines: list[str], execution: dict[str, Any]
) -> None:
    """Append one execution's metrics, decisions, and truth details."""
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
        ('Annotation file', execution.get('annotation_file')),
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
                '<br>'.join(
                    f'{message_index}. {message}'
                    for message_index, message in enumerate(
                        decision.get('error_context_messages', []),
                        start=1,
                    )
                )
                or None,
                decision.get('summary'),
                decision.get('artifact_api_response'),
            )
            for index, decision in enumerate(decisions, start=1)
            if isinstance(decision, dict)
        ]
        lines.extend(['', '**LLM decisions**', ''])
        lines.extend(
            markdown_table(
                (
                    '#',
                    'Anomaly',
                    'Severity',
                    'Action',
                    'Model latency',
                    'Error messages analyzed',
                    'Summary',
                    'Original LLM response',
                ),
                decision_rows,
            )
        )
    append_messages(lines, 'Warnings', execution.get('warnings'))
    append_messages(lines, 'Errors', execution.get('errors'))
    render_ground_truth(lines, execution.get('ground_truth'))


def append_configuration_details(
    lines: list[str], configurations: list[dict[str, Any]]
) -> None:
    """Append configuration, repeatability, and per-run details."""
    lines.extend(['', '## Configuration details'])
    for configuration in configurations:
        lines.extend(['', f'### {configuration["name"]}', ''])
        randomization = configuration.get('randomization')
        if isinstance(randomization, dict):
            lines.extend(
                [
                    f'Randomized from `{randomization.get("source_experiment")}`: '
                    f'sample {randomization.get("sample_index")} of '
                    f'{randomization.get("sample_count")} '
                    f'(seed `{randomization.get("seed")}`).',
                    '',
                    '#### Sampled LLM settings',
                    '',
                ]
            )
            sampled = randomization.get('sampled_llm_parameters', {})
            if isinstance(sampled, dict):
                lines.extend(
                    markdown_table(('Parameter', 'Selected value'), sampled.items())
                )
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
            (
                'Model provider',
                llm.get('provider', llm.get('model_provider')),
            ),
            ('Model', llm.get('model')),
            ('Local model', llm.get('local')),
            ('Ollama host', llm.get('ollama_host')),
            ('Model timeout', seconds_text(llm.get('timeout_seconds'))),
            ('Model token context', llm.get('num_ctx')),
            ('Maximum output tokens', llm.get('num_predict')),
            ('Vision enabled', llm.get('vision_enabled')),
            ('Image context enabled', llm.get('image_context_enabled')),
            ('Routine image frames', llm.get('image_context_max_frames')),
            ('Maximum image frames', llm.get('image_max_frames')),
            ('Image JPEG quality', llm.get('image_jpeg_quality')),
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
        response_consistency = repeatability.get('api_response_consistency', [])
        if response_consistency:
            response_consistency_rows = [
                (
                    Path(item['bag']).name,
                    item.get('completed_trials', 0),
                    item.get('comparable_responses', 0),
                    item.get('llm_judged_responses', 0),
                    item.get('consistent_responses', 0),
                    ratio_text(item.get('semantic_response_agreement')),
                    item.get('judge_errors', 0),
                )
                for item in response_consistency
            ]
            lines.extend(['', '**LLM-judged API-response consistency across trials**', ''])
            lines.extend(
                markdown_table(
                    (
                        'Bag',
                        'Completed trials',
                        'Comparable responses',
                        'LLM judgments',
                        'Consistent responses',
                        'Semantic agreement',
                        'Judge errors',
                    ),
                    response_consistency_rows,
                )
            )
        for execution in configuration['executions']:
            append_execution_details(lines, execution)


def report_executions(
    report: dict[str, Any],
) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    """Return configuration/execution pairs in report order."""
    return [
        (configuration, execution)
        for configuration in report.get('configurations', [])
        if isinstance(configuration, dict)
        for execution in configuration.get('executions', [])
        if isinstance(execution, dict)
    ]


def short_path(value: Any) -> str | None:
    """Reduce a report path to its final component."""
    if not isinstance(value, (str, Path)) or not str(value):
        return None
    return Path(value).name


def execution_status_text(executions: Sequence[dict[str, Any]]) -> str:
    """Summarize one configuration's execution statuses."""
    statuses = Counter(str(item.get('status', 'unknown')) for item in executions)
    if not statuses:
        return 'Not run'
    if len(statuses) == 1:
        return next(iter(statuses)).replace('_', ' ').title()
    return ', '.join(
        f'{status.replace("_", " ").title()}: {count}'
        for status, count in sorted(statuses.items())
    )


def captured_decision_count(metrics: dict[str, Any]) -> int:
    """Count reportable model responses represented by aggregate metrics."""
    behavior = metrics.get('behavior', {})
    if not isinstance(behavior, dict):
        return 0
    return sum(
        int(behavior.get(key, 0) or 0)
        for key in (
            'positive_decisions',
            'negative_decisions',
            'unparseable_decisions',
        )
    )


def configuration_result_rows(report: dict[str, Any]) -> list[tuple[Any, ...]]:
    """Build one comparison-first result row per configuration."""
    rows = []
    for configuration in report.get('configurations', []):
        if not isinstance(configuration, dict):
            continue
        aggregate = configuration.get('aggregate', {})
        aggregate = aggregate if isinstance(aggregate, dict) else {}
        reliability = aggregate.get('reliability', {})
        reliability = reliability if isinstance(reliability, dict) else {}
        behavior = aggregate.get('behavior', {})
        behavior = behavior if isinstance(behavior, dict) else {}
        truth = aggregate.get('ground_truth_metrics')
        truth = truth if isinstance(truth, dict) else {}
        latency = aggregate.get('model_response_latency', {})
        latency = latency if isinstance(latency, dict) else {}
        executions = [
            item
            for item in configuration.get('executions', [])
            if isinstance(item, dict)
        ]
        responses = captured_decision_count(aggregate)
        calls = int(reliability.get('llm_calls', 0) or 0)
        rows.append(
            (
                configuration.get('name'),
                execution_status_text(executions),
                f'{responses} / {calls}',
                reliability.get('pending_llm_calls_at_timeout', 0),
                (
                    f'{truth.get("true_positives", 0)} / '
                    f'{truth.get("false_negatives", 0)} / '
                    f'{truth.get("false_positives", 0)}'
                    if truth
                    else '—'
                ),
                ratio_text(truth.get('precision')) if truth else '—',
                ratio_text(truth.get('recall')) if truth else '—',
                ratio_text(truth.get('f1')) if truth else '—',
                ratio_text(truth.get('bag_accuracy')) if truth else '—',
                seconds_text(latency.get('mean_seconds')),
                behavior.get('alerts', 0),
            )
        )
    return rows


def append_reliability_summary(lines: list[str], report: dict[str, Any]) -> None:
    """Lead with incomplete inference instead of burying it in run details."""
    overall = report.get('overall', {})
    overall = overall if isinstance(overall, dict) else {}
    reliability = overall.get('reliability', {})
    reliability = reliability if isinstance(reliability, dict) else {}
    pairs = report_executions(report)
    timeouts = sum(
        execution.get('status') == 'completed_with_timeout'
        for _, execution in pairs
    )
    calls = int(reliability.get('llm_calls', 0) or 0)
    responses = captured_decision_count(overall)
    pending = int(reliability.get('pending_llm_calls_at_timeout', 0) or 0)
    if timeouts or pending or responses < calls:
        lines.extend(
            [
                '> **Results incomplete.** '
                f'{timeouts} of {len(pairs)} run(s) timed out waiting for inference. '
                f'{responses} of {calls} LLM call(s) produced reportable responses; '
                f'{pending} remained pending at timeout.',
                '',
            ]
        )
    else:
        lines.extend(
            [
                f'All {len(pairs)} run(s) completed with {responses} reportable '
                f'response(s) from {calls} LLM call(s).',
                '',
            ]
        )


def append_compact_issues(lines: list[str], report: dict[str, Any]) -> None:
    """Aggregate repeated run warnings into a short diagnostic list."""
    warnings = [
        str(message)
        for _, execution in report_executions(report)
        for message in execution.get('warnings', [])
    ]
    errors = [
        str(message)
        for _, execution in report_executions(report)
        for message in execution.get('errors', [])
    ]
    timeout_events = [
        event
        for _, execution in report_executions(report)
        for event in (execution.get('timeout_events') or [])
        if isinstance(event, dict) and event.get('kind')
    ]
    timeout_counts = Counter(str(event['kind']) for event in timeout_events)
    timeout_warnings = (
        timeout_counts.get('inference_drain', 0)
        or sum('Inference drain timed out' in item for item in warnings)
    )
    empty_artifacts = sum('contains no model response' in item for item in warnings)
    count_mismatches = sum('Artifact-file/LLM-call counts differ' in item for item in warnings)
    recognized = (
        'Inference drain timed out',
        'contains no model response',
        'Artifact-file/LLM-call counts differ',
    )
    other = Counter(
        item for item in warnings if not any(marker in item for marker in recognized)
    )
    rows = []
    if timeout_warnings:
        rows.append(f'{timeout_warnings} run(s) hit the inference-drain timeout.')
    timeout_labels = {
        'model_preflight_timeout': 'model preflight',
        'detector_startup': 'detector startup',
        'bag_playback': 'bag playback',
        'model_request': 'model request',
        'player_shutdown': 'bag-player shutdown',
        'detector_shutdown': 'detector shutdown',
    }
    rows.extend(
        f'{count} run event(s) hit the {timeout_labels[kind]} timeout.'
        for kind, count in sorted(timeout_counts.items())
        if kind in timeout_labels and count
    )
    if empty_artifacts:
        rows.append(f'{empty_artifacts} API artifact(s) contained no model response.')
    if count_mismatches:
        rows.append(
            f'{count_mismatches} run(s) had different artifact and LLM-call counts.'
        )
    backend_failures = sum(
        len(execution.get('backend_failures', []))
        for _, execution in report_executions(report)
        if isinstance(execution.get('backend_failures', []), list)
    )
    if backend_failures:
        rows.append(f'{backend_failures} model backend failure(s) were logged.')
    log_directories = {
        str(Path(path).parent)
        for _, execution in report_executions(report)
        for path in (execution.get('log_files') or [])
        if isinstance(path, str)
    }
    if log_directories:
        roots = sorted({Path(path).parent.name for path in log_directories})
        rows.append('Diagnostic logs retained under ' + ', '.join(f'`{root}/`' for root in roots) + '.')
    rows.extend(
        f'{message}' + (f' (repeated {count} times)' if count > 1 else '')
        for message, count in other.items()
    )
    rows.extend(dict.fromkeys(errors))
    if rows:
        lines.extend(['## Reliability issues', ''])
        lines.extend(f'- {row}' for row in rows)
        lines.append('')


def flatten_parameters(value: Any, prefix: str = '') -> dict[str, Any]:
    """Flatten nested evaluated parameters for a compact comparison table."""
    if not isinstance(value, dict):
        return {prefix: value}
    result: dict[str, Any] = {}
    for key, item in value.items():
        name = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(item, dict):
            result.update(flatten_parameters(item, name))
        else:
            result[name] = item
    return result


def parameter_label(name: str) -> str:
    """Give commonly compared configuration fields concise labels."""
    labels = {
        'api_frequency_seconds': 'API interval',
        'cache_max_items': 'Cache messages',
        'cache_max_age_seconds': 'Cache age',
        'llm_min_trigger_importance': 'Trigger importance',
        'llm.provider': 'Provider',
        'llm.model_provider': 'Model provider',
        'llm.model': 'Model',
        'llm.local': 'Local model',
        'llm.ollama_host': 'Ollama host',
        'llm.timeout_seconds': 'Model timeout',
        'llm.vision_enabled': 'Vision',
        'llm.image_context_enabled': 'Image context',
        'llm.image_context_max_frames': 'Routine image frames',
        'llm.image_max_frames': 'Maximum image frames',
        'llm.num_ctx': 'Context tokens',
        'llm.num_predict': 'Output tokens',
        'llm.image_jpeg_quality': 'JPEG quality',
        'llm.system_prompt_hash': 'System prompt hash',
    }
    return labels.get(name, name.replace('.', ' / ').replace('_', ' ').title())


def append_configuration_parameters(lines: list[str], report: dict[str, Any]) -> None:
    """Show shared settings once and only repeat parameters that differ."""
    configurations = [
        item for item in report.get('configurations', []) if isinstance(item, dict)
    ]
    if not configurations:
        return
    flattened = [
        flatten_parameters(item.get('evaluated_parameters', {}))
        for item in configurations
    ]
    keys = sorted(set().union(*(values.keys() for values in flattened)))
    common = [
        key
        for key in keys
        if len({json.dumps(values.get(key), sort_keys=True) for values in flattened}) == 1
        and flattened[0].get(key) is not None
    ]
    differing = [
        key
        for key in keys
        if len({json.dumps(values.get(key), sort_keys=True) for values in flattened}) > 1
    ]
    lines.extend(['## Configuration', ''])
    important_common = [
        key
        for key in (
            'llm.provider',
            'llm.model_provider',
            'llm.model',
            'api_frequency_seconds',
            'cache_max_items',
            'cache_max_age_seconds',
            'llm_min_trigger_importance',
        )
        if key in common
    ]
    if important_common:
        values = ', '.join(
            f'{parameter_label(key)}: `{flattened[0][key]}`'
            for key in important_common
        )
        lines.extend([f'**Shared settings:** {values}.', ''])
    headers = ('Parameter',) + tuple(item.get('name') for item in configurations)
    rows: list[tuple[Any, ...]] = [
        ('Configuration hash',)
        + tuple(str(item.get('configuration_hash', ''))[:12] for item in configurations)
    ]
    rows.extend(
        (parameter_label(key),) + tuple(values.get(key) for values in flattened)
        for key in differing
    )
    lines.extend(markdown_table(headers, rows))
    lines.append('')


def append_agreement_matrix(lines: list[str], report: dict[str, Any]) -> None:
    """Replace repeated pairwise sections with one outcome-agreement matrix."""
    names = [
        str(item.get('name'))
        for item in report.get('configurations', [])
        if isinstance(item, dict)
    ]
    if len(names) < 2:
        return
    lookup = {}
    for comparison in report.get('configuration_comparisons', []):
        if not isinstance(comparison, dict):
            continue
        key = frozenset(
            (comparison.get('left_experiment'), comparison.get('right_experiment'))
        )
        shared = int(comparison.get('shared_executions', 0) or 0)
        agreement = comparison.get('bag_level_positive_agreement')
        lookup[key] = f'{ratio_text(agreement)} ({shared})' if shared else '—'
    rows = []
    for left in names:
        rows.append(
            (left,)
            + tuple(
                '—' if left == right else lookup.get(frozenset((left, right)), '—')
                for right in names
            )
        )
    lines.extend(['## Outcome agreement', ''])
    lines.append('Cells show bag-level agreement and shared run count.')
    lines.append('')
    lines.extend(markdown_table(('Configuration', *names), rows))
    lines.append('')

    if not report.get('experiment', {}).get('semantic_judge_enabled'):
        return
    semantic_rows = []
    for comparison in report.get('configuration_comparisons', []):
        judged = consistent = errors = matched = 0
        for execution in comparison.get('executions', []):
            result = execution.get('api_response_comparison', {})
            matched += int(result.get('matched_responses', 0) or 0)
            judged += int(result.get('llm_judged_responses', 0) or 0)
            consistent += int(result.get('consistent_responses', 0) or 0)
            errors += int(result.get('judge_errors', 0) or 0)
        semantic_rows.append(
            (
                f'{comparison.get("left_experiment")} vs. '
                f'{comparison.get("right_experiment")}',
                matched,
                judged,
                ratio_text(safe_ratio(consistent, judged)),
                errors,
            )
        )
    lines.extend(['### Semantic response agreement', ''])
    lines.extend(
        markdown_table(
            ('Pair', 'Matched', 'Judged', 'Agreement', 'Judge errors'),
            semantic_rows,
        )
    )
    lines.append('')


def append_compact_repeatability(lines: list[str], report: dict[str, Any]) -> None:
    """Show repeatability only when a bag actually has repeated trials."""
    rows = []
    for configuration in report.get('configurations', []):
        if not isinstance(configuration, dict):
            continue
        repeatability = configuration.get('repeatability', {})
        for item in repeatability.get('bag_consistency', []):
            trials = int(item.get('completed_trials', 0) or 0)
            if trials <= 1:
                continue
            rows.append(
                (
                    configuration.get('name'),
                    short_path(item.get('bag')),
                    trials,
                    ratio_text(item.get('positive_outcome_rate')),
                    item.get('unanimous_positive_outcome'),
                    item.get('identical_positive_decision_counts'),
                )
            )
    if rows:
        lines.extend(['## Repeatability', ''])
        lines.extend(
            markdown_table(
                (
                    'Configuration',
                    'Bag',
                    'Trials',
                    'Positive outcome rate',
                    'Same outcome',
                    'Same positive count',
                ),
                rows,
            )
        )
        lines.append('')


def append_compact_decisions(lines: list[str], report: dict[str, Any]) -> None:
    """Show one concise decision table per configuration."""
    groups: list[tuple[Any, list[tuple[Any, ...]]]] = []
    has_decisions = False
    for configuration in report.get('configurations', []):
        if not isinstance(configuration, dict):
            continue
        rows = []
        for execution in configuration.get('executions', []):
            if not isinstance(execution, dict):
                continue
            for index, decision in enumerate(
                execution.get('final_decisions', []), start=1
            ):
                if not isinstance(decision, dict):
                    continue
                rows.append(
                    (
                        short_path(execution.get('bag')),
                        execution.get('trial'),
                        index,
                        decision.get('anomaly'),
                        decision.get('severity'),
                        decision.get('action'),
                        seconds_text(decision.get('model_latency_seconds')),
                        decision.get('summary'),
                    )
                )
        has_decisions = has_decisions or bool(rows)
        groups.append((configuration.get('name'), rows))

    if has_decisions:
        lines.extend(['## Model decisions', ''])
        for name, rows in groups:
            lines.extend([f'### {name}', ''])
            if not rows:
                lines.extend(['No reportable model decisions.', ''])
                continue
            lines.extend(
                markdown_table(
                    (
                        'Bag',
                        'Trial',
                        '#',
                        'Anomaly',
                        'Severity',
                        'Action',
                        'Latency',
                        'Summary',
                    ),
                    rows,
                )
            )
            lines.append('')


def short_label(value: Any) -> str | None:
    """Shorten a sidecar-derived event identifier."""
    if not isinstance(value, str):
        return None
    marker = '.annotations.json:'
    return value.split(marker, 1)[-1] if marker in value else value


def append_compact_ground_truth(lines: list[str], report: dict[str, Any]) -> None:
    """Render event outcomes once and collapse unavailable latency summaries."""
    rows = []
    latency_rows = []
    annotated = False
    for configuration, execution in report_executions(report):
        truth = execution.get('ground_truth')
        if not isinstance(truth, dict):
            continue
        annotated = True
        for event in truth.get('events', []):
            if not isinstance(event, dict):
                continue
            label = event.get('label')
            decision = event.get('decision')
            rows.append(
                (
                    configuration.get('name'),
                    short_path(execution.get('bag')),
                    execution.get('trial'),
                    str(event.get('result', '')).replace('_', ' ').title(),
                    short_label(label.get('id')) if isinstance(label, dict) else None,
                    decision.get('action') if isinstance(decision, dict) else None,
                    seconds_text(event.get('event_difference_seconds')),
                )
            )
        processing = truth.get('processing_latency', {})
        relative = truth.get('label_relative_latency', {})
        processing_count = (
            int(processing.get('count', 0) or 0)
            if isinstance(processing, dict)
            else 0
        )
        relative_count = (
            int(relative.get('count', 0) or 0)
            if isinstance(relative, dict)
            else 0
        )
        if processing_count or relative_count:
            latency_rows.append(
                (
                    configuration.get('name'),
                    short_path(execution.get('bag')),
                    execution.get('trial'),
                    seconds_text(processing.get('mean_seconds')),
                    seconds_text(relative.get('mean_seconds')),
                )
            )
    if rows:
        lines.extend(['## Ground-truth events', ''])
        lines.extend(
            markdown_table(
                (
                    'Configuration',
                    'Bag',
                    'Trial',
                    'Result',
                    'Event',
                    'Action',
                    'Offset',
                ),
                rows,
            )
        )
        lines.append('')
    if latency_rows:
        lines.extend(['### Ground-truth latency', ''])
        lines.extend(
            markdown_table(
                (
                    'Configuration',
                    'Bag',
                    'Trial',
                    'Processing mean',
                    'Label-relative mean',
                ),
                latency_rows,
            )
        )
        lines.append('')
    elif annotated:
        lines.extend(
            [
                'Ground-truth latency was unavailable because replay observation '
                'timestamps were not retained.',
                '',
            ]
        )


def append_run_metadata(lines: list[str], report: dict[str, Any]) -> None:
    """Retain compact per-run provenance without repeating metric tables."""
    rows = []
    for configuration, execution in report_executions(report):
        metrics = execution.get('metrics')
        metrics = metrics if isinstance(metrics, dict) else {}
        rows.append(
            (
                configuration.get('name'),
                short_path(execution.get('bag')),
                execution.get('trial'),
                str(execution.get('status', '')).replace('_', ' ').title(),
                seconds_text(metrics.get('elapsed_seconds')),
                short_path(execution.get('annotation_file')),
            )
        )
    if rows:
        lines.extend(['## Run metadata', ''])
        lines.extend(
            markdown_table(
                ('Configuration', 'Bag', 'Trial', 'Status', 'Elapsed', 'Annotation'),
                rows,
            )
        )
        lines.append('')


def render_report_markdown(
    report: dict[str, Any], detail_filename: str | None = None
) -> str:
    """Render a compact comparison-first Markdown report."""
    experiment = report['experiment']
    status = str(experiment['run_status']).replace('_', ' ').title()
    lines = [
        '# Offline Anomaly-Detection Evaluation',
        '',
        f'**Status:** {status}',
        '',
    ]
    append_reliability_summary(lines, report)
    lines.extend(['## Results by configuration', ''])
    lines.extend(
        markdown_table(
            (
                'Configuration',
                'Status',
                'Responses / calls',
                'Pending',
                'TP / FN / FP',
                'Precision',
                'Recall',
                'F1',
                'Bag accuracy',
                'Mean latency',
                'Alerts',
            ),
            configuration_result_rows(report),
        )
    )
    lines.append('')
    if experiment.get('evaluation_mode') == 'unlabeled':
        lines.extend(
            [
                '> Unlabeled mode reports behavior and agreement, not accuracy.',
                '',
            ]
        )
    append_compact_issues(lines, report)
    append_configuration_parameters(lines, report)
    append_agreement_matrix(lines, report)
    append_compact_repeatability(lines, report)
    append_compact_decisions(lines, report)
    append_compact_ground_truth(lines, report)
    append_run_metadata(lines, report)

    revision = experiment.get('source_revision')
    execution_count = (
        f'{experiment["attempted_executions"]} attempted / '
        f'{experiment["expected_executions"]} expected'
    )
    lines.extend(['## Report metadata', ''])
    metadata_rows = [
        ('Mode', experiment.get('evaluation_mode')),
        ('Started (UTC)', experiment.get('started_at_utc')),
        ('Finished (UTC)', experiment.get('finished_at_utc')),
        ('Playback rate', f'{experiment.get("playback_rate")}×'),
        ('ROS domain', experiment.get('ros_domain_id')),
        (
            'Inference drain minimum',
            seconds_text(experiment.get('inference_drain_minimum_seconds')),
        ),
        ('Model preflight', experiment.get('model_preflight_enabled')),
        ('Executions', execution_count),
        ('Evaluation config', short_path(experiment.get('evaluation_yaml'))),
        ('Recordings', short_path(experiment.get('recordings_folder'))),
        ('Source revision', str(revision)[:12] if revision else None),
        ('Report schema', report.get('schema_version')),
        ('Detailed JSON', detail_filename),
    ]
    lines.extend(markdown_table(('Item', 'Value'), metadata_rows))
    lines.extend(['', '**Bags**', ''])
    lines.extend(
        markdown_table(
            ('Bag', 'Recorded duration'),
            [
                (short_path(bag.get('path')), seconds_text(bag.get('duration_seconds')))
                for bag in experiment.get('bags', [])
                if isinstance(bag, dict)
            ],
        )
    )
    lines.extend(
        [
            '',
            'Full responses, error context, detailed metrics, and diagnostics are '
            'stored in the companion JSON file.',
            '',
        ]
    )
    return '\n'.join(lines)


def atomic_write_text(path: Path, content: str) -> None:
    """Write one report artifact atomically."""
    temporary_path: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent
        )
        temporary_path = Path(temporary_name)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def write_report(report: dict[str, Any], path: Path) -> None:
    """Atomically write compact Markdown and detailed JSON reports."""
    detail_path = path.with_suffix('.json')
    try:
        rendered = render_report_markdown(report, detail_path.name)
        serialized = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
        atomic_write_text(detail_path, serialized)
        atomic_write_text(path, rendered)
    except (OSError, TypeError, ValueError) as exc:
        raise EvaluationError(f'Could not write report {path}: {exc}') from exc


def evaluated_parameters(configuration: dict[str, Any]) -> dict[str, Any]:
    """Retain comparison inputs without copying the full runtime configuration."""
    llm_value = configuration.get('llm', {})
    llm = llm_value if isinstance(llm_value, dict) else {}
    keys = (
        'provider',
        'model_provider',
        'model',
        'local',
        'ollama_host',
        'timeout_seconds',
        'vision_enabled',
        'image_context_enabled',
        'image_context_max_frames',
        'image_max_frames',
        'num_ctx',
        'num_predict',
        'image_jpeg_quality',
    )
    prompt = llm.get('system_prompt')
    prompt_hash = (
        hashlib.sha256(prompt.encode('utf-8')).hexdigest()[:12]
        if isinstance(prompt, str)
        else None
    )
    return {
        'api_frequency_seconds': configuration.get('api_frequency_seconds'),
        'cache_max_items': configuration.get('cache_max_items'),
        'cache_max_age_seconds': configuration.get('cache_max_age_seconds'),
        'llm_min_trigger_importance': configuration.get(
            'llm_min_trigger_importance'
        ),
        'llm': {
            **{key: llm.get(key) for key in keys},
            'system_prompt_hash': prompt_hash,
        },
    }


def compact_decision(decision: dict[str, Any]) -> dict[str, Any]:
    """Retain the final decision without its raw prompt context."""
    return {
        'anomaly': decision.get('anomaly'),
        'severity': decision.get('severity'),
        'action': decision.get('action'),
        'summary': decision.get('summary'),
        'artifact_api_response': decision.get('artifact_api_response'),
        'model_latency_seconds': decision.get('model_latency_seconds'),
        'error_context_messages': decision.get('error_context_messages', []),
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
        'source': truth.get('source'),
        'annotation_file': truth.get('annotation_file'),
        'bag_anomalous': truth.get('bag_anomalous'),
        'bag_classification': truth.get('bag_classification'),
        'label_count': truth.get('label_count'),
        'ignored_interval_count': truth.get('ignored_interval_count'),
        'metrics': truth.get('metrics'),
        'processing_latency': truth.get('processing_latency'),
        'label_relative_latency': truth.get('label_relative_latency'),
        'events': compact_events,
        'unscorable_decision_count': (
            len(unscorable) if isinstance(unscorable, list) else 0
        ),
        'ignored_decision_count': (
            len(truth.get('ignored_decisions', []))
            if isinstance(truth.get('ignored_decisions'), list)
            else 0
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
        'annotation_file',
        'warnings',
        'errors',
        'timeout_events',
        'graph_contamination',
        'model_preflight',
        'inference_drain',
        'backend_failures',
        'log_files',
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
    response_judge = (
        ollama_response_judge(experiments[0].aad_config)
        if (
            settings.semantic_judge_enabled
            and not settings.dry_run
            and experiments
        )
        else None
    )
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
                'randomization': experiment.randomization,
                'evaluated_parameters': evaluated_parameters(
                    experiment.aad_config
                ),
                'aggregate': aggregate_executions(
                    executions,
                    settings.mode,
                    expected_per_experiment[experiment.name],
                ),
                'repeatability': repeatability_summary(executions, response_judge),
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
            'provider_config': (
                str(settings.provider_config_path)
                if settings.provider_config_path is not None
                else None
            ),
            'recordings_folder': str(settings.bags_path),
            'playback_rate': settings.playback_rate,
            'ros_domain_id': settings.ros_domain_id,
            'model_preflight_enabled': settings.model_preflight_enabled,
            'model_preflight_timeout_seconds': (
                settings.model_preflight_timeout_seconds
            ),
            'inference_drain_minimum_seconds': (
                settings.inference_drain_timeout_seconds
            ),
            'inference_drain_model_passes': settings.inference_drain_model_passes,
            'semantic_judge_enabled': settings.semantic_judge_enabled,
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
        'configuration_comparisons': configuration_comparisons(
            grouped, response_judge
        ),
        'configurations': configuration_reports,
    }


def example_configuration() -> dict[str, Any]:
    """Return a runnable template using the repository's current AAD config."""
    base_config = (
        Path(__file__).resolve().parents[2]
        / 'anomaly_detection'
        / 'anomaly_detection'
        / 'config.yaml'
    )
    return {
        'runner': {
            'bags': '/path/to/recordings',
            'output_directory': './evaluation_results',
            'provider_config': './llm_providers.yaml',
            'mode': 'human-labeled',
            'trials': 1,
            'playback_rate': 1.0,
            'continue_on_error': True,
            'ros_domain_id': 42,
            'startup_timeout_seconds': 15.0,
            'startup_grace_seconds': 3.0,
            'post_playback_grace_seconds': 2.0,
            'inference_drain_timeout_seconds': 180.0,
            'inference_drain_model_passes': 2,
            'shutdown_timeout_seconds': 20.0,
            'graph_clean_timeout_seconds': 5.0,
            'model_preflight_enabled': True,
            'model_preflight_timeout_seconds': 120.0,
            'label_buffer_seconds': 5.0,
            'semantic_judge_enabled': False,
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
        '--provider-config',
        help='Override runner.provider_config with a provider-profile YAML path.',
    )
    parser.add_argument(
        '--trials',
        type=int,
        help='Override the number of repetitions for every configuration and bag.',
    )
    parser.add_argument(
        '--mode',
        choices=('unlabeled', 'human-labeled'),
        help=(
            'Override YAML mode; human-labeled uses adjacent '
            '*.annotations.json sidecars.'
        ),
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


def run_execution_matrix(
    bags: Sequence[BagInfo],
    experiments: Sequence[Experiment],
    settings: RunnerSettings,
    collector: EvaluationCollector | None,
    annotation_cache: dict[Path, dict[str, Any]],
    grouped: dict[str, list[dict[str, Any]]],
    log_root: Path,
) -> bool:
    """Execute the requested matrix and return whether it aborted early."""
    for experiment in experiments:
        print(f'Configuration: {experiment.name} ({experiment.config_hash[:12]})')
        for bag in bags:
            annotation_truth = annotation_cache.get(bag.path)
            for trial in range(1, experiment.trials + 1):
                print(f'  [{trial}/{experiment.trials}] {bag.path.name}')
                execution = execute_one(
                    bag,
                    experiment,
                    trial,
                    settings,
                    collector,
                    annotation_truth,
                    log_root,
                )
                grouped[experiment.name].append(execution)
                if execution['status'] not in {'failed', 'invalid_bag'}:
                    continue
                print(f"    {execution['status']}: {execution['errors']}")
                if not settings.continue_on_error:
                    return True
    return False


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

    annotation_cache: dict[Path, dict[str, Any]] = {}
    if settings.mode == 'human-labeled':
        for bag in bags:
            annotation_cache[bag.path] = load_annotation_ground_truth(bag)

    expected = sum(experiment.trials for experiment in experiments) * len(bags)
    print(
        f'Preparing {expected} execution(s): {len(bags)} bag(s) × '
        f'{len(experiments)} configuration(s), mode={settings.mode}, '
        f'ROS domain={settings.ros_domain_id}'
    )
    started = datetime.now(timezone.utc)
    report_path = reserve_report_path(settings.output_directory, started)
    log_root = report_path.with_name(f'{report_path.stem}_logs')
    revision = source_revision()
    grouped: dict[str, list[dict[str, Any]]] = {
        experiment.name: [] for experiment in experiments
    }
    collector: EvaluationCollector | None = None
    executor: SingleThreadedExecutor | None = None
    spin_thread: threading.Thread | None = None
    spin_stop = threading.Event()
    previous_ros_domain = os.environ.get('ROS_DOMAIN_ID')

    if not settings.dry_run:
        if ROS_IMPORT_ERROR is not None:
            raise EvaluationError(f'ROS 2 Python modules are unavailable: {ROS_IMPORT_ERROR}')
        os.environ['ROS_DOMAIN_ID'] = str(settings.ros_domain_id)
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

    try:
        aborted = run_execution_matrix(
            bags,
            experiments,
            settings,
            collector,
            annotation_cache,
            grouped,
            log_root,
        )
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
        if previous_ros_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_ros_domain

    if interrupted:
        run_status = 'interrupted'
    elif aborted:
        run_status = 'aborted'
    else:
        run_status = 'completed'
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
    print(f'  Details:   {report_path.with_suffix(".json")}')
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
