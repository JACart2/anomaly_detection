"""Tests for exact LLM-response correlation in offline evaluation reports."""

from dataclasses import asdict
from datetime import datetime, timezone
import importlib.util
from pathlib import Path
import sys

from response_handler import parse_llm_response


SCRIPT_PATH = Path(__file__).parents[2] / 'scripts' / 'run_offline_evaluation.py'
SPEC = importlib.util.spec_from_file_location('run_offline_evaluation', SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
OFFLINE_EVALUATION = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = OFFLINE_EVALUATION
SPEC.loader.exec_module(OFFLINE_EVALUATION)


def test_artifact_creates_one_decision_with_all_error_context() -> None:
    """One response owns every ERROR message included in its model request."""
    response = (
        '{"anomaly":true,"severity":"high","action":"stop_cart",'
        '"summary":"Two related errors require a stop."}'
    )
    artifacts = [
        {
            'artifact_id': 'api_artifact_1',
            'timestamp_ns': 2_000_000_000,
            'cached_data': [
                '[t=1.0] node=a importance=ERROR type=TEXT msg=first',
                '[t=1.1] node=b importance=ERROR type=TEXT msg=second',
                '[t=1.2] node=c importance=WARNING type=TEXT msg=context',
            ],
            'api_response': response,
        }
    ]
    calls = [{'received_at_utc_ns': 1_000_000_000}]

    decisions, warnings = OFFLINE_EVALUATION.llm_decisions_from_artifacts(
        artifacts, calls
    )

    assert warnings == []
    assert len(decisions) == 1
    assert decisions[0]['artifact_api_response'] == response
    assert decisions[0]['model_latency_seconds'] == 1.0
    assert decisions[0]['error_context_messages'] == artifacts[0]['cached_data'][:2]


def test_malformed_response_keeps_original_beside_fallback() -> None:
    """A fallback summary remains paired with the malformed response that caused it."""
    response = '{"anomaly": true, "summary": "cut off'
    artifacts = [
        {
            'artifact_id': 'api_artifact_2',
            'timestamp_ns': 2_000_000_000,
            'cached_data': [],
            'api_response': response,
        }
    ]

    decisions, warnings = OFFLINE_EVALUATION.llm_decisions_from_artifacts(
        artifacts, [{'received_at_utc_ns': 1_000_000_000}]
    )

    assert warnings == []
    assert len(decisions) == 1
    assert decisions[0]['summary'].startswith('Invalid/malformed LLM response')
    assert decisions[0]['artifact_api_response'] == response


def test_empty_backend_result_is_not_reported_as_an_llm_decision() -> None:
    """A failed call without a response cannot create an orphan summary row."""
    artifacts = [
        {
            'artifact_id': 'api_artifact_3',
            'timestamp_ns': 2_000_000_000,
            'cached_data': [],
            'api_response': '',
        }
    ]

    decisions, warnings = OFFLINE_EVALUATION.llm_decisions_from_artifacts(
        artifacts, [{'received_at_utc_ns': 1_000_000_000}]
    )

    assert decisions == []
    assert len(warnings) == 1
    assert 'contains no model response' in warnings[0]


def test_report_parser_matches_runtime_response_contract() -> None:
    """Artifact parsing must produce the same fields as the live detector."""
    responses = [
        '{"anomaly":false,"action":"none","summary":"clear"}',
        '{"anomaly":true,"action":"stop_cart","reason":"blocked"}',
        '{"summary":"missing fields"}',
        '{"anomaly": true',
    ]

    for response in responses:
        runtime = asdict(parse_llm_response(response))
        report = OFFLINE_EVALUATION.decision_from_llm_response(response)
        assert {
            key: runtime[key] for key in ('anomaly', 'severity', 'action', 'summary')
        } == report


def test_malformed_artifact_does_not_shift_later_call_latency() -> None:
    """Timestamp matching keeps later artifacts paired after a malformed file."""
    artifacts = [
        {
            '_file': 'api_artifact_1500000000.json',
            '_file_timestamp_ns': 1_500_000_000,
            '_parse_error': 'truncated',
        },
        {
            'artifact_id': 'api_artifact_3000000000',
            'timestamp_ns': 3_000_000_000,
            'cached_data': [],
            'api_response': '{"anomaly":false,"action":"none","summary":"clear"}',
        },
    ]
    calls = [
        {'received_at_utc_ns': 1_000_000_000},
        {'received_at_utc_ns': 2_000_000_000},
    ]

    decisions, warnings = OFFLINE_EVALUATION.llm_decisions_from_artifacts(
        artifacts, calls
    )

    assert len(decisions) == 1
    assert decisions[0]['model_latency_seconds'] == 1.0
    assert any('Could not parse LLM artifact' in warning for warning in warnings)
    assert not any('counts differ' in warning for warning in warnings)


def test_immediate_decisions_exclude_artifact_backed_model_output() -> None:
    """Only unmatched high-severity stop messages count as immediate decisions."""
    model_decision = {
        'anomaly': True,
        'severity': 'high',
        'action': 'stop_cart',
        'summary': 'model requested stop',
    }
    messages = [
        'anomaly=True severity=high action=stop_cart summary=hardware emergency',
        'anomaly=True severity=high action=stop_cart summary=model requested stop',
        'anomaly=False severity=low action=none summary=clear',
    ]

    immediate = OFFLINE_EVALUATION.immediate_decisions_from_topic(
        messages, [model_decision]
    )

    assert len(immediate) == 1
    assert immediate[0]['summary'] == 'hardware emergency'


def test_report_filename_uses_readable_timestamp(tmp_path: Path) -> None:
    """Report names omit the old prefix and separate date/time components."""
    started = datetime(2026, 9, 23, 21, 30, 45, tzinfo=timezone.utc)

    first = OFFLINE_EVALUATION.reserve_report_path(tmp_path, started)
    second = OFFLINE_EVALUATION.reserve_report_path(tmp_path, started)

    assert first.name == '2026-09-23_21-30-45.md'
    assert second.name == '2026-09-23_21-30-45-2.md'


def test_disabled_semantic_judge_is_not_reported_as_an_error() -> None:
    """Skipping optional semantic judging must not create false judge errors."""
    response = '{"anomaly":false,"action":"none","summary":"clear"}'
    execution = {
        'decisions': [
            {
                'artifact_api_response': response,
                'source_timestamp_candidates_ns': [1],
            }
        ]
    }

    comparison = OFFLINE_EVALUATION.api_response_comparison(
        execution, execution, response_judge=None
    )

    assert comparison['matched_responses'] == 1
    assert comparison['llm_judged_responses'] == 0
    assert comparison['judge_errors'] == 0
