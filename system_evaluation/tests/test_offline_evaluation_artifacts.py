"""Tests for exact LLM-response correlation in offline evaluation reports."""

import importlib.util
import json
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

from response_handler import parse_llm_response


SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'run_offline_evaluation.py'
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


def test_adjacent_annotation_sidecar_supplies_range_and_message_truth(
    tmp_path: Path,
) -> None:
    """The evaluator loads schema-v1 truth beside a standalone MCAP file."""
    bag_path = tmp_path / 'recording.mcap'
    bag_path.touch()
    sidecar = tmp_path / 'recording.annotations.json'
    sidecar.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'bag': {'file': bag_path.name, 'anomalous': True},
                'ranges': [
                    {
                        'start_timestamp_ns': 100,
                        'end_timestamp_ns': 200,
                        'label': 'anomaly',
                        'category': 'Sensor issue',
                        'description': 'sensor unavailable',
                    },
                    {
                        'start_timestamp_ns': 300,
                        'end_timestamp_ns': 350,
                        'label': 'ignore',
                        'category': '',
                        'description': 'setup period',
                    },
                ],
                'annotations': [
                    {
                        'message_index': 1,
                        'recorded_timestamp_ns': 150,
                        'topic': '/logs',
                        'node_name': 'camera',
                        'label': 'anomaly',
                        'category': 'Sensor issue',
                        'description': 'covered by the range',
                    },
                    {
                        'message_index': 2,
                        'recorded_timestamp_ns': 250,
                        'topic': '/logs',
                        'node_name': 'camera',
                        'label': 'anomaly',
                        'category': 'Sensor issue',
                        'description': 'separate event',
                    },
                ],
            }
        ),
        encoding='utf-8',
    )
    bag = OFFLINE_EVALUATION.BagInfo(
        path=bag_path,
        topics={'/logs': 'anomaly_msg/msg/AnomalyLog'},
        start_ns=50,
        end_ns=400,
        duration_seconds=0.00000035,
    )

    truth = OFFLINE_EVALUATION.load_annotation_ground_truth(bag)

    assert truth['source'] == str(sidecar)
    assert truth['errors'] == []
    assert truth['bag_anomalous'] is True
    assert len(truth['labels']) == 2
    assert truth['labels'][0]['timestamp_source'] == 'annotation_range'
    assert truth['labels'][1]['timestamp_ns'] == 250
    assert len(truth['ignored_intervals']) == 1


def test_annotation_ranges_drive_temporal_scoring_and_ignore_ranges() -> None:
    """A decision inside an anomaly range matches; ignored-only context is omitted."""
    labels = [
        {
            'id': 'recording.annotations.json:range:1',
            'timestamp_ns': 100,
            'start_timestamp_ns': 100,
            'end_timestamp_ns': 200,
            'timestamp_source': 'annotation_range',
        }
    ]
    decisions = [
        {'anomaly': True, 'source_timestamp_candidates_ns': [175]},
        {'anomaly': True, 'source_timestamp_candidates_ns': [325]},
    ]
    ignored = [
        {
            'timestamp_ns': 300,
            'start_timestamp_ns': 300,
            'end_timestamp_ns': 350,
        }
    ]

    result = OFFLINE_EVALUATION.evaluate_ground_truth(
        labels,
        decisions,
        buffer_seconds=0,
        playback_rate=1,
        ignored_intervals=ignored,
        bag_anomalous=True,
    )

    assert result['metrics']['true_positives'] == 1
    assert result['metrics']['false_positives'] == 0
    assert result['metrics']['ignored_positive_decisions'] == 1
    assert result['events'][0]['event_difference_seconds'] == 0
    assert result['bag_classification']['result'] == 'true_positive'


def test_unfinished_annotation_sidecar_is_not_scorable(tmp_path: Path) -> None:
    """A sidecar must have a final bag-level Yes/No value before evaluation."""
    bag_path = tmp_path / 'recording.mcap'
    bag_path.touch()
    (tmp_path / 'recording.annotations.json').write_text(
        json.dumps(
            {
                'schema_version': 1,
                'bag': {'file': bag_path.name, 'anomalous': None},
                'ranges': [],
                'annotations': [],
            }
        ),
        encoding='utf-8',
    )
    bag = OFFLINE_EVALUATION.BagInfo(
        path=bag_path,
        topics={},
        start_ns=1,
        end_ns=2,
        duration_seconds=0.000000001,
    )

    truth = OFFLINE_EVALUATION.load_annotation_ground_truth(bag)

    assert any(
        'bag.anomalous must be true or false' in error
        for error in truth['errors']
    )


def test_ros_bag_directory_uses_sidecar_beside_its_mcap(tmp_path: Path) -> None:
    """A metadata.yaml bag directory finds the sidecar next to its data file."""
    bag_directory = tmp_path / 'recording'
    bag_directory.mkdir()
    mcap = bag_directory / 'recording_0.mcap'
    mcap.touch()
    sidecar = bag_directory / 'recording_0.annotations.json'
    sidecar.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'bag': {'file': mcap.name, 'anomalous': False},
                'ranges': [],
                'annotations': [],
            }
        ),
        encoding='utf-8',
    )
    bag = OFFLINE_EVALUATION.BagInfo(
        path=bag_directory,
        topics={},
        start_ns=1,
        end_ns=2,
        duration_seconds=0.000000001,
    )

    truth = OFFLINE_EVALUATION.load_annotation_ground_truth(bag)

    assert truth['source'] == str(sidecar)
    assert truth['errors'] == []
    assert truth['bag_anomalous'] is False


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


def test_semantic_judge_caches_equivalent_response_pair(monkeypatch) -> None:
    """Repeated and reversed comparisons should share one model judgment."""
    requests = []

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        @staticmethod
        def read() -> bytes:
            return json.dumps(
                {
                    'message': {
                        'content': json.dumps(
                            {'consistent': True, 'rationale': 'same outcome'}
                        )
                    }
                }
            ).encode()

    def fake_urlopen(request, timeout):
        requests.append((request, timeout))
        return FakeResponse()

    monkeypatch.setattr(OFFLINE_EVALUATION, 'urlopen', fake_urlopen)
    judge = OFFLINE_EVALUATION.ollama_response_judge(
        {
            'llm': {
                'local': True,
                'model': 'test-model',
                'ollama_host': 'http://localhost:11434',
            }
        }
    )
    assert judge is not None

    assert judge('left', 'right')['consistent'] is True
    assert judge('right', 'left')['consistent'] is True
    assert len(requests) == 1


def compact_report_fixture() -> dict:
    """Return a small report containing the details the compact view suppresses."""
    def configuration(name: str, latency: float) -> dict:
        return {
            'name': name,
            'configuration_hash': f'{name}-hash',
            'randomization': None,
            'evaluated_parameters': {
                'api_frequency_seconds': 10,
                'cache_max_items': 20,
                'cache_max_age_seconds': 30,
                'llm_min_trigger_importance': 'warning',
                'llm': {
                    'provider': 'local',
                    'model': 'test-model',
                    'num_ctx': 2048 if name == 'baseline' else 4096,
                    'system_prompt_hash': name,
                },
            },
            'aggregate': {
                'executions_expected': 1,
                'executions_completed': 1,
                'executions_failed': 0,
                'executions_invalid': 0,
                'model_response_latency': {
                    'count': 1,
                    'mean_seconds': latency,
                },
                'behavior': {
                    'positive_decisions': 1,
                    'negative_decisions': 0,
                    'unparseable_decisions': 0,
                    'alerts': 1,
                },
                'reliability': {
                    'llm_calls': 2,
                    'pending_llm_calls_at_timeout': 1,
                },
                'ground_truth_metrics': {
                    'true_positives': 1,
                    'false_negatives': 0,
                    'false_positives': 0,
                    'precision': 1.0,
                    'recall': 1.0,
                    'f1': 1.0,
                    'bag_accuracy': 1.0,
                },
            },
            'repeatability': {
                'bag_consistency': [
                    {
                        'bag': '/tmp/bags/example',
                        'completed_trials': 1,
                        'positive_outcome_rate': 1.0,
                        'unanimous_positive_outcome': True,
                        'identical_positive_decision_counts': True,
                    }
                ],
                'api_response_consistency': [],
            },
            'executions': [
                {
                    'bag': '/tmp/bags/example',
                    'trial': 1,
                    'status': 'completed_with_timeout',
                    'annotation_file': '/tmp/bags/example.annotations.json',
                    'timeout_events': [
                        {'kind': 'inference_drain', 'limit_seconds': 180.0},
                        *(
                            [{'kind': 'model_request', 'limit_seconds': 90.0}]
                            if name == 'baseline'
                            else []
                        ),
                    ],
                    'warnings': [
                        'Inference drain timed out; pending work is recorded.',
                        'Artifact-file/LLM-call counts differ; responses were partial.',
                    ],
                    'errors': [],
                    'metrics': {'elapsed_seconds': 12.0},
                    'final_decisions': [
                        {
                            'anomaly': True,
                            'severity': 'high',
                            'action': 'stop_cart',
                            'summary': 'Obstacle detected.',
                            'model_latency_seconds': latency,
                            'artifact_api_response': 'RAW RESPONSE MUST STAY IN JSON',
                            'error_context_messages': ['verbose input context'],
                        }
                    ],
                    'ground_truth': {
                        'annotation_file': '/tmp/bags/example.annotations.json',
                        'events': [
                            {
                                'result': 'true_positive',
                                'label': {
                                    'id': 'example.annotations.json:range:1',
                                    'timestamp_ns': 1,
                                },
                                'decision': {
                                    'action': 'stop_cart',
                                    'summary': 'Obstacle detected.',
                                },
                                'event_difference_seconds': 0,
                            }
                        ],
                        'processing_latency': {'count': 0},
                        'label_relative_latency': {'count': 0},
                    },
                }
            ],
        }

    return {
        'schema_version': 12,
        'experiment': {
            'run_status': 'completed',
            'evaluation_mode': 'human-labeled',
            'semantic_judge_enabled': False,
            'started_at_utc': '2026-10-01T00:00:00+00:00',
            'finished_at_utc': '2026-10-01T00:01:00+00:00',
            'playback_rate': 1.0,
            'attempted_executions': 2,
            'expected_executions': 2,
            'evaluation_yaml': '/tmp/config/offline_evaluation.yaml',
            'recordings_folder': '/tmp/bags/example',
            'source_revision': '1234567890abcdef',
            'bags': [{'path': '/tmp/bags/example', 'duration_seconds': 10.0}],
        },
        'overall': {
            'behavior': {
                'positive_decisions': 2,
                'negative_decisions': 0,
                'unparseable_decisions': 0,
            },
            'reliability': {
                'llm_calls': 4,
                'pending_llm_calls_at_timeout': 2,
            },
        },
        'configuration_comparisons': [
            {
                'left_experiment': 'baseline',
                'right_experiment': 'variant',
                'shared_executions': 1,
                'bag_level_positive_agreement': 1.0,
                'executions': [],
            }
        ],
        'configurations': [
            configuration('baseline', 2.0),
            configuration('variant', 3.0),
        ],
    }


def test_compact_report_omits_repeated_and_verbose_sections() -> None:
    """The human-readable report stays short while retaining key comparisons."""
    rendered = OFFLINE_EVALUATION.render_report_markdown(
        compact_report_fixture(), 'report.json'
    )

    assert 'Results by configuration' in rendered
    assert '2 of 4 LLM call(s) produced reportable responses' in rendered
    assert '2 run(s) hit the inference-drain timeout' in rendered
    assert '1 run event(s) hit the model request timeout' in rendered
    assert 'Outcome agreement' in rendered
    assert 'Ground-truth latency was unavailable' in rendered
    assert '### baseline' in rendered
    assert '### variant' in rendered
    assert rendered.count('| Bag | Trial | # | Anomaly |') == 2
    assert 'Repeatability' not in rendered
    assert 'LLM-judged API-response consistency' not in rendered
    assert 'Reading this report' not in rendered
    assert 'Original LLM response' not in rendered
    assert 'RAW RESPONSE MUST STAY IN JSON' not in rendered
    assert '/tmp/' not in rendered
    assert len(rendered.splitlines()) < 150


def test_report_writes_verbose_details_to_json_companion(tmp_path: Path) -> None:
    """Full responses remain available without bloating the Markdown report."""
    report_path = tmp_path / 'report.md'

    OFFLINE_EVALUATION.write_report(compact_report_fixture(), report_path)

    rendered = report_path.read_text(encoding='utf-8')
    details = json.loads(report_path.with_suffix('.json').read_text(encoding='utf-8'))
    assert 'report.json' in rendered
    assert 'RAW RESPONSE MUST STAY IN JSON' not in rendered
    assert (
        details['configurations'][0]['executions'][0]['final_decisions'][0][
            'artifact_api_response'
        ]
        == 'RAW RESPONSE MUST STAY IN JSON'
    )


def test_drain_timeout_covers_two_model_passes_and_shutdown() -> None:
    """The effective drain cannot be shorter than queued serial model work."""
    settings = SimpleNamespace(
        inference_drain_timeout_seconds=180.0,
        inference_drain_model_passes=2,
        shutdown_timeout_seconds=20.0,
    )

    timeout = OFFLINE_EVALUATION.effective_inference_drain_timeout(
        {'llm': {'timeout_seconds': 90}}, settings
    )

    assert timeout == 200.0


def test_graph_contamination_detects_existing_detector_and_input_publishers() -> None:
    """Reject a shared domain before replay can affect an existing detector."""
    endpoint = SimpleNamespace(node_name='existing', node_namespace='/')
    collector = SimpleNamespace(
        settings=SimpleNamespace(
            decision_topic='/aad/decisions',
            alert_topic='/aad/alerts',
            llm_called_topic='/aad/llm_called',
            formatted_topic='/aad/formatted_messages',
        ),
        get_publishers_info_by_topic=lambda topic: (
            [endpoint] if topic in ('/aad/decisions', '/input') else []
        ),
        get_subscriptions_info_by_topic=lambda topic: [endpoint],
        _graph_endpoint=OFFLINE_EVALUATION.EvaluationCollector._graph_endpoint,
    )

    contaminants = OFFLINE_EVALUATION.EvaluationCollector.graph_contamination(
        collector, '/input'
    )

    assert {(item['role'], item['topic']) for item in contaminants} == {
        ('publisher', '/aad/decisions'),
        ('publisher', '/input'),
        ('subscription', '/input'),
    }


def test_local_model_preflight_selects_installed_host_and_warms(monkeypatch) -> None:
    """Preflight checks model inventory before sending a minimal warm-up request."""
    requests = []

    class FakeResponse:
        def __init__(self, body: dict):
            self.body = body

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self) -> bytes:
            return json.dumps(self.body).encode()

    def fake_urlopen(request, timeout):
        requests.append((request.full_url, timeout))
        if request.full_url.endswith('/api/tags'):
            return FakeResponse({'models': [{'name': 'test-model'}]})
        return FakeResponse({'message': {'content': '{}'}})

    monkeypatch.setattr(OFFLINE_EVALUATION, 'urlopen', fake_urlopen)
    settings = SimpleNamespace(
        model_preflight_enabled=True,
        model_preflight_timeout_seconds=12.0,
    )

    result = OFFLINE_EVALUATION.preflight_model(
        {
            'llm': {
                'local': True,
                'model': 'test-model',
                'ollama_host': 'http://127.0.0.1:11434',
            }
        },
        settings,
    )

    assert result['status'] == 'passed'
    assert result['host'] == 'http://127.0.0.1:11434'
    assert [url for url, _ in requests] == [
        'http://127.0.0.1:11434/api/tags',
        'http://127.0.0.1:11434/api/chat',
    ]


def test_failure_logs_are_retained_and_model_timeout_is_classified(
    tmp_path: Path,
) -> None:
    """Backend timeout details survive temporary execution-directory cleanup."""
    detector_log = tmp_path / 'detector.log'
    player_log = tmp_path / 'player.log'
    detector_log.write_text(
        '[AAD] LLM call failed. See: request timed out\n', encoding='utf-8'
    )
    player_log.write_text('playback complete\n', encoding='utf-8')
    result = {
        'execution_id': 'baseline:bag:trial-1',
        'status': 'completed',
        'warnings': [],
        'timeout_events': [],
        '_runtime_configuration': {'llm': {'timeout_seconds': 90}},
    }

    OFFLINE_EVALUATION.retain_execution_logs(
        result,
        detector_log,
        player_log,
        tmp_path / 'report_logs',
    )

    assert result['timeout_events'] == [
        {'kind': 'model_request', 'limit_seconds': 90.0, 'failures': 1}
    ]
    assert len(result['log_files']) == 2
    assert all(Path(path).is_file() for path in result['log_files'])
    assert '_runtime_configuration' not in result
