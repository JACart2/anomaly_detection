# Batch Offline Replay Evaluation Plan

## Status

Initial implementation is available at `scripts/run_offline_evaluation.py`.

The current implementation includes the terminal and IDE quick-run workflows, YAML experiment resolution, recursive bag discovery, isolated sequential replay, detector readiness checks, artifact and decision capture, unlabeled metrics, human-labeled matching, cross-configuration agreement, and a timestamped human-readable Markdown report. Further project decisions and enhancements remain tracked below.

This project incorporates the useful evaluation concepts from the paused human-label bag comparator while adding full system replay. It must support recordings both with and without human truth labels.

## Objective

Develop a batch experiment runner that points at a folder of recorded cart runs, replays every ROS 2 bag through the anomaly-detection system, and repeats the experiment using one or more anomaly-detection configurations.

The runner must have two explicit evaluation modes:

- **Unlabeled mode:** Replays recordings without human truth labels and reports behavior, latency, reliability, consistency, and cross-configuration agreement.
- **Human-labeled mode:** Replays recordings containing human truth labels and additionally reports detection accuracy, precision, recall, F1, false positives, false negatives, and label-relative detection latency.

The runner should make it easy to compare choices such as:

- Local versus remote LLM inference.
- Different providers and models.
- Images enabled versus disabled.
- Different context sizes and context ages.
- Different system prompts.
- Different trigger, timeout, and inference settings.

Each evaluation run should produce one timestamped report containing overall results, per-configuration results, and an individual breakdown for every bag and configuration combination.

## Evaluation Modes

### Unlabeled mode

Unlabeled mode must not claim to calculate ground-truth accuracy. Without human labels, it cannot directly calculate:

- True positives.
- False positives.
- False negatives.
- Precision.
- Recall.
- F1 score.
- Anomaly-detection accuracy.

An alert count or agreement between models is not proof that an anomaly was correctly detected. Results that compare models must be described as behavior, consistency, agreement, latency, or notification quality rather than accuracy.

### Human-labeled mode

Future recordings will include timestamped human truth labels. In this mode, the runner should evaluate every replayed configuration against those labels using an inclusive time window. The initial default should be five seconds before and five seconds after each human label.

Human-labeled mode can calculate:

- True positives.
- False positives.
- False negatives.
- Precision.
- Recall.
- F1 score.
- Detection accuracy, defined as the fraction of human-labeled anomalies detected within the matching window.
- Processing and label-relative latency.

The report must always identify which mode produced a metric. Metrics from unlabeled runs must never be mixed into labeled accuracy totals.

### Mode selection

The initial interface should support explicit selection:

```text
--mode unlabeled
--mode human-labeled
```

An optional future `auto` mode may inspect configured label topics, but explicit selection is safer for the first version. In human-labeled mode, missing or unparseable truth labels should make the run incomplete rather than silently falling back to unlabeled scoring.

## Proposed Folder Layout

```text
offline_evaluation/
├── recordings/
│   ├── run_001/
│   │   ├── metadata.yaml
│   │   └── run_001_0.mcap
│   ├── run_002/
│   └── run_003.mcap
├── configs/
│   ├── local_text_only.yaml
│   ├── local_vision.yaml
│   └── remote_vision.yaml
└── results/
    └── offline_evaluation_YYYYMMDD_HHMMSS.md
```

The exact folder organization should remain configurable. The runner should recognize logical ROS bag directories and avoid treating split MCAP files as separate runs.

## Proposed Command

### Terminal workflow

The primary terminal interface should be straightforward:

```text
run_offline_evaluation \
  --bags /path/to/recordings \
  --config /path/to/offline_evaluation.yaml \
  --mode unlabeled \
  --output-dir /path/to/results
```

A starter YAML can be generated without maintaining a second checked-in template:

```text
python3 scripts/run_offline_evaluation.py \
  --write-example-config offline_evaluation.yaml
```

Configuration and bag discovery can then be checked without starting a detector or calling an LLM:

```text
python3 scripts/run_offline_evaluation.py \
  --config offline_evaluation.yaml \
  --dry-run
```

The Python file should also be directly executable:

```text
python3 run_offline_evaluation.py \
  --bags /path/to/recordings \
  --config /path/to/offline_evaluation.yaml \
  --mode human-labeled
```

CLI arguments should override corresponding runner settings from YAML or the in-file defaults. The resolved values, including where each value came from, should be stored in the report.

### Edit-and-click-run workflow

The same Python file should contain a short, clearly marked user settings block near the top. A user should be able to paste a recordings-folder path or supported link, select a YAML file, choose a mode, and click **Run** in an IDE without supplying terminal arguments.

Conceptual example:

```python
# ---------------------------------------------------------------------------
# QUICK RUN SETTINGS
# Edit these values, then run this file directly from an IDE.
# Command-line arguments override them.
# ---------------------------------------------------------------------------
QUICK_RUN_BAG_LOCATION = "/path/to/recordings"
QUICK_RUN_CONFIG = "/path/to/offline_evaluation.yaml"
QUICK_RUN_MODE = "unlabeled"  # "unlabeled" or "human-labeled"
QUICK_RUN_OUTPUT_DIRECTORY = "/path/to/results"
```

These in-file values should contain only launch conveniences:

- Recordings folder path or supported location link.
- Evaluation YAML path.
- Evaluation mode.
- Output directory.

Model, prompt, image, context, replay, timeout, and trial settings belong in YAML so experiments remain reproducible. The Python quick-run block must not become a second full configuration system.

If both CLI arguments and quick-run values are absent, the script should print a clear usage message rather than failing with an obscure path error.

Possible optional arguments:

```text
--bag-pattern
--config-pattern
--trials
--playback-rate
--startup-timeout
--inference-timeout
--shutdown-timeout
--continue-on-error
--baseline-config
--human-label-topic
--label-buffer-seconds
```

### Location/link handling

The first version must support normal local filesystem paths. If "link" means a `file://` URL, the runner may normalize it to a local path. Remote HTTP, cloud-storage, or shared-drive links require an explicit download/cache design and should not be silently treated as local folders.

The runner should validate the resolved location before launching any processes and report whether it came from the CLI, quick-run block, or YAML.

## Experiment Configuration

Configuration should be stored in YAML, consistent with other parts of this project, rather than expressed entirely through CLI arguments or Python constants. Each file should have a unique experiment name and contain the complete runner and anomaly-detector configuration needed to reproduce the run.

The recommended first-version approach is one evaluation YAML containing runner settings plus a list of experiment variants. Each variant should reuse the existing AAD configuration names wherever possible.

Example conceptual configuration:

```yaml
name: local_gemma_vision

runner:
  mode: unlabeled
  playback_rate: 1.0
  trials: 1
  continue_on_error: true
  startup_timeout_seconds: 15
  inference_timeout_seconds: 60
  shutdown_timeout_seconds: 20
  label_buffer_seconds: 5

model:
  provider: gemma
  name: gemma4:12b-it-qat
  local: true
  timeout_seconds: 35
  temperature: 0
  max_output_tokens: 96

context:
  max_items: 20
  max_age_seconds: 30
  minimum_trigger_importance: warning

images:
  enabled: true
  context_enabled: true
  maximum_frames: 2
  maximum_age_seconds: 10
  maximum_dimension: 448
  jpeg_quality: 75

inference:
  frequency_seconds: 10
  repeated_trials: 1

prompt:
  system_prompt: |
    You are an anomaly detection assistant...
```

For multiple configurations, the YAML may use a shared base plus named overrides:

```yaml
runner:
  mode: unlabeled
  trials: 2
  playback_rate: 1.0

base_config: ../anomaly_detection/anomaly_detection/config.yaml

experiments:
  - name: local_text_only
    overrides:
      llm.local: true
      llm.model: gemma4:12b-it-qat
      llm.vision_enabled: false

  - name: local_vision
    overrides:
      llm.local: true
      llm.model: gemma4:12b-it-qat
      llm.vision_enabled: true
      llm.image_max_frames: 2

  - name: remote_vision
    overrides:
      llm.local: false
      llm.model_provider: openai
      llm.model: example-model
      llm.vision_enabled: true
```

The exact override syntax must be validated before implementation. Nested YAML mappings may be preferable to dotted keys if they make merging and schema validation clearer.

The final schema should either wrap the existing AAD configuration format or map to it explicitly. Avoid maintaining two unrelated names for the same runtime setting.

### Configuration precedence

Use a documented precedence order:

```text
CLI arguments
    override YAML runner settings
        override Python quick-run settings
            override built-in safe defaults
```

Experiment model/context/image/prompt parameters should come from the resolved YAML experiment definitions. Every report must include the fully resolved configuration and a deterministic configuration hash.

Configuration validation should happen before the first bag is replayed. Unknown keys, invalid types, missing referenced files, duplicate experiment names, unsupported modes, and impossible parameter combinations should produce actionable errors.

## Configuration Dimensions

### Model

- Provider.
- Model name and version.
- Local or remote execution.
- Endpoint or local host configuration.
- Request timeout.
- Sampling temperature, when supported.
- Maximum output tokens.
- Model context-window limit.
- Structured-output settings.
- Model warmup and keep-alive settings.

### Text context

- Maximum cached items.
- Maximum cache age.
- Minimum message importance that triggers inference.
- Inference frequency.
- Duplicate-message suppression.
- Publisher rate limiting.
- Immediate-alert cooldown.

### Images

- Vision enabled or disabled.
- Routine image context enabled or disabled.
- Maximum attached images.
- Maximum image age.
- Maximum image dimension.
- JPEG quality.

When images are disabled, the runner must verify that image data is not passed to the model rather than merely changing the prompt.

### Prompt

- Complete system prompt.
- Prompt version or name.
- Prompt hash stored in the report.

### Repetition

- Number of trials per bag/configuration pair.
- Optional random seed where supported.
- Temperature and other nondeterministic sampling controls.

Repeated trials are important when comparing nondeterministic remote or local models.

## Execution Matrix

The runner evaluates the Cartesian product:

```text
bags × configurations × trials
```

For example, 20 bags, 4 configurations, and 3 trials produce 240 isolated executions.

The report must record the expected and completed execution counts. Failed and skipped executions must remain visible rather than disappearing from aggregate results.

## Execution Lifecycle

For every bag/configuration/trial combination:

1. Validate the bag and required input topics.
2. Resolve and validate the experiment configuration.
3. Create an isolated working and artifact directory.
4. Start a clean anomaly-detection process with that configuration.
5. Wait for an explicit readiness condition.
6. Start collecting output topics and runtime measurements.
7. Replay the source bag at the configured playback rate.
8. Wait for outstanding inference calls to complete, subject to a timeout.
9. Stop output recording.
10. Terminate all child processes cleanly.
11. Parse decisions, API artifacts, timestamps, and failures.
12. Save the execution result and continue through the matrix.

Each execution must start with empty detector caches and independent process state. Messages, cooldown state, and artifacts must not leak from one bag or configuration into another.

## Process Isolation and Cleanup

The runner will create multiple ROS processes and must handle them safely:

- Launch each execution in its own process group.
- Use unique ROS node names where needed.
- Consider unique ROS domain IDs if executions may overlap.
- Prefer sequential execution for the first version.
- Stop processes gracefully before escalating termination.
- Always clean up in `finally` paths.
- Keep failed-run logs and artifacts for diagnosis.
- Never delete source bags.

Timeouts should be separate for startup, bag playback, outstanding inference, and shutdown.

## Topics and Data to Capture

Candidate topics include:

- `/aad/decisions`
- `/aad/alerts`
- `/aad/llm_called`
- `/aad/formatted_messages`
- `/ai_anomaly_logging`
- The configured human truth-label topic in human-labeled mode.

The runner should also preserve API artifacts generated by the anomaly-detection node. Each detector call should ideally have a stable identifier connecting:

- Input/cache snapshot.
- Source-message timestamps.
- Image metadata.
- Raw model response.
- Normalized decision.
- Response timestamp.
- Parse or validation errors.

The current API artifact structure provides `timestamp_ns`, `cached_data`, image metadata, and `api_response`. The design should preserve these fields and add explicit timing fields if needed.

## Timestamp and Latency Model

Keep the following times distinct:

- **Source event time:** timestamp embedded in the formatted input message, such as `[t=seconds.nanoseconds ...]`.
- **Inference request time:** when the LLM call begins.
- **Inference response time:** when the LLM returns.
- **Decision publication time:** when `/aad/decisions` is published.
- **Bag record time:** when rosbag records a message.

Possible latency measurements:

```text
model_latency = inference_response_time - inference_request_time
decision_publish_latency = decision_publication_time - inference_response_time
end_to_end_latency = decision_publication_time - selected_source_event_time
```

When a decision contains several cached source messages, the system cannot automatically prove which one caused the decision. Any selected source timestamp must be identified as an attribution heuristic unless the detector begins recording explicit trigger/cause metadata.

## Metrics Available Without Ground Truth

### Performance

- Mean response latency.
- Median response latency.
- Minimum and maximum latency.
- p90 and p95 latency.
- End-to-end latency.
- Model cold-start latency.
- Bag processing duration.
- Playback real-time factor.
- Timeout count and rate.

Average latency alone is insufficient because it can conceal slow outliers.

### Detector behavior

- Positive anomaly decisions.
- Negative decisions.
- Positive-decision rate.
- Alerts per bag.
- Duplicate or closely repeated alerts.
- Time to first positive decision.
- Severity distribution.
- Action distribution.
- Decisions with image context.
- Decisions with text-only context.
- Bags that produced no decisions.

### Reliability

- LLM calls attempted.
- Successful calls.
- Failed calls.
- Malformed model responses.
- Decision-schema validation failures.
- Detector process crashes.
- Bag playback failures.
- Missing required topics.
- Inference calls unfinished at timeout.

### Cost and resource use

Where available:

- Input tokens.
- Output tokens.
- Estimated remote API cost.
- CPU time.
- Peak process memory.
- GPU memory.
- GPU utilization.

Resource collection can be deferred if it materially complicates the first version.

## Human-Labeled Evaluation

When `--mode human-labeled` is selected, the runner should extract human truth labels from each source bag before replay. The exact label topic, message type, positive-label field, and timestamp field must be configurable until the future recording format is finalized.

For a human label at timestamp `T`, a detector event is eligible when its associated source timestamp falls inside:

```text
[T - 5 seconds, T + 5 seconds]
```

The five-second buffer should remain configurable.

Matching requirements:

1. Parse and sort human labels by timestamp.
2. Parse positive detector decisions and their associated source timestamps.
3. Match labels and positive decisions one-to-one.
4. Maximize the number of matches, then minimize total timestamp distance.
5. Never reuse a label or detector decision.
6. Count every unmatched label as a false negative.
7. Count every unmatched positive decision as a false positive.
8. Classify repeated decisions near an already matched label as duplicate false positives.

Human-labeled metrics:

```text
detection_accuracy = true_positives / (true_positives + false_negatives)
precision          = true_positives / (true_positives + false_positives)
recall             = true_positives / (true_positives + false_negatives)
F1                 = 2 * true_positives /
                     (2 * true_positives + false_positives + false_negatives)
```

For each match, retain:

```text
processing_latency = decision_return_time - selected_source_event_time
label_relative_latency = decision_return_time - human_label_time
```

The report must preserve timestamp provenance and warn about missing clocks, fallbacks, or negative processing latency. If multiple cached messages could have contributed to a result, identify the selected timestamp as a matching heuristic rather than a proven causal message.

## Cross-Configuration Comparisons

For the same source bag, compare configurations using aligned source or decision timestamps and a configurable comparison window.

Questions the report should answer include:

- Did both configurations produce a positive decision around the same time?
- Did only one configuration report an anomaly?
- Which configuration returned first?
- Did image input change the boolean result?
- Did severity or action differ?
- Did repeated trials produce consistent results?
- How closely does each configuration agree with a designated baseline?

These values are agreement and consistency measurements, not accuracy.

Potential metrics:

- Pairwise decision agreement rate.
- Agreement with a named baseline configuration.
- Positive-decision count difference.
- Median latency difference.
- Trial-to-trial consistency.
- Image-enabled versus text-only decision changes.

## Notification Quality Evaluation

An optional later phase may use an independent LLM to evaluate notification messages. The judge should receive:

- Relevant recorded source messages.
- Image descriptions or images when appropriate.
- Detector decision and notification.
- A fixed structured rubric.

Suggested result:

```json
{
  "factually_consistent": true,
  "relevant_to_context": true,
  "clarity_score": 4,
  "urgency_calibration_score": 3,
  "contains_unsupported_claims": false,
  "overall_appropriate": true,
  "reason": "Concise explanation"
}
```

Safeguards:

- Keep the judge model and prompt fixed across experiments.
- Prefer a judge different from the model being evaluated.
- Hide configuration and model names from the judge.
- Randomize response order for direct comparisons.
- Store raw judge responses and parse failures.
- Manually audit a sample of judgments.
- Report judge scores separately from detector behavior metrics.

The judge cannot establish whether an anomaly was real without ground truth. This feature measures notification quality relative to the supplied context, not anomaly accuracy.

## Output Report

Each batch invocation should create one collision-safe UTC timestamped file:

```text
offline_evaluation_YYYYMMDD_HHMMSS.md
```

The Markdown report should contain:

- Experiment status, mode, timestamps, playback rate, source revision, and bags.
- Overall execution, latency, detection behavior, and reliability summaries.
- Side-by-side configuration agreement and positive-decision counts.
- Trial-to-trial averages, standard deviations, ranges, and outcome consistency.
- The key model, cache, and image parameters varied by each configuration.
- Detailed aggregate and individual run tables.
- Warnings, failures, and human-label metrics when available.

Raw periodic messages, API artifacts, image payloads, and process logs should remain omitted from the report. The evaluator may retain these internally while calculating metrics. In human-labeled mode, the report should show aggregate and per-configuration accuracy results. In unlabeled mode, it must prominently explain that behavior and agreement are not accuracy.

## Recommended Initial Release

Version one should include:

- A terminal CLI and a direct IDE click-run workflow using the same Python entry point.
- A small quick-run path/mode block at the top of the Python file.
- YAML-based runner and experiment configuration with validation and overrides.
- Recursive bag discovery.
- Multiple YAML configurations.
- Local and remote model selection through existing AAD configuration.
- Images enabled or disabled.
- Configurable text and image context.
- Configurable system prompts.
- Sequential isolated replay execution.
- Decision and artifact capture.
- Latency, behavior, and reliability metrics.
- Per-bag and per-configuration breakdowns.
- Cross-configuration agreement.
- Explicit unlabeled and human-labeled modes.
- Human-label extraction and ±5-second one-to-one matching when labeled mode is selected.
- Accuracy, precision, recall, F1, and false-positive/false-negative reporting in labeled mode.
- One timestamped human-readable Markdown report.

Defer initially:

- LLM notification judging.
- Detailed hardware profiling.
- Parallel experiment execution.
- Automatic prompt optimization.

## Open Decisions

1. Should experiment YAML files be complete copies of the existing AAD config or small override files applied to a base config?
2. Which ROS topic or readiness signal confirms that the detector can safely receive replayed messages?
3. Which topics must every input bag contain?
4. Should bags replay at recorded speed or as quickly as the detector can process them?
5. How should the runner determine that all outstanding LLM calls have finished?
6. Should each trial use a new ROS domain ID or is sequential process isolation sufficient?
7. Which configuration should act as the baseline for agreement comparisons?
8. Should raw output bags be retained, or should the Markdown report and API artifacts be sufficient?
9. Which model usage and cost metadata are available from local and remote backends?
10. When should notification-quality judging enter the project roadmap?
11. What topic, ROS message type, and field schema will future human truth labels use?
12. Will human labels be stored in the same replay bag or in a separate timestamp-aligned bag?
13. Does a pasted "link" mean a local path, `file://` URL, or a remote/cloud URL requiring download support?
14. Should the evaluation YAML contain nested experiment overrides or reference one complete AAD YAML per experiment?
