# Batch Anomaly Bag Comparison Plan

## Status

Planning only. No comparator implementation has been started.

## Objective

Build an offline evaluation script that scans a folder containing many ROS 2 bags, pairs each human-labeled bag with its detector-output counterpart using timestamps, and reports:

- How many human-labeled anomalies were detected.
- How many were missed.
- How many detector results were false positives.
- How long the detector took to return a positive result after the relevant anomaly message occurred.

Version one evaluates only the boolean anomaly decision. It does not grade action, severity, category, or summary text.

## Confirmed Timestamp Format

Existing API artifacts confirm that relevant timestamps are available even when embedded in strings. An artifact contains a top-level response time:

```json
"timestamp_ns": 1784050884027865409
```

Its cached source messages contain their original timestamps:

```text
[t=1784050861.875419090 frame=collision_detector] node=... msg=...
```

These are different moments and must remain separate:

- **Event timestamp:** source-message time parsed from `[t=<seconds>.<nanoseconds> ...]` or an equivalent timestamp embedded in a string.
- **Return timestamp:** when the detector response was produced, preferably the artifact's `timestamp_ns`.
- **Bag-record timestamp:** when rosbag recorded a message; retained as provenance and used only as a documented fallback.

Parse all timestamps as integer nanoseconds to avoid floating-point precision loss.

## Folder Input

The required workflow processes one folder containing many bags:

```text
compare_anomaly_bags \
  --bag-dir /path/to/evaluation_bags \
  --buffer-seconds 5 \
  --output-dir /path/to/results
```

The scanner should recognize ROS bag directories containing `metadata.yaml` and one or more MCAP files, standalone MCAP files where supported, and nested bag directories. It must not treat split MCAP files inside one ROS bag as separate runs.

An optional explicit two-bag mode may be useful for debugging, but folder-wide evaluation is the required first-version workflow.

## Bag Roles

Classify every discovered bag as:

- **Human-labeled:** contains the configured human-label topic or message pattern.
- **Detector-output:** contains detector results and source-message context.
- **Unknown:** lacks enough information to assign a role.
- **Ambiguous:** satisfies both roles or contains conflicting evidence.

Role detection should use topics and message contents, not filenames. Filenames may appear in reports but should not determine pairing.

The human-label topic, message type, and positive-label representation still need confirmation before implementation.

## Timestamp Extraction

### Human labels

For each positive human label, use this precedence:

1. Timestamp embedded in the label message string.
2. Message header timestamp, if present.
3. Bag-record timestamp as a fallback.

Record the selected source and raw value in the report.

### Detector event time

For each detector result:

1. Parse `[t=seconds.nanoseconds ...]` values from its associated cached/formatted messages.
2. If only one relevant source timestamp exists, use it directly.
3. If multiple cached messages are associated with one result, preserve every candidate timestamp for matching.
4. Fall back to a header or bag-record timestamp only if no embedded source time exists, and flag that fallback.

### Detector return time

Use this precedence:

1. Explicit result or artifact `timestamp_ns`.
2. Timestamp embedded in the returned-result message, if present.
3. Bag-record timestamp of the returned decision.

## Automatic Bag Pairing

Bag pairing must be one-to-one and based on extracted time ranges. For every bag, derive:

- Earliest and latest embedded event timestamps.
- Earliest and latest bag-record timestamps.
- Recording start and duration from `metadata.yaml`, when available.
- Bag role and relevant message counts.

Pair a human-labeled bag with the detector-output bag whose embedded event-time interval has the strongest overlap:

1. Prefer the largest embedded-time overlap.
2. Break ties using the smallest difference between interval start times.
3. Use bag-record intervals only when embedded ranges are unavailable.
4. Require a configurable minimum overlap or maximum start-time difference.

Do not silently guess when two candidates score similarly. Ambiguous, missing, or reused counterparts should be reported as pairing errors and excluded from aggregate accuracy until resolved.

The pairing report should show both bags, their time ranges, overlap duration, selection reason, and all unpaired bags before event scoring begins.

## Event Matching

For a human label at event time `T`, the valid detector-event window is:

```text
[T - 5 seconds, T + 5 seconds]
```

Five seconds is the default and remains configurable.

Within each paired run:

1. Extract and sort positive human labels by event timestamp.
2. Parse detector results and retain only boolean-positive results (`anomaly=true`).
3. Associate each positive result with its candidate cached/source-message timestamps.
4. A result is eligible when at least one associated event timestamp falls in the label's window.
5. Match labels and positive results one-to-one.
6. When windows overlap, choose assignments minimizing overall absolute event-time difference.
7. Never reuse a label or detector result.

Record the selected source timestamp so it is clear which cached message connected the human label to the detector result.

## Classification

- **True positive:** A positive detector result has an associated source-message timestamp inside the human label's window.
- **False negative:** No positive result matches the human label.
- **False positive:** A positive result matches no human label.
- **True-negative pair:** A paired run contains neither human labels nor positive detector results.

Repeated positive results around one label should be identified as duplicates when possible. Whether duplicates also reduce precision remains an open policy decision.

## Detection Latency

For every true positive, calculate:

```text
processing_latency = detector_return_timestamp - matched_detector_event_timestamp
```

This compares the source message that identified the anomaly against the time the positive result was returned.

Also retain:

```text
label_relative_latency = detector_return_timestamp - human_label_timestamp
```

The processing metric measures detector turnaround from the relevant source message. The label-relative metric shows timing relative to the human label, including possible human reaction delay.

Never clamp negative values. Negative processing latency normally indicates a parsing, association, or clock problem and should generate a warning. Negative label-relative latency can occur legitimately because matching allows five seconds before the human label.

Aggregate statistics should include mean, median, minimum, maximum, and p95 when enough samples exist.

## Metrics

Report per-pair and folder-wide totals:

- Bags discovered by role
- Successfully paired bags
- Unpaired or ambiguous bags
- Human-labeled anomaly count
- Correctly detected count
- Missed count
- Positive detector-result count
- False-positive count
- Duplicate count, if distinguishable
- Overall detection accuracy
- Precision, recall, and F1
- Processing-latency statistics
- Label-relative-latency statistics

```text
overall_detection_accuracy = true_positives / (true_positives + false_negatives)
precision = true_positives / (true_positives + false_positives)
recall    = true_positives / (true_positives + false_negatives)
F1        = 2 * precision * recall / (precision + recall)
```

For version one, **overall detection accuracy** means the percentage of human-labeled anomalies that received a matching positive detector result. It is therefore numerically equal to recall. This definition is used because continuous bag data does not provide a finite set of negative events from which conventional classification accuracy could be calculated. False positives remain a separate reported metric. Undefined metrics should be reported as unavailable.

## Output

Each run must create one timestamped report file in the output directory:

```text
anomaly_evaluation_YYYYMMDD_HHMMSS.json
```

The timestamp should be UTC and represent when evaluation began. If a file with that name already exists, the script must add a collision-safe suffix instead of overwriting it.

The report must be self-contained and ordered conceptually as:

1. Run metadata and configuration.
2. Overall folder accuracy and aggregate metrics.
3. Individual breakdown for every bag-file pair.
4. Unpaired/ambiguous bags and processing warnings.

The terminal may show progress, including pairing results:

```text
PAIR  human_run_001 <-> detector_run_001  overlap=302.4s
PAIR  human_run_002 <-> detector_run_002  overlap=184.1s
ERROR human_run_003 has no unambiguous detector counterpart
```

The top-level `overall` section of the file should contain:

```text
Bag pairs evaluated:    18
Human anomalies:        42
Correctly detected:     37
Missed:                  5
False positives:         4
Overall accuracy:      88.1%
Detection precision:   90.2%
Median processing time: 1.42 s
```

Each entry in the `bag_pairs` section should identify the human-labeled bag and detector-output bag, then provide that pair's:

- Pairing timestamps and overlap information.
- Human-label count.
- Correctly detected and missed counts.
- False-positive and duplicate counts.
- Pair accuracy, precision, recall, and F1.
- Processing- and label-relative-latency statistics.
- Full per-event matching records.
- Pair-specific warnings or parsing failures.

Proposed per-event fields within each pair:

```text
human_bag
detector_bag
human_label_timestamp_ns
human_timestamp_source
window_start_ns
window_end_ns
matched
detector_event_timestamp_ns
detector_event_timestamp_source
detector_return_timestamp_ns
detector_return_timestamp_source
event_time_difference_seconds
processing_latency_seconds
label_relative_latency_seconds
result
raw_label
raw_detector_context
raw_detector_response
```

The single report should preserve enough raw data and timestamp provenance to audit every aggregate result. Optional CSV exports may be considered later, but they are not part of the required first-version output.

## Validation and Failure Handling

Clearly report:

- Malformed or missing embedded timestamps.
- Multiple timestamps associated with one result.
- Bags with no relevant topics.
- Non-overlapping time ranges.
- Ambiguous pairing candidates.
- Duplicate positive decisions.
- Positive responses without source-message timestamps.
- Mixed clocks or implausible negative processing latency.
- Empty folders and partially written bags.

Parsing failures must not silently become false negatives or false positives.

## Version-One Scope

Included:

- Recursive folder scanning for many bags
- Content- and timestamp-based bag role detection
- Automatic one-to-one pairing based on time overlap
- Embedded timestamp parsing from string messages
- Configurable ±5-second event matching
- Boolean anomaly evaluation
- Processing latency from source message to returned result
- Label-relative latency as a diagnostic
- One timestamped report file containing folder-wide accuracy and per-pair breakdowns

Excluded:

- Correct-action evaluation
- Severity evaluation
- Category or summary matching
- Starting or stopping the detector
- Replaying bags
- Calling an LLM

## Relationship to Existing Code

`run_aad_config_tests.py` launches the detector, replays bags, and treats any alert in an anomalous bag as correct. The proposed comparator should be separate because it evaluates previously recorded bags and performs timestamp-based pairing and event matching.

The API artifact structure supplies the initial timestamp contract:

- `cached_data[*]` strings contain source times such as `[t=1784050861.875419090 ...]`.
- `timestamp_ns` records artifact/response time.
- `api_response` provides the boolean `anomaly` result.

## Open Decisions

Before implementation, confirm:

1. Human-label topic, ROS message type, and exact positive-label format.
2. How API artifact data appears inside detector-output bags: one JSON string topic, separate topics, or another message type.
3. Minimum time overlap required to pair two bags.
4. Ambiguity tolerance when two detector bags overlap one labeled bag similarly.
5. Which cached timestamp represents a positive result when several fall inside a label window; the current proposal chooses the timestamp closest to the human label.
6. Whether duplicate positive results reduce precision or are reported separately only.
