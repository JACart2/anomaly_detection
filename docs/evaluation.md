# Evaluation guide

[README](../README.md) · [Development](development.md) · [Usage](usage.md) · [Evaluation](evaluation.md)

Build and source the workspace first using the README quick start. Stop any
manual detector running in the evaluation ROS domain before replay.

For detailed CLI options, experiment configuration, and troubleshooting, see the
[Offline Evaluator Instruction Manual](../system_evaluation/OFFLINE_EVALUATOR_MANUAL.md).
For the complete labeling workflow, see the
[Annotation User Guide](../system_evaluation/ANNOTATION_USER_GUIDE.md).

## Extract and annotate a recording

Run the extractor from the repository root:

```bash
cd /root/dev_ws/src/anomaly_detection
```

Standalone utility scripts that operate outside the ROS2 graph (not built/installed as part of any package).

`extract_anomaly_bag.py` reads a recorded `.mcap` bag of `AnomalyMsg` records and writes a self-contained report: `messages.json`, `messages.csv`, `summary.json`, extracted images under `images/`, and a searchable/filterable `index.html`. The HTML report also supports bag metadata, per-message labels, time-range labels, browser autosave, and JSON annotation import/export. It works locally without a server. Run with:
```
python3 system_evaluation/scripts/extract_anomaly_bag.py path/to/bag.mcap [-o output_dir]
```

The export button downloads `<bag-stem>.annotations.json`. Keep that sidecar beside
the source bag (or in another persistent dataset directory), not in the generated
report directory. For example:

```
system_evaluation/bags/driver_missing_001/
|-- driver_missing_001.mcap
|-- driver_missing_001.annotations.json
`-- report/
    |-- index.html
    |-- messages.json
    |-- messages.csv
    |-- summary.json
    `-- images/
```

The extractor recreates its output directory. As a safety measure, it refuses to
do so if it finds an `*.annotations.json` file there. The original MCAP is always
read-only. In-progress edits are keyed to the report path and bag name in browser
`localStorage`; JSON export remains the canonical dataset artifact.

Annotation files use `schema_version: 1`. Nanosecond timestamps are the primary
message identity, with topic and node name disambiguating them and message index
retained as useful provenance. The formal format is documented by
[`system_evaluation/schemas/anomaly_annotations.schema.json`](../system_evaluation/schemas/anomaly_annotations.schema.json).

## Offline Testing

Validate the selected recordings and experiments before making model calls:

```bash
cd /root/dev_ws
python3 src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml \
  --dry-run
```

Dry runs still require ROS and MCAP support to read bag metadata. Results from
actual replays go to `runner.output_directory`, normally
`system_evaluation/results/`.

A script is provided for replaying recorded bags against one or more experiments. The shared detector configuration and experiment-specific system prompts are declared in `system_evaluation/offline_evaluation.yaml`:

```bash
cd /root/dev_ws
python3 src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml
```

**Notes:**
- The executable automatically sources the detected ROS installation and nearest workspace `install/setup.bash`
- Evaluation bags live under `src/anomaly_detection/system_evaluation/bags/`
- Add or change an experiment prompt through `experiments[].overrides.llm.system_prompt`
- The examples use the `dev_ws/` directory; source the built workspace so the detector executable is available

### Evaluation annotations

Human-labeled evaluation reads the schema-v1 `<bag-stem>.annotations.json`
sidecar stored beside each `.mcap` file. Set `runner.mode: human-labeled` in
`offline_evaluation.yaml`; no label topic or dataset CSV is required. The
evaluator validates `bag.anomalous`, scores anomaly ranges and per-message
anomaly labels, and records the annotation filename in the report.

For a ROS bag directory containing `metadata.yaml`, keep the sidecar beside its
`.mcap` data file inside that directory. Standalone `.mcap` files use the same
layout shown in the annotation workflow above.

### Reading evaluation reports

The Markdown report is a compact comparison view. A positive decision is a
model response with `anomaly: true`; immediate safety decisions are counted
separately because they do not have an original model response. Model latency
appears only for captured model responses, and unlabeled-mode agreement is not
ground-truth accuracy. When semantic judging is enabled, semantic agreement
allows wording changes but flags conflicting anomaly, severity, action, or
evidence claims. Unparseable decisions have no usable boolean anomaly value.

Each Markdown report has a same-named JSON companion containing full responses,
error context, detailed metrics, warnings, and event diagnostics.

The evaluator uses the dedicated `runner.ros_domain_id` from
`offline_evaluation.yaml`, rejects pre-existing detector endpoints in that
domain, and counts detector callbacks only during the active replay window. For
local Ollama configurations it verifies that the model is installed and sends a
one-token warm-up request before replay. Drain limits are calculated from the
model timeout and allowed serial passes. If a run fails, reaches any timeout, or
records a backend error, its detector and bag-player logs are retained in a
same-timestamp `_logs` directory beside the report. Timeout stages remain
separate in the detailed JSON instead of being collapsed into one generic
timeout.
