# Offline evaluator instruction manual

The offline evaluator replays recorded ROS 2 bags through the anomaly detector,
compares YAML-defined experiments, and writes Markdown and JSON results. Each
bag, resolved configuration, and trial gets a fresh detector process. The runner
uses temporary runtime configurations; experiment overrides do not modify the
production configuration.

## 1. Prepare the environment

Use the ROS 2 workspace containing this repository (normally `/root/dev_ws`).
Install the project's runtime dependencies, including PyYAML, ROS 2 Python
modules (`rclpy`, `rosbag2_py`), and MCAP bag storage support. Build the
`anomaly_msg` and `anomaly_detection` packages with the workspace's normal build
procedure. Confirm that the installed detector matches the source you intend to
evaluate:

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg executables anomaly_detection
```

The last command should list `anomaly_detection_node`. Substitute your ROS
distribution if it differs. The evaluator automatically sources a detected ROS
installation and the nearest workspace `install/setup.bash`, but cannot build
missing packages or install dependencies. Run from the workspace root, as in the
commands below, to match the project's normal runtime setup.

For local inference, start Ollama and install the exact `llm.model` selected by
the base configuration or experiment override:

```bash
ollama list
ollama pull <model-name>
```

For cloud inference, select a profile from
[`llm_providers.yaml`](llm_providers.yaml), export its credential environment
variable, and set `llm.provider`, `llm.model`, and `llm.local: false` in the
experiment. For example, the `anthropic` profile uses `ANTHROPIC_API_KEY`.
Keep actual credentials out of YAML. “Offline” refers to replaying recorded
inputs; cloud experiments still make network calls and may incur provider costs.

## 2. Select and label recordings

Recordings and generated results are local data and are not shipped with the
evaluator. Create `system_evaluation/bags/` and add your recordings before running,
or point the configuration at an external dataset directory.

Set `runner.bags` in [`offline_evaluation.yaml`](offline_evaluation.yaml) to:

- A ROS bag directory containing `metadata.yaml`, to select one recording.
- A parent directory, to recursively discover bag directories and standalone
  `.mcap` files. MCAP files already inside discovered bag directories are not
  counted again.

The input selection must be a **directory**, including when selecting standalone
MCAP recordings. Each bag must contain the detector's configured
`raw_input_topic` (normally `/ai_anomaly_logging`) with compatible message types.

Choose one evaluation mode:

- `human-labeled`: scores against adjacent schema-v1 annotation sidecars.
- `unlabeled`: compares model behavior and repeatability without ground-truth
  accuracy. Sidecars are not required.

For labeled evaluation, use the extraction and annotation workflow described in
the [repository README](../README.md#system_evaluationscripts):

```bash
python3 src/anomaly_detection/system_evaluation/scripts/extract_anomaly_bag.py \
  path/to/recording.mcap -o path/to/report
```

Open the generated `index.html`, fill in bag metadata, label anomaly ranges or
messages, and export the annotations. Save the exported sidecar beside its MCAP:

```text
bags/scenario_001/
├── metadata.yaml                         # When recorded as a ROS bag directory
├── scenario_001.mcap
├── scenario_001.annotations.json
└── report/
    └── index.html
```

Keep sidecars outside the generated report directory. Browser autosave alone is
not an evaluation input. See the
[annotation schema](schemas/anomaly_annotations.schema.json) for required fields.
`bag.anomalous` must be a boolean for evaluation; `null` is not a completed label.
Anomaly labels contradicting `bag.anomalous: false` invalidate the bag. If an
anomalous bag has no anomaly ranges or messages, the evaluator treats the entire
bag as one truth interval and reports a warning. Message anomalies already
covered by anomaly ranges are not added as duplicate truth events. `uncertain`
and `ignore` labels define ignored intervals.

## 3. Configure experiments

Edit `offline_evaluation.yaml`. Relative YAML paths are resolved from that
file's directory. Important settings are:

| Setting | Purpose |
| --- | --- |
| `base_config` | Production detector YAML used as the base for experiments. |
| `runner.bags` | Recording directory to evaluate. |
| `runner.output_directory` | Destination for reports. |
| `runner.provider_config` | Provider endpoint/authentication profiles. |
| `runner.mode` | `human-labeled` or `unlabeled`. |
| `runner.trials` | Default repetitions per configuration and bag. |
| `runner.playback_rate` | Replay speed; `1.0` preserves recorded timing. |
| `runner.ros_domain_id` | Dedicated evaluation domain; the checked-in value is `42`. |
| `runner.continue_on_error` | Continue after failed or invalid executions. |
| `runner.label_buffer_seconds` | Matching tolerance around annotated points/ranges. |
| `runner.semantic_judge_enabled` | Extra model calls for semantic response comparison. |
| `experiments` | Named configurations and detector overrides. |

Each experiment needs a unique `name`. Nested `overrides` merge into the base
configuration; scalar and list values replace the corresponding base values.
An experiment can supply its own `config` path and `trials` count. For example:

```yaml
experiments:
  - name: text-only
    trials: 3
    overrides:
      llm:
        vision_enabled: false
        system_prompt: >-
          Analyze the supplied logs and return the required anomaly decision JSON.
```

Retain the detector's required response format when changing prompts. The
checked-in YAML contains full prompt examples and an optional randomized
experiment. `randomize.samples` expands into separate configurations; a fixed
`seed` reproduces sampled values. Trials repeat each sampled configuration
without resampling. Total executions equal the number of bags multiplied by the
sum of trial counts across all expanded configurations.

## 4. Validate, then run

From `/root/dev_ws`, validate the selected matrix first:

```bash
python3 src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml \
  --dry-run
```

Dry runs read real bag metadata, check configurations, required input topics,
and labeled sidecars, and write reports. They still require ROS Python modules
and MCAP support. They do not start the detector, replay bags, or verify model
availability. Check the report for `dry_run_validated` or `invalid_bag` results.

Remove `--dry-run` to evaluate:

```bash
python3 src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml
```

Use CLI overrides for a particular run:

```bash
python3 src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml \
  --bags src/anomaly_detection/system_evaluation/bags \
  --output-dir /tmp/aad-evaluation \
  --mode unlabeled --trials 3
```

| Option | Meaning |
| --- | --- |
| `--config PATH` | Evaluation YAML. |
| `--bags DIRECTORY` | Override the recording selection. |
| `--output-dir PATH` | Override the report destination. |
| `--provider-config PATH` | Override provider profiles. |
| `--mode {unlabeled,human-labeled}` | Override evaluation mode. |
| `--trials N` | Override trial counts for every experiment. |
| `--dry-run` | Validate and report without replay. |
| `--write-example-config PATH` | Write a starter YAML and exit; refuses to overwrite. |
| `--help` | Show supported options. |

CLI options override YAML values; YAML runner values override the script's
`QUICK_RUN_*` defaults. CLI bag/output paths are relative to the current directory;
provider profile paths are resolved relative to the evaluation YAML, including
when supplied with `--provider-config`. Use absolute paths to avoid ambiguity.
For IDE runs, edit the script's `QUICK_RUN_*` settings and the YAML; their
checked-in absolute paths assume `/root/dev_ws`.

During a real run, the evaluator rejects existing detector endpoints in its ROS
domain, checks and warms up local Ollama models when preflight is enabled,
starts the detector, and replays the bag. Recorded evaluator output topics are
excluded from playback. It then waits for outstanding inference and shuts down
processes before the next execution. Remote model preflight is reported as
skipped. Avoid concurrent evaluations in the same domain.

## 5. Read the results

The console prints the report paths and attempted, completed, and failed counts.
The output directory receives a UTC-timestamped `.md` report and a same-named
`.json` companion; existing reports are not overwritten. Markdown gives a compact
comparison; JSON includes final responses, metrics, errors, warnings, and event
diagnostics. Failed runs, timeouts, or backend errors retain detector and player
logs in a matching `<timestamp>_logs` directory.

Check execution status and errors before comparing scores. A report marked
`completed` means the matrix finished; individual executions may still have
failed. The CLI can return zero even when individual runs fail, so automation
must inspect report results. Configuration/input setup errors exit with code 2.
Ctrl+C normally writes a partial report with `interrupted` status and exits 130.

Interpret the main results as follows:

- Positive model decisions have `anomaly: true`. Immediate safety decisions are
  counted separately because they lack an original model response.
- Labeled results include bag classification and event precision, recall, and
  F1. Matching uses recorded event timing and `label_buffer_seconds`; inspect
  detailed event diagnostics when investigating missed or unmatched decisions.
- Unlabeled agreement measures consistency, not accuracy. Semantic judging allows
  wording changes while checking conflicting anomaly, severity, action, and
  evidence claims.
- Model latency is available only for captured model responses. An unparseable
  decision has no usable boolean anomaly value.

## 6. Troubleshoot a run

| Symptom | Action |
| --- | --- |
| ROS modules unavailable or package/executable missing | Source ROS and the workspace; install MCAP support and build the required packages. |
| Recording directory missing / no bags discovered | Check `runner.bags` and YAML-relative paths; pass a directory rather than an MCAP filename. |
| `invalid_bag` | Read its errors; verify the input topic and adjacent annotations, including `bag.anomalous`. |
| Graph contamination | Stop the other detector/evaluation in the selected domain or choose another unused `runner.ros_domain_id`. |
| Local model preflight failure | Check the Ollama endpoint and exact installed model name. Review `model_preflight_timeout_seconds` for slow loading. |
| Cloud backend/authentication failure | Check provider/model selection, exported credentials, and the profile file. Inspect retained logs. |
| Startup or playback timeout | Inspect detector/player logs; adjust `startup_timeout_seconds` or `playback_timeout_padding_seconds` after finding the cause. |
| Inference still pending at drain timeout | Inspect backend latency and `llm.timeout_seconds`; adjust drain settings if more serial passes are expected. |

The effective inference drain is at least the configured
`inference_drain_timeout_seconds`, and may increase based on model timeout,
`inference_drain_model_passes`, and shutdown margin. Detailed JSON preserves
separate timeout stages; increasing one timeout will not resolve a failure at
another stage.
