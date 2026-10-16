# Offline Evaluation Report

This report pulls together the current offline replay setup from [run_offline_evaluation.py](scripts/run_offline_evaluation.py) and [offline_evaluation.yaml](offline_evaluation.yaml). The point here is not the code path itself, but the shape of the evaluation: which knobs exist, how they change the replay, and what the current experiment is actually testing.

## Configuration Surface

The runner is built around a layered configuration model. Command-line arguments win first, then the YAML runner block, then the quick-run defaults near the top of the Python file. Each experiment inherits one shared detector configuration and can override individual settings directly in the evaluation YAML.

### Runner settings

These settings control the replay session itself:

- `mode`: `unlabeled` or `human-labeled`
- `bags`: one bag directory or a folder of bags
- `output_directory`: where the markdown report is written
- `trials`: how many repeats to run for each bag and experiment
- `playback_rate`: replay speed, with `1.0` preserving recorded timing
- `continue_on_error`: whether the matrix keeps going after a failure
- `ros_domain_id`: the dedicated ROS domain shared only by the evaluator, its detector, and its bag player
- `startup_timeout_seconds`: how long the detector gets to come up
- `startup_grace_seconds`: a short grace period after startup
- `playback_timeout_padding_seconds`: extra time added on top of the recorded duration
- `post_playback_grace_seconds`: quiet time after playback ends
- `inference_drain_timeout_seconds`: minimum time to wait for pending model activity
- `inference_drain_model_passes`: serial model passes allowed when calculating the effective drain limit
- `shutdown_timeout_seconds`: how long to wait for the detector process to stop cleanly
- `graph_clean_timeout_seconds`: how long stale ROS discovery entries may take to disappear before the run is rejected as contaminated
- `model_preflight_enabled`: verify and warm a local Ollama model before replay
- `model_preflight_timeout_seconds`: maximum time for the preflight warm-up request
- `label_buffer_seconds`: allowed timing margin around sidecar point/range labels
- `semantic_judge_enabled`: whether final comparisons may make additional model calls to judge response summaries
- `decision_topic`: detector decision topic, defaulting to `/aad/decisions`
- `alert_topic`: alert topic, defaulting to `/aad/alerts`
- `llm_called_topic`: LLM-call topic, defaulting to `/aad/llm_called`
- `formatted_topic`: recorded formatted-message topic excluded from playback, defaulting to `/aad/formatted_messages`
- `detector_command`: the command used to launch the anomaly detector node
- `dry_run`: validate the setup without replaying the bag

The quick-run block in the Python file adds only launch conveniences:

- `QUICK_RUN_BAG_LOCATION`
- `QUICK_RUN_CONFIG`
- `QUICK_RUN_MODE`
- `QUICK_RUN_OUTPUT_DIRECTORY`

Those values are meant to make editor-based runs easier, not to replace the YAML.

### Experiment settings

The experiment block is where the detector-side comparison is defined. The current experiments inherit the production detector settings from `base_config` and override their system prompts directly in the evaluation YAML.

The available experiment controls are:

- `base_config`: an optional shared detector config inherited by experiments without their own `config`
- `experiments[].name`: the label used in the report and comparison tables
- `experiments[].trials`: optional per-experiment repeat count
- `experiments[].config`: optional alternate base config for a single experiment
- `experiments[].overrides`: nested detector settings applied on top of the base config

The override mechanism is flexible enough to handle either nested YAML or dotted keys. That means a configuration can target individual fields without rewriting the whole config file. In practice, this is what makes small comparison studies manageable.

## How The Data Moves

The replay flow is fairly direct once the settings are resolved.

1. The YAML file is loaded and merged with any command-line overrides.
2. The shared detector config is read, then each experiment's system prompt is layered on top.
3. The bag path is resolved and discovered.
4. The evaluator enters its dedicated ROS domain and rejects unexpected detector publishers or subscribers.
5. A local Ollama endpoint is checked for the configured model and warmed with a one-token request.
6. A fresh runtime config is written to a temporary location for each run.
7. The detector is started with that isolated config.
8. The collector opens a run-specific accounting window and the bag is replayed at the requested rate.
9. After playback, the runner uses the larger of the configured drain minimum or `(model timeout × model passes) + shutdown margin`.
10. Timeout stages are recorded separately. Failed, timed-out, and backend-error runs retain detector and player logs beside the report.

That gives the evaluation a clean data path: bag in, resolved config in, detector output out, report at the end.

## What The Current Experiment Is Testing

The current YAML selects one bag folder and compares two experiment variants declared in that same file.

The active experiments are:

- `local-baseline`
- `local-golf-cart`

Both inherit the same detector configuration and override the system prompt. They use these fixed parameter combinations:

| Experiment | `num_ctx` | `num_predict` | `image_jpeg_quality` |
|---|---:|---:|---:|
| `local-baseline` | 2048 | 192 | 65 |
| `local-golf-cart` | 3072 | 256 | 75 |

The YAML also includes a commented example for reproducibly sampling randomized LLM parameters.

## What The Report Is Supposed To Answer

The current run is human-labeled. It reads the adjacent
`<bag-stem>.annotations.json` sidecar and reports event precision, recall, F1,
bag accuracy, behavior, latency, and agreement.

That makes the output useful for questions like:

- Did the two configurations produce similar decision patterns?
- How much latency did the detector show during replay?
- How many alerts and positive decisions came out of each run?
- Did any run leave LLM calls pending when the bag stopped?
- Which detections matched the labeled anomaly ranges and message points?

The report is also set up to preserve run-by-run detail, so the final markdown is not just an aggregate summary. It keeps the configuration details, the per-run metrics, and the pairwise comparison between experiments.

## Plain Summary

The short version is that this evaluation replays one annotated driving session
through two configurations and compares their accuracy, behavior, and latency.
They use different prompts and context, output, and image-quality settings.
