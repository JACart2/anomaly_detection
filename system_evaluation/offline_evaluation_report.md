# Offline Evaluation Report

This report pulls together the current offline replay setup from [run_offline_evaluation.py](../scripts/run_offline_evaluation.py) and [offline_evaluation.yaml](offline_evaluation.yaml). The point here is not the code path itself, but the shape of the evaluation: which knobs exist, how they change the replay, and what the current experiment is actually testing.

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
- `startup_timeout_seconds`: how long the detector gets to come up
- `startup_grace_seconds`: a short grace period after startup
- `playback_timeout_padding_seconds`: extra time added on top of the recorded duration
- `post_playback_grace_seconds`: quiet time after playback ends
- `inference_drain_timeout_seconds`: how long to wait for pending model activity to finish
- `shutdown_timeout_seconds`: how long to wait for the detector process to stop cleanly
- `semantic_judge_enabled`: whether final comparisons may make additional model calls to judge response summaries
- `decision_topic`: detector decision topic, defaulting to `/aad/decisions`
- `alert_topic`: alert topic, defaulting to `/aad/alerts`
- `llm_called_topic`: LLM-call topic, defaulting to `/aad/llm_called`
- `formatted_topic`: recorded formatted-message topic excluded from playback, defaulting to `/aad/formatted_messages`
- `detector_command`: the command used to launch the anomaly detector node
- `human_label_topic`: required when running in human-labeled mode
- `human_label_positive_regex`: optional filter for label messages in human-labeled mode
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
4. A fresh runtime config is written to a temporary location for each run.
5. The detector is started with that isolated config.
6. The bag is replayed at the requested playback rate.
7. The collector watches detector decisions, alerts, and LLM-call notifications while the replay runs.
8. After the bag finishes, the runner waits for the detector to go quiet, then captures the final state and writes the report.

That gives the evaluation a clean data path: bag in, resolved config in, detector output out, report at the end.

## What The Current Experiment Is Testing

The current YAML selects one bag folder and compares four experiment variants declared in that same file.

The active experiments are:

- `local-baseline`
- `local-golf-cart`
- `local-production-prompt-long-context`
- `local-evidence-first-randomized` (reported as `local-evidence-first-randomized__sample_1`)

All four inherit the same detector configuration. The first two override the system prompt, while the third deliberately retains the production prompt. The first three use these fixed parameter combinations:

| Experiment | `num_ctx` | `num_predict` | `image_jpeg_quality` |
|---|---:|---:|---:|
| `local-baseline` | 2048 | 192 | 65 |
| `local-golf-cart` | 3072 | 256 | 75 |
| `local-production-prompt-long-context` | 4096 | 320 | 85 |

The fourth profile uses an evidence-first safety prompt and reproducibly samples one combination of the same three parameters using seed `20260923`.

## What The Report Is Supposed To Answer

Because the current run is unlabeled, the report stays in behavior-and-agreement territory instead of pretending to measure ground-truth accuracy.

That makes the output useful for questions like:

- Did the four configurations produce similar decision patterns?
- How much latency did the detector show during replay?
- How many alerts and positive decisions came out of each run?
- Did any run leave LLM calls pending when the bag stopped?

The report is also set up to preserve run-by-run detail, so the final markdown is not just an aggregate summary. It keeps the configuration details, the per-run metrics, and the pairwise comparison between experiments.

## Plain Summary

The short version is that this evaluation replays one recorded driving session through four configurations and compares their output behavior and latency. The first three test progressively larger context, output, and image-quality settings; the third keeps the production prompt, while the fourth uses a custom prompt and one reproducibly randomized parameter set.
