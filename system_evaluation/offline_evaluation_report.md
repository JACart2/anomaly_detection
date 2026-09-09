# Offline Evaluation Report

This report pulls together the current offline replay setup from [run_offline_evaluation.py](../scripts/run_offline_evaluation.py) and [offline_evaluation.yaml](offline_evaluation.yaml). The point here is not the code path itself, but the shape of the evaluation: which knobs exist, how they change the replay, and what the current experiment is actually testing.

## Configuration Surface

The runner is built around a layered configuration model. Command-line arguments win first, then the YAML runner block, then the quick-run defaults near the top of the Python file. The experiment-specific detector settings come from YAML on top of the base AAD config, so the run is assembled in pieces instead of being hard-coded in one place.

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
- `context_lookback_seconds`: how far back the collector can look for related formatted messages
- `comparison_window_seconds`: the matching window used when comparing decisions and replayed context
- `decision_topic`: detector decision topic, defaulting to `/aad/decisions`
- `alert_topic`: alert topic, defaulting to `/aad/alerts`
- `llm_called_topic`: LLM-call topic, defaulting to `/aad/llm_called`
- `formatted_topic`: formatted-message topic, defaulting to `/aad/formatted_messages`
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

The experiment block is where the detector-side comparison is defined. The runner starts from `base_config`, then applies each experiment’s overrides and resolves them into a full runtime config.

The current file uses these experiment controls:

- `base_config`: the shared detector config inherited by every experiment
- `experiments[].name`: the label used in the report and comparison tables
- `experiments[].trials`: optional per-experiment repeat count
- `experiments[].config`: optional alternate base config for a single experiment
- `experiments[].overrides`: nested detector settings applied on top of the base config

The override mechanism is flexible enough to handle either nested YAML or dotted keys. That means a configuration can target individual fields without rewriting the whole config file. In practice, this is what makes small comparison studies manageable.

## How The Data Moves

The replay flow is fairly direct once the settings are resolved.

1. The YAML file is loaded and merged with any command-line overrides.
2. The base AAD configuration is read, then the experiment overrides are layered on top.
3. The bag path is resolved and discovered.
4. A fresh runtime config is written to a temporary location for each run.
5. The detector is started with that isolated config.
6. The bag is replayed at the requested playback rate.
7. The collector watches the detector decision, alert, LLM-called, and formatted-message topics while the replay runs.
8. After the bag finishes, the runner waits for the detector to go quiet, then captures the final state and writes the report.

That gives the evaluation a clean data path: bag in, resolved config in, detector output out, report at the end.

## What The Current Experiment Is Testing

The current YAML is intentionally narrow. It selects one bag folder, runs three trials, and compares two local detector setups that differ only in retained text context size.

The active experiments are:

- `local_text_cache_10`
- `local_text_cache_20`

Both use local inference and both disable vision and image context. The only explicit difference is `cache_max_items`, which is set to `10` in one case and `20` in the other. That makes the comparison about context depth, not about changing the model, the prompt, or the image pipeline.

The base config still carries the full production detector profile, including the model provider, model name, local execution flag, token window, image-related defaults, prompt, trigger settings, and replay timing values inherited from the main anomaly-detection configuration. The evaluation file only changes the pieces needed for the comparison.

## What The Report Is Supposed To Answer

Because the current run is unlabeled, the report stays in behavior-and-agreement territory instead of pretending to measure ground-truth accuracy.

That makes the output useful for questions like:

- Did the two configurations produce similar decision patterns?
- How much latency did the detector show during replay?
- How many alerts and positive decisions came out of each run?
- Did any run leave LLM calls pending when the bag stopped?

The report is also set up to preserve run-by-run detail, so the final markdown is not just an aggregate summary. It keeps the configuration details, the per-run metrics, and the pairwise comparison between experiments.

## Plain Summary

The short version is that this evaluation replays one recorded driving session through two local detector configurations, varies the amount of text context retained by the LLM, and compares the runs on output behavior and latency. The interesting part is not the bag itself; it is the configuration spread around the bag and how the detector changes when that spread changes.
