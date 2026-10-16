# AI Anomaly Detection
Last updated: 4/28/2026 

Anomaly Detection for a ROS2 system. Integrates with LiteLLM (https://docs.litellm.ai/docs/) or local Ollama model for determinations. 

1. Subscribes to `config.yaml:raw_input_topic` containing AnomalyMsg types.
2. Blocks duplicate messages with `config.yaml:duplicate_message_min_period_sec`, and converts AnomalyMsgs into compact string format for LLM.
3. Periodic/on-demand LLM trigger (`config.yaml:trigger_scripts` via ROS node publishing to `config.yaml:trigger_input_topic`).
4. LLM call using `llm_client.py`; a JSON artifact capturing the cached input data and model response is saved to `config.yaml:api_artifact_output_dir` on each invocation.
5. LLM response standardization into `Decision` type using `response_handler.py`.
6. Decision evaluation and alert publishing to `config.yaml:alert_topic`.
7. `.bag` file of entire run context monitoring `config.yaml:raw_input_topic` if enabled in separate terminal.

## Prerequisites

_It is recommended that this system exists in a Docker container that shares a network with (or contains) the source of ROS2 topics publishing system data. See (https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/Dockerfile)_

* ROS2 installed
* Requirements.txt. See (https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/requirements.txt)
* `.env` for local testing/prod run. Contains the API key associated with the model specified in `config.yaml`
* Python (>= 3.12 recommended, older versions may cause bugs)

Follow these steps below for Ollama config on host machine for supporting local model integration (assuming Linux OS)

1. curl -fsSL https://ollama.com/install.sh | sh
2. ollama pull `my_model`

_Assert that the downloaded model name matches with the names referenced in the AAD node's config.yaml._


## Project Structure

### ./anomaly_msg
The declaration of the custom message type that the anomaly detection system expects. Any data source that provides context to the anomaly detection service should send information using this message type to the topic specified in `config.yaml:raw_input_topic`.

### ./tester
Provides utility nodes such as: `fake_camera_data` and `lidar_test_node`. Both publish fake data and may be used to test dataflow in the system without external node access. Run with `ros2 run tester <NODE>`.

### ./system_evaluation/scripts
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
[`system_evaluation/schemas/anomaly_annotations.schema.json`](system_evaluation/schemas/anomaly_annotations.schema.json).

### ./anomaly_detection
The ROS2 node declaration for the anomaly detection system. 

`anomaly_detection_node.py` contains the manager that drives the system.

`config.yaml` contains system configuration such as API call frequency, cache size, LLM specs (prompt, local enabling, system_prompt), trigger script specification & trigger topic, input data topic for the AAD system, output alert topic for the AAD system, duplicate message blocking, logging frequency, and artifact output path & size.

`llm_client.py` integrates with the LLM either through an API call in LiteLLM, or through a locally hosted model through the Ollama CLI.

`response_handler.py` standardizes API responses into a `Decision` type.

### ./anomaly_detection/anomaly_detection/triggers
Each trigger script runs as its own ROS node and publishes to the `config.yaml:trigger_input_topic` topic. Adding a new trigger script requires:

**Required:**
- `trigger_scripts/<SCRIPT_NAME>/<SCRIPT_NAME>.py` — the ROS node
- `trigger_scripts/<SCRIPT_NAME>/install.sh`
- `config.yaml['trigger_scripts']` entry: `['<SCRIPT1_NAME>', '<SCRIPT2_NAME>']`

**Optional:**
- `trigger_scripts/<SCRIPT_NAME>/requirements.txt`

## Offline Testing

A script is provided for replaying recorded bags against one or more experiments. The shared detector configuration and experiment-specific system prompts are declared in `system_evaluation/offline_evaluation.yaml`:

```
./src/anomaly_detection/system_evaluation/scripts/run_offline_evaluation.py \
  --config src/anomaly_detection/system_evaluation/offline_evaluation.yaml
```

**Notes:**
- The executable automatically sources the detected ROS installation and nearest workspace `install/setup.bash`
- Evaluation bags live under `src/anomaly_detection/system_evaluation/bags/`
- Add or change an experiment prompt through `experiments[].overrides.llm.system_prompt`
- Run the script from the `dev_ws/` directory; otherwise it will be unable to start `anomaly_detection_node`

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

## Artifact Generation

Each time the LLM is invoked, a JSON artifact is written to `config.yaml:api_artifact_output_dir` capturing a snapshot of the system state. 

Each artifact includes:
- **Artifact ID** — unique traceable identifier
- **Cached Data** — the log messages collected before the LLM call
- **API Response** — the structured decision returned by the LLM (anomaly flag, severity, action, summary)


## System Diagrams

### Architecture Diagram
![Architecture Diagram](docs/architecture_diagram.jpeg)

### Dataflow Diagram
![Dataflow Diagram](docs/dataflow_diagram.jpeg)

## Usage

Start the anomaly detection node:

```
ros2 run anomaly_detection anomaly_detection_node
```

For local inference, install Ollama and the configured model. For cloud
inference, select a provider profile and model in the AAD/evaluation YAML:

```yaml
llm:
  provider: anthropic
  model: claude-sonnet-4-5-20250929
  local: false
```

Offline evaluation reads endpoint and authentication setup from the separate
`runner.provider_config` file. `system_evaluation/llm_providers.yaml` contains
ready-to-use OpenAI and Anthropic profiles. Profiles reference environment
variable names rather than containing secrets:

```yaml
providers:
  anthropic:
    litellm_provider: anthropic
    api_key_env: ANTHROPIC_API_KEY
```

Export the referenced key before launching the evaluator:

```bash
export ANTHROPIC_API_KEY='...'
python3 system_evaluation/scripts/run_offline_evaluation.py \
  --config system_evaluation/offline_evaluation.yaml
```

Use `--provider-config PATH` to select a different profile file. Direct node
runs may instead set `AAD_LLM_PROVIDERS_PATH` or `llm.provider_config`.

See `config.yaml` for full deployment configuration options, including model selection, topic names, and trigger script registration.

## Attributions
Created by John Rosario Cruz (gr8jj28@gmail.com), Allie O'Keeffe, & Christopher Michael Hubbard.
