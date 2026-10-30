# Usage guide

[README](../README.md) · [Development](development.md) · [Usage](usage.md) · [Evaluation](evaluation.md)

Commands assume `/root/dev_ws` is the workspace root and this repository is at
`/root/dev_ws/src/anomaly_detection`. Build and source the workspace using the
[README quick start](../README.md#quick-start) before running nodes.

## Configure inference

The detector configuration is
[`anomaly_detection/anomaly_detection/config.yaml`](../anomaly_detection/anomaly_detection/config.yaml).
Select the model and endpoint, input/output topics, and a writable artifact
directory. The checked-in configuration uses local Ollama inference.

The Dockerfile downloads a default Ollama model. To use a different model,
download it into the Ollama instance the detector will use, then update the
configuration. For example, replace `your-model:tag` below with the exact model
name you want:

```bash
# Use the same endpoint configured in llm.ollama_host.
OLLAMA_HOST=http://127.0.0.1:21434 ollama pull your-model:tag
OLLAMA_HOST=http://127.0.0.1:21434 ollama list
```

Update these fields in the detector configuration:

```yaml
llm:
  provider: "your-model"
  model: "your-model:tag"
  local: true
  ollama_host: "http://127.0.0.1:21434"
```

The `model` value must exactly match the downloaded model name and tag. Run the
commands where the configured Ollama endpoint is reachable; adjust the host and
port for your deployment. The checked-in preferred port is `21434`. The detector
can also discover another reachable local endpoint containing the configured
model. When sending camera frames, select a model that supports image input.

Rebuild, source the workspace, and restart the detector after changing its
configuration, following the [development workflow](development.md#development-workflow). For offline experiments,
also update any model overrides in
[`system_evaluation/offline_evaluation.yaml`](../system_evaluation/offline_evaluation.yaml).
Cloud inference configuration is described below.

## Start the detector

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export AAD_CONFIG_PATH="$PWD/src/anomaly_detection/anomaly_detection/anomaly_detection/config.yaml"
ros2 run anomaly_detection anomaly_detection_node
```

`AAD_CONFIG_PATH` explicitly selects a configuration. Without it, the node loads
`config.yaml` beside its Python module.

## Send a test message and inspect output

In another terminal, source the same ROS and workspace setup files, then watch
all decisions:

```bash
ros2 topic echo /aad/decisions
```

In a third sourced terminal, publish a warning:

```bash
ros2 topic pub --once /ai_anomaly_logging anomaly_msg/msg/AnomalyMsg \
  "{node_name: 'readme_test', importance: 1, type: 0, msg: 'Localization updates are delayed; investigate sensor health.'}"
```

With the default configuration, the warning becomes eligible for model processing.
A decision appears after scheduling and inference complete. An alert is published
to `/aad/alerts` if the decision identifies an anomaly. Confirm message receipt,
model/backend errors, and artifact creation in the detector logs.

For continuous synthetic input, use `ros2 run tester lidar_test_node` or
`ros2 run tester camera_anomaly_node`.

## How detection works

1. Subscribe to `raw_input_topic` for custom `AnomalyMsg` messages.
2. Apply duplicate suppression and per-publisher severity rate limits, then cache
   accepted telemetry and eligible images.
3. Schedule model processing from periodic checks, severity-qualified telemetry,
   or messages on `trigger_input_topic`. The default minimum telemetry trigger
   importance is `warning`; INFO telemetry can still provide context.
4. Send cached context to a cloud model through LiteLLM or a local model through
   the Ollama Python client. Image selection and freshness are configurable.
5. Standardize the response into a `Decision`, publish the decision, and publish
   an alert when appropriate. Save a JSON artifact for the model invocation.

High-severity telemetry and recognized high-severity trigger messages can publish
an immediate `stop_cart` alert before model inference. A cooldown limits repeated
stop alerts. Treat these immediate decisions separately when reading evaluation
reports.

Bag recording is a separate operation; running the detector alone does not record
the entire ROS graph. For example, from a sourced terminal:

```bash
ros2 bag record /ai_anomaly_logging
```

## ROS interfaces

Topic names below are the checked-in defaults; input, trigger, and alert topics
can be changed in the detector configuration.

| Topic | ROS type | Purpose |
| --- | --- | --- |
| `/ai_anomaly_logging` | `anomaly_msg/msg/AnomalyMsg` | Telemetry and camera input |
| `/trigger_messages` | `std_msgs/msg/String` | Explicit model trigger and context |
| `/aad/decisions` | `std_msgs/msg/String` | Published decisions, including normal results |
| `/aad/alerts` | `std_msgs/msg/String` | Anomaly alerts and immediate stop alerts |

[`AnomalyMsg.msg`](../anomaly_msg/msg/AnomalyMsg.msg) defines the input contract:

- `header` and `node_name` identify the observation and source.
- `importance`: `INFO=0`, `WARNING=1`, or `ERROR=2`.
- `type`: `TEXT=0`, `IMAGE=1`, or `DATA=2`.
- `msg` carries text; `image` carries a `sensor_msgs/Image` for image messages.
- `data_type` and `data` describe and carry binary data for DATA messages.

The model is asked to return a JSON decision such as:

```json
{"anomaly": true, "severity": "medium", "action": "alert_admin", "summary": "Localization updates are delayed."}
```

ROS output messages contain plain text, rather than that JSON object. For example:

```text
# /aad/decisions
anomaly=True severity=medium action=alert_admin summary=Localization updates are delayed.

# /aad/alerts
[AAD ALERT] severity=medium action=alert_admin summary=Localization updates are delayed.
```

## Cloud inference

Select a provider profile and model in the detector configuration or experiment
overrides:

```yaml
llm:
  provider: anthropic
  model: claude-sonnet-4-5-20250929
  local: false
```

[`system_evaluation/llm_providers.yaml`](../system_evaluation/llm_providers.yaml)
contains provider profiles for OpenAI and Anthropic. Profiles reference credential
environment variables:

```yaml
providers:
  anthropic:
    litellm_provider: anthropic
    api_key_env: ANTHROPIC_API_KEY
```

Export the referenced key in the terminal launching the detector or evaluator:

```bash
export ANTHROPIC_API_KEY='...'
```

Offline evaluation selects profiles through `runner.provider_config` or
`--provider-config PATH`. Direct node runs can use:

```bash
export AAD_LLM_PROVIDERS_PATH=/root/dev_ws/src/anomaly_detection/system_evaluation/llm_providers.yaml
```

Alternatively, set `llm.provider_config`; relative paths resolve beside the
detector configuration. A cloud API key is unnecessary for local Ollama inference.
The client also loads an optional `.env` beside `llm_client.py`.

## Artifact Generation

Each time the LLM is invoked, a JSON artifact is written to `config.yaml:api_artifact_output_dir` capturing a snapshot of the system state.

Each artifact includes:
- **Artifact ID** — unique traceable identifier
- **Cached Data** — the log messages collected before the LLM call
- **API Response** — the structured decision returned by the LLM (anomaly flag, severity, action, summary)

## Troubleshooting

- **Package or executable not found:** rebuild from `/root/dev_ws`, source
  `install/setup.bash` in the launching terminal, and restart the node.
- **Changes do not appear:** check `AAD_CONFIG_PATH`, repeat the build/source
  workflow, and restart affected processes. Do not edit generated installed files.
- **No telemetry received:** check `ros2 topic list`, the configured input topic,
  publisher message type, ROS domain, and container networking.
- **No model decision:** inspect detector logs for backend errors and check the
  configured minimum importance, trigger timing, and message rate limits.
- **Ollama model unavailable:** use `ollama list` against the configured endpoint
  as shown above, and confirm the exact model name and tag.
- **Cloud authentication fails:** check the selected provider profile and export
  its referenced key in the terminal launching the node or evaluator.
