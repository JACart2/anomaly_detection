# AI Anomaly Detection
Last updated: 4/19/2026 (Sprint 4)

Anomaly Detection for a ROS2 system. Integrates with LiteLLM (https://docs.litellm.ai/docs/) or local Ollama model for determinations. 

1. Subscribes to `config.yaml:raw_input_topic` containing AnomalyMsg types.
2. Throttles certain message types with `config.yaml:throttle_info`, and converts AnomalyMsgs into compact string format for LLM.
3. Periodic/on-demand LLM trigger (`config.yaml:trigger_scripts` via ROS node publishing to `config.yaml:trigger_input_topic`).
4. LLM call using `llm_client.py`; a JSON artifact capturing the cached input data and model response is saved to `config.yaml:api_artifact_output_dir` on each invocation.
5. LLM response standardization into `Decision` type using `response_handler.py`.
6. Decision evaluation and alert publishing to `config.yaml:alert_topic`.
7. `.bag` file of entire run context monitoring `config.yaml:raw_input_topic` if enabled in separate terminal.

Sprint 5 will focus on finalized system documentation, scenario recordings/dataset creation, (potentially) security vulnerability addressing, and planning system evaluation methods.

## Prerequisites

_It is recommended that this system exists in a Docker container that shares a network with (or contains) the source of ROS2 topics publishing system data. See (https://github.com/JACart2/docker_files)_

* ROS2 installed
* Requirements.txt. See (https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/requirements.txt)
* `.env` for local testing/prod run. Contains the API key associated with the model specified in `config.yaml`
* Python (>= 3.12 recommended, older versions may cause bugs)

## Project Structure

### ./anomaly_msg
The declaration of the custom message type that the anomaly detection system expects. Any data source that provides context to the anomaly detection service should send information using this message type to the topic specified in `config.yaml:raw_input_topic`.

### ./tester
Provides utility nodes such as: `fake_camera_data` and `lidar_test_node`. Both publish fake data and may be used to test dataflow in the system without external node access. Run with `ros2 run tester <NODE>`.

### ./anomaly_detection
The ROS2 node declaration for the anomaly detection system. 

`anomaly_detection_node.py` contains the manager that drives the system.

`config.yaml` contains system configuration such as API call frequency, cache size, LLM specs (prompt, local enabling, system_prompt), trigger script specification & trigger topic, input data topic for the AAD system, output alert topic for the AAD system, message throttling, logging frequency, and artifact output path & size.

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

A script is provided for running multiple configs against the dataset in batch:

```
python3 path/to/run_aad_config_tests.py --csv dataset.csv --configs openai_config.yaml ollama_config.yaml other_configs.yaml
```

**Notes:**
- Create `bags/` and `configs/` folders inside `dev_ws/` before running
- Run the script from the `dev_ws/` directory; otherwise it will be unable to start `anomaly_detection_node`

### Dataset Outline

The dataset is a `.csv` file (`dataset_outline.csv`) with one row per scenario. Fields:

| Field | Values |
|---|---|
| Anomaly Category | Normal, Dynamic obstacle, Static obstacle, Mechanical issue, Sensor issue, Unauthorized access, Route issue |
| Description | Additional detail about the anomaly |
| Anomalous | Yes / No |
| Bag file | Name of the associated `.mcap` bag file |

Store all bag files in the `bags/` folder inside `dev_ws/`.

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

This requires a `.env` file in the same folder as the AAD node, with API keys in the format for LiteLLM, or Ollama CLI installed with appropriate models on the host machine:

```
<Provider-Name>_API_KEY=<Your-Key>
```

See `config.yaml` for full deployment configuration options, including model selection, topic names, and trigger script registration.
