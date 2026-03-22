# AI Anomaly Detection
Last updated: 3/22/2026 (Sprint 2)

Anomaly Detection ROS2 system. Set to integrate with LiteLLM (https://docs.litellm.ai/docs/) for API integration. Currently integrates with trigger scripts through trigger_script folder directly (will be rewritten to use new topic in Sprint 3). 

1. Subscribes to `raw_input_topic` containing AnomalyMsg types 
2. Processes them into LLM-friendly format & stores in cache 
3. Periodic/on-demand (`trigger_script.py`) cache dump and LLM call using `llm_client.py` 
4. LLM response standardization into `Decision` type using `response_handler.py` 
5. Decision evaluation and alert publishing
6. .bag file of entire run context monitoring `raw_input_topic`

Future development will provide dynamic trigger script integration logic, additional artifact creation for cache/API responses, and dynamic config file selection for system evaluation.

## Prerequisites
_It is recommended that this system exists in a Docker container that shares a network with (or contains) the source of ROS2 topics publishing system data. See (https://github.com/JACart2/docker_files)_

* ROS2 installed
* Requirements.txt. See (https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/requirements.txt)
* .env for local testing/prod run. Contains the API key associated with the model specified in config.yaml

## Project Structure

### ./anomaly_msg
The declaration of the custom message type that the anomaly detection system expects. Any data source that provides context to the anomaly detection service should send information using this message type to the topic specified in the config.yaml `raw_input_topic`.

### ./tester
This provides two utility nodes: `fake_camera_data` and `lidar_test_node`. Both of these will publish fake data and may be used to test dataflow in the system without external node access. This can be ran with `ros2 run tester <NODE>`.

### ./anomaly_detection
This is the ROS2 node declaration for the anomaly detection system. `anomaly_detection_node.py` contains the manager that will handle API integration, data collection/processing, trigger integration, and alert publishing. 

`config.yaml` contains system configuration.
`llm_client.py` is the API integration.
`response_handler.py` standardizes API responses into a `Decision` type.

## System Diagrams

### Architecture Diagram
![Architecture Diagram](docs/cs480_sprint2_arch_diagram.jpeg)

### Dataflow Digram
![Dataflow Diagram](docs/cs480_sprint2_dataflow_diagram.jpeg)


## Usage

Running this command: 
`ros2 run anomaly_detection anomaly_detection_node`
will start the anomaly detection node.

However, note the dependencies in prerequisites, and the specifications of `./anomaly_detection/anomaly_detection/config.yaml` for successful deloyment.
