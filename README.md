# AI Anomaly Detection
Last updated: 2/25/2026 (Sprint 1)

Anomaly Detection ROS2 system. Set to integrate with GPT-5-mini through OpenAI dev creds. Currently hard-coded data sources, API integration, and API trigger based on set time interval from config.yaml. No meaningful data processing takes place. .bag file creation of runtime conditions are not yet automated.

Future development should provide alternatives for API integration, API buffer models, data source topics, meaningful data processing, and automated system deployment and artifact creation. 

## Prerequisites
_It is recommended that this system exists in a Docker container that shares a network with the source of ROS2 topics publishing system data. See (https://github.com/JACart2/docker_files)_

* ROS2 installed
* Requirements.txt (see https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/requirements.txt)
* OA_SECRET.txt for local testing, or OA_SECRET set as environment variable (contains secret key for OpenAI dev account for API call)

## Project Structure

### ./anomaly_msg
The declaration of the custom message type that the anomaly detection system expects. Any data source that provides context to the anomaly detection service should send information using this message type to the topic specified in the config.yaml `raw_input_topic`.

### ./tester
This provides two utility nodes: `fake_camera_data` and `lidar_test_node`. Both of these will publish fake data and may be used to test dataflow in the system without external node access. This can be ran with `ros2 run tester <NODE>`.

### ./anomaly_detection
This is the ROS2 node declaration for the anomaly detection system. The logic is located under `./anomaly_detection/anomaly_detection`. `anomaly_detection_node.py` contains the manager that will handle API integration, data subscription, data processing, and subsequent action for the system to take if anomalies are detected. `config.yaml` contains system configuration for the system. `openai_call.py` is the current API integration, but this will change after this sprint.


## Usage

Running this command: 
`ros2 run anomaly_detection anomaly_detection_node`
will start the anomaly detection node.

However, note the dependencies in prerequisites, and the specifications of `./anomaly_detection/anomaly_detection/config.yaml` for successful deloyment.
