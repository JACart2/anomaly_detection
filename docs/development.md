# Development guide

[README](../README.md) · [Development](development.md) · [Usage](usage.md) · [Evaluation](evaluation.md)

Use this guide when changing source code, message definitions, or packaged configuration.

## Development workflow

**After every code change, run `colcon build`, then `source install/setup.bash`,
and restart the affected nodes.** Follow this workflow even when using
`--symlink-install`; changes to message definitions, package metadata, entry
points, and installed resources still require a build. Running nodes must be
restarted to load your changes.

1. Stop the detector and any test nodes you are changing with `Ctrl+C`.
2. Edit the source files under `/root/dev_ws/src/anomaly_detection`.
3. Build from the workspace root and source the resulting environment:

   ```bash
   cd /root/dev_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-select anomaly_msg anomaly_detection tester
   # Continue only after the build succeeds.
   source install/setup.bash
   ```

4. In every other terminal used to launch nodes or run ROS commands, source
   `/opt/ros/jazzy/setup.bash` and `/root/dev_ws/install/setup.bash` again.
5. Restart the detector and affected nodes. Repeat the test-message example
   in the [usage guide](usage.md#send-a-test-message-and-inspect-output), inspect `/aad/decisions` and `/aad/alerts`, and check the detector logs.

Use the same rebuild/source/restart sequence after changes to the detector YAML
or packaged resources. Keep custom configurations outside generated `build/` and
`install/` directories, and select them with `AAD_CONFIG_PATH`.

### Where to make changes

| Change | Source location |
| --- | --- |
| Input message fields or constants | `anomaly_msg/msg/AnomalyMsg.msg` |
| Caching, scheduling, ROS subscriptions, and alert publishing | `anomaly_detection/anomaly_detection/anomaly_detection_node.py` |
| Model requests, image payloads, and provider configuration | `anomaly_detection/anomaly_detection/llm_client.py` |
| Response parsing and decision normalization | `anomaly_detection/anomaly_detection/response_handler.py` |
| Topics, prompts, thresholds, and model selection | `anomaly_detection/anomaly_detection/config.yaml` |
| Synthetic telemetry publishers | `tester/tester/` |
| Replay experiments and model comparisons | `system_evaluation/offline_evaluation.yaml` |

When changing `AnomalyMsg.msg`, rebuild the message package and both consumers
with the command above, then restart all publishers and subscribers using that
message. Register new executable nodes in the relevant package's `setup.py`.
Trigger nodes use the separate registration process described under Project
Structure.

### Run checks

After building and sourcing the workspace, run the ROS package checks:

```bash
cd /root/dev_ws
colcon test --packages-select anomaly_detection tester
colcon test-result --verbose
```

The standalone evaluation tools also have tests. Run them from the repository
root in the configured Python/ROS environment:

```bash
cd /root/dev_ws/src/anomaly_detection
python3 -m pytest system_evaluation/tests
```

Use the [offline replay workflow](evaluation.md#offline-testing) when a change affects model prompts,
detection behavior, or evaluation metrics. Inspect both the Markdown comparison
and its detailed JSON companion; agreement in unlabeled mode is not a measure
of ground-truth accuracy.

## Project Structure

### ./anomaly_msg
The declaration of the custom message type that the anomaly detection system expects. Any data source that provides context to the anomaly detection service should send information using this message type to the topic specified in `config.yaml:raw_input_topic`.

### ./tester
Provides utility nodes such as: `camera_anomaly_node` and `lidar_test_node`. Both publish fake data and may be used to test dataflow in the system without external node access. Run with `ros2 run tester <NODE>`.

### ./anomaly_detection
The ROS2 node declaration for the anomaly detection system.

`anomaly_detection_node.py` contains the manager that drives the system.

`config.yaml` contains system configuration such as API call frequency, cache size, LLM specs (prompt, local enabling, system_prompt), trigger script specification & trigger topic, input data topic for the AAD system, output alert topic for the AAD system, duplicate message blocking, logging frequency, and artifact output path & size.

`llm_client.py` integrates with the LLM either through an API call in LiteLLM, or through a locally hosted model through the Ollama Python client.

`response_handler.py` standardizes API responses into a `Decision` type.

### ./anomaly_detection/anomaly_detection/triggers
Each trigger script runs as its own ROS node and publishes to the `config.yaml:trigger_input_topic` topic. Adding a new trigger script requires:

**Required:**

- `anomaly_detection/anomaly_detection/triggers/<SCRIPT_NAME>/<SCRIPT_NAME>.py` — the ROS node
- `anomaly_detection/anomaly_detection/triggers/<SCRIPT_NAME>/install.sh`
- `trigger_scripts` list in the detector configuration containing the trigger directory names

**Optional:**

- `anomaly_detection/anomaly_detection/triggers/<SCRIPT_NAME>/requirements.txt`
