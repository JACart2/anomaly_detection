# AI Anomaly Detection

A ROS 2 service that collects telemetry and camera frames, uses local or cloud
language models to identify anomalies, and publishes alerts and decisions.
Offline replay tools compare model configurations against recorded bags and
human annotations.

## Documentation

Detailed instructions live in the [`docs/`](docs/) folder:

- [Development guide](docs/development.md): code layout, rebuild/source workflow,
  adding nodes and triggers, and tests.
- [Usage guide](docs/usage.md): selecting and downloading models, cloud providers,
  configuration, ROS interfaces, recording bags, and troubleshooting.
- [Evaluation guide](docs/evaluation.md): replay experiments, annotations,
  dataset layout, metrics, and reports.

## Quick start

Commands assume this repository is at `/root/dev_ws/src/anomaly_detection`.
Adjust the workspace path for your installation.

### Prepare and build

Use a ROS 2 installation with Python support, `colcon`, and `rosdep`. The commands
below use ROS 2 Jazzy and Python 3.12; use the Python interpreter associated with
your ROS installation.

The project's deployment environment is defined in the
[Dockerfile](https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/Dockerfile)
and its companion
[Python requirements](https://github.com/JACart2/docker_files/blob/main/services/anomaly_detection/requirements.txt).
The Docker environment installs the dependencies and downloads a default Ollama
model. If you use that environment, you do not need to install Ollama or download
the default model again. For a manual environment, install the dependencies from
the linked requirements file before building. A container should share ROS
connectivity with the nodes providing telemetry.

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/anomaly_detection --ignore-src -r -y
colcon build --symlink-install --packages-select anomaly_msg anomaly_detection tester
source install/setup.bash
```

### Configure and run

The Dockerfile downloads a default Ollama model. Check the model, endpoint, and
artifact output directory in
[`config.yaml`](anomaly_detection/anomaly_detection/config.yaml).
See the [usage guide](docs/usage.md#configure-inference) to download and select a
different model or configure a cloud provider.

From `/root/dev_ws`, after building and sourcing the workspace:

```bash
export AAD_CONFIG_PATH="$PWD/src/anomaly_detection/anomaly_detection/anomaly_detection/config.yaml"
ros2 run anomaly_detection anomaly_detection_node
```

In another terminal, source the ROS and workspace environments and publish a test
warning:

```bash
cd /root/dev_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once /ai_anomaly_logging anomaly_msg/msg/AnomalyMsg \
  "{node_name: 'readme_test', importance: 1, type: 0, msg: 'Localization updates are delayed; investigate sensor health.'}"
ros2 topic echo /aad/decisions
```

Decisions appear after scheduling and inference complete. Anomaly alerts are
published on `/aad/alerts`; invocation artifacts are written to the configured
output directory. See the [usage guide](docs/usage.md) for output formats and
troubleshooting.

## Making changes

**After every code change, run `colcon build`, then `source install/setup.bash`,
and restart affected nodes.** Run the build from `/root/dev_ws` using the package
selection above. Source the updated environment in every terminal launching nodes.
Follow the same sequence after changes to packaged configuration and resources.
See the [development guide](docs/development.md) for the full workflow and tests.

## Recording and evaluation

The [usage guide](docs/usage.md#bag-recording-ui) explains the bag recorder UI
and detector dashboard. For replay and labeling, start with the
[evaluation guide](docs/evaluation.md), which links to the detailed evaluator
and annotation manuals.

## Architecture

The detector caches telemetry and images, schedules local or cloud model calls,
normalizes model responses, and publishes decisions and alerts. High-severity
inputs can publish an immediate stop alert before inference completes.

- [Architecture diagram](docs/architecture_diagram.jpeg)
- [Dataflow diagram](docs/dataflow_diagram.jpeg)

## Attributions
Created by John Rosario Cruz (gr8jj28@gmail.com), Allie O'Keeffe, & Christopher Michael Hubbard.
