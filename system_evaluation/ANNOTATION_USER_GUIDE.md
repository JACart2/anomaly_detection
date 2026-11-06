# Anomaly Bag Annotation User Guide

This guide explains how to turn a recorded ROS 2 MCAP bag into an interactive annotation report, label the data, and save the resulting ground-truth annotations for later AI anomaly-detection testing.

The annotation workflow is designed around two persistent dataset files:

```text
<bag-name>.mcap
<bag-name>.annotations.json
```

The MCAP file is never modified. The annotation JSON is a sidecar file that stores bag-level metadata, message-level labels, and timestamp ranges.

## 1. Prerequisites

Run the extractor in the same ROS 2 environment used by the anomaly-detection project.

You need:

- ROS 2 Jazzy installed and sourced.
- The `anomaly_msg` package built and sourced so `anomaly_msg/msg/AnomalyMsg` can be deserialized.
- Python with the project dependencies available, including `rosbag2_py`, `rclpy`, `cv_bridge`, and OpenCV.
- An MCAP bag containing the anomaly logging topic. By default the extractor reads `/ai_anomaly_logging`.

Run the commands below from the repository root, with ROS and the workspace sourced.

## 2. Generate an Annotation Report

The extractor is:

```text
system_evaluation/scripts/extract_anomaly_bag.py
```

Basic usage:

```bash
python3 system_evaluation/scripts/extract_anomaly_bag.py path/to/bag.mcap
```

By default, this creates:

```text
system_evaluation/scripts/report/<bag-stem>_report/
├── index.html
├── messages.json
├── messages.csv
├── summary.json
└── images/
```

For dataset work, it is recommended to place the generated report inside the same bag directory while keeping the annotation sidecar outside the report directory:

```text
system_evaluation/
└── bags/
    └── driver_missing_001/
        ├── driver_missing_001.mcap
        ├── driver_missing_001.annotations.json
        └── report/
            ├── index.html
            ├── messages.json
            ├── messages.csv
            ├── summary.json
            └── images/
```

Example:

```bash
python3 system_evaluation/scripts/extract_anomaly_bag.py \
  system_evaluation/bags/driver_missing_001/driver_missing_001.mcap \
  -o system_evaluation/bags/driver_missing_001/report
```

If the bag uses a different AnomalyMsg topic, specify it with `-t`:

```bash
python3 system_evaluation/scripts/extract_anomaly_bag.py \
  path/to/bag.mcap \
  -t /your/anomaly/topic
```

The selected topic must contain `anomaly_msg/msg/AnomalyMsg` messages.

## 3. Open the Report

Open the generated:

```text
report/index.html
```

in a web browser.

No backend server is required. The page contains the extracted messages, images, search/filter controls, bag metadata form, message annotation controls, time-range annotation tools, JSON import/export, and browser autosave.

## 4. Fill In Bag Metadata

At the top of the report, use the **Bag metadata** section to describe the recording as a whole.

Available fields are:

| Field | Purpose |
| --- | --- |
| Bag file | Original MCAP filename. Read-only. |
| Scenario | Short name for the test scenario, such as `Driver missing`. |
| Category | High-level anomaly category. |
| Anomalous | Whether the recording is expected to contain an anomaly. |
| Description | Explanation of what happens in the recording. |
| Environment | Where or under what conditions the run was recorded. |
| Cart | Cart/platform used for the run. |
| Annotator | Person creating the ground truth. |
| Notes | Any additional information useful for later testing. |

Current category choices are:

- Driver issue
- Dynamic obstacle
- Static obstacle
- Mechanical issue
- Sensor issue
- Unauthorized access
- Route issue
- Other

Bag metadata is automatically saved in the browser while you work.

## 5. Review and Filter Messages

The message table shows the extracted AnomalyMsg records.

Each row includes:

| Column | Description |
| --- | --- |
| # | Message index within the extracted report. |
| Recorded time | UTC time and exact recorded timestamp in nanoseconds. |
| Node | Source node recorded in the AnomalyMsg. |
| Importance | INFO, WARNING, or ERROR. |
| Type | TEXT, IMAGE, or DATA. |
| Contents | Message text or data payload information. |
| Image | Extracted camera image when present. |
| Label | Ground-truth label for this message. |
| Category | Anomaly category for this message. |
| Annotation | Free-text explanation plus range controls. |

Use the controls above the table to filter by search text, importance, message type, or annotation label.

The search box also searches annotation labels, categories, and annotation notes.

## 6. Annotate Individual Messages

Each message can be assigned one of five labels:

| Label | Use |
| --- | --- |
| Unlabeled | The message has not been reviewed or intentionally has no annotation. |
| Normal | The message represents normal system behavior. |
| Anomaly | The message is part of an anomalous event. |
| Uncertain | The annotator cannot confidently classify the message. |
| Ignore | The message should not be used as ground truth. |

For a message you want to annotate:

1. Choose a **Label**.
2. Select a **Category** when appropriate.
3. Add a short explanation in the **Annotation** field.

Only meaningful message annotations are stored in the exported annotation document. The report keeps the exact recorded timestamp, topic, node name, and message index so the annotation can be matched back to the original bag data.

The recorded nanosecond timestamp is the primary event identity. The message index is retained as useful provenance but should not be treated as the only identifier.

## 7. Annotate a Time Range

Use a time range when an anomaly lasts across multiple messages instead of labeling every message individually.

A row has two buttons:

- **Set range start**
- **Set range end**

Recommended workflow:

1. Find the first message that belongs to the event and click **Set range start**.
2. Find the last message that belongs to the event and click **Set range end**.
3. In the **Time ranges** section, choose the range label and category.
4. Add a description of what happens during the range.
5. Click **Save range**.

The report stores the exact start and end timestamps in nanoseconds and, when the endpoints were selected from message rows, also preserves the corresponding message indexes.

You can also type exact nanosecond timestamps directly into the range editor.

Saved ranges are shown in the range list. Existing ranges can be edited or deleted.

The end timestamp cannot be earlier than the start timestamp.

## 8. Autosave

Annotation work is automatically saved to browser `localStorage`.

The report displays a status such as:

```text
Saved locally 2:34:18 PM
```

When the same report is reopened in the same browser, locally saved annotation work is restored automatically.

Browser autosave is only a convenience and is **not** the canonical dataset.

If browser storage is unavailable, the page displays a warning. In that case, export the annotation JSON regularly.

## 9. Export the Annotation JSON

When the annotation is ready, click:

**Export annotations**

The browser downloads:

```text
<bag-stem>.annotations.json
```

For example:

```text
driver_missing_001.annotations.json
```

Move or save this file beside the original MCAP:

```text
driver_missing_001/
├── driver_missing_001.mcap
├── driver_missing_001.annotations.json
└── report/
```

Do **not** keep the canonical annotation file inside the generated report directory.

The report directory is considered regenerable output. The extractor protects annotation files by refusing to delete a report directory if it finds an `*.annotations.json` file inside it, but the intended location is still beside the source bag or in another persistent dataset directory.

## 10. Import Existing Annotations

To continue or revise previous work:

1. Open the report for the desired MCAP.
2. Click **Import annotations**.
3. Select the matching `.annotations.json` file.

The report restores bag metadata, message annotations, and time ranges.

If the imported annotation file names a different bag, the page warns you and asks whether you want to import it anyway.

After a successful import, the imported state is also saved to browser local storage.

## 11. Annotation File Format

Annotation files use:

```json
{
  "schema_version": 1,
  "bag": {},
  "ranges": [],
  "annotations": []
}
```

The formal JSON Schema is stored at:

```text
system_evaluation/schemas/anomaly_annotations.schema.json
```

A message annotation contains fields such as:

```json
{
  "message_index": 184,
  "recorded_timestamp_ns": 1785178501234567890,
  "topic": "/ai_anomaly_logging",
  "node_name": "zed_camera",
  "label": "anomaly",
  "category": "Driver issue",
  "description": "Driver is no longer visible."
}
```

A range annotation contains fields such as:

```json
{
  "start_timestamp_ns": 1785178501234567890,
  "end_timestamp_ns": 1785178510234567890,
  "start_message_index": 184,
  "end_message_index": 191,
  "label": "anomaly",
  "category": "Driver issue",
  "description": "Driver remains absent from the seat."
}
```

Do not manually round or shorten the nanosecond timestamps. They are intentionally preserved at full precision.

## 12. Regenerating a Report

The original MCAP is read-only during extraction.

You can regenerate the report from the same bag at any time:

```bash
python3 system_evaluation/scripts/extract_anomaly_bag.py path/to/bag.mcap -o path/to/report
```

The extractor recreates the output directory.

If an `*.annotations.json` file is found inside the report directory, regeneration stops instead of deleting it. Move that annotation file beside the source bag, then run the extractor again.

After regenerating the report, use **Import annotations** to load the persistent sidecar file if the browser does not automatically restore the previous local state.

## 13. Recommended Annotation Workflow

For each dataset recording:

1. Keep the original MCAP unchanged.
2. Generate the report into a dedicated `report/` directory.
3. Open `report/index.html`.
4. Fill in the bag metadata.
5. Review the messages and images.
6. Add message-level annotations for important individual events.
7. Add time ranges for events that span multiple messages.
8. Use `Uncertain` instead of guessing when ground truth is ambiguous.
9. Export the annotation JSON.
10. Store the exported sidecar beside the MCAP.
11. Commit or otherwise version the MCAP metadata/annotation sidecar according to the dataset workflow.

This produces a repeatable pair of raw evidence and human-created ground truth that can later be consumed by the AAD evaluation pipeline.

## 14. Troubleshooting

### Topic not found

If the extractor reports that `/ai_anomaly_logging` is missing, inspect the topics in the bag and rerun with the correct topic:

```bash
python3 system_evaluation/scripts/extract_anomaly_bag.py bag.mcap -t /correct/topic
```

### Wrong message type

The selected topic must use:

```text
anomaly_msg/msg/AnomalyMsg
```

Make sure the project workspace containing `anomaly_msg` has been built and sourced.

### Python cannot import ROS modules

Source the ROS 2 installation and the project workspace before running the extractor.

For example, on Linux:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Then rerun the command from the repository/workspace environment.

### Report regeneration is refused

If you see a message explaining that annotation sidecars were found in the report directory, move the `*.annotations.json` file out of the report directory and place it beside the MCAP.

This safeguard exists to prevent annotation data from being deleted.

### Autosave is unavailable

The report will warn you if browser local storage cannot be used. Continue annotating, but use **Export annotations** frequently to preserve your work.

### Imported annotations belong to another bag

The report checks the `bag.file` field during import. Verify that you selected the correct sidecar before choosing to import annotations created for a different bag.

## 15. What the Annotation Data Is For

The annotation sidecar supplies ground truth to the offline evaluator in `human-labeled` mode. Set the bag metadata’s Anomalous field to Yes or No before exporting.

Bag-level metadata describes what the entire test run represents. Message annotations identify individual pieces of evidence. Time ranges identify when an event starts and ends.

The evaluator reports bag classification and event-level precision, recall, F1, and timing diagnostics. See the [Offline Evaluator Manual](OFFLINE_EVALUATOR_MANUAL.md) to configure parameter comparisons and run evaluation.
