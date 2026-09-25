#!/usr/bin/env python3
"""Extract anomaly_msg/AnomalyMsg records and embedded images from an MCAP bag."""

from __future__ import annotations

import argparse
import base64
import csv
import json
import os
import shutil
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

try:
    from anomaly_annotation_report import write_html
except ModuleNotFoundError:  # Support importing from the repository root in tests.
    from scripts.anomaly_annotation_report import write_html


IMPORTANCE_NAMES = {0: "INFO", 1: "WARNING", 2: "ERROR"}
TYPE_NAMES = {0: "TEXT", 1: "IMAGE", 2: "DATA"}
ANOMALY_MSG_TYPE = "anomaly_msg/msg/AnomalyMsg"
DEFAULT_ANOMALY_TOPIC = "/ai_anomaly_logging"


def register_windows_dll_directories() -> list[object]:
    """Make DLLs from sourced ROS/Pixi environments visible to Python."""
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return []

    handles = []
    for entry in os.environ.get("PATH", "").split(os.pathsep):
        if not entry or not Path(entry).is_dir():
            continue
        try:
            handles.append(os.add_dll_directory(entry))
        except OSError:
            continue
    return handles


def iso_time(nanoseconds: int) -> str:
    """Convert an epoch timestamp in nanoseconds to an exact UTC string."""
    seconds, nanos = divmod(nanoseconds, 1_000_000_000)
    stamp = datetime.fromtimestamp(seconds, timezone.utc)
    return f"{stamp:%Y-%m-%dT%H:%M:%S}.{nanos:09d}Z"


def ros_stamp_ns(stamp: object) -> int:
    """Convert a ROS time object to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def prepare_output_directory(output: Path) -> None:
    """Recreate a report directory without silently deleting annotations."""
    if output.exists():
        annotation_files = sorted(output.rglob("*.annotations.json"))
        if annotation_files:
            found = "\n  ".join(str(path) for path in annotation_files)
            raise SystemExit(
                "Refusing to regenerate the report because it contains annotation "
                f"sidecars. Move them beside the bag first:\n  {found}"
            )
        shutil.rmtree(output)
    (output / "images").mkdir(parents=True)


def read_records(bag: Path, output: Path, topic: str) -> tuple:
    """Read the selected AnomalyMsg topic and extract its image payloads."""
    dll_directory_handles = register_windows_dll_directories()
    import cv2
    import rosbag2_py
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    # Keep the handles alive while compiled extension modules are in use.
    _ = dll_directory_handles

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise SystemExit(
            f"Topic {topic!r} not found in bag. Available topics: "
            f"{', '.join(sorted(topic_types)) or '(none)'}"
        )
    if topic_types[topic] != ANOMALY_MSG_TYPE:
        raise SystemExit(
            f"Topic {topic!r} is type {topic_types[topic]!r}, expected "
            f"{ANOMALY_MSG_TYPE!r}"
        )

    message_type = get_message(topic_types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    bridge = CvBridge()
    records: list[dict] = []
    importance_counts: Counter = Counter()
    type_counts: Counter = Counter()
    image_count = 0
    images_dir = output / "images"

    while reader.has_next():
        _topic, serialized, recorded_ns = reader.read_next()
        msg = deserialize_message(serialized, message_type)
        index = len(records) + 1
        importance = int(msg.importance)
        msg_type = int(msg.type)
        importance_counts[importance] += 1
        type_counts[msg_type] += 1
        image = msg.image
        image_info = {
            "width": int(image.width),
            "height": int(image.height),
            "encoding": image.encoding,
            "step": int(image.step),
            "is_bigendian": bool(image.is_bigendian),
            "frame_id": image.header.frame_id,
            "timestamp_ns": ros_stamp_ns(image.header.stamp),
            "data_length": len(image.data),
            "file": "",
            "raw_file": "",
            "error": "",
        }

        if msg_type == 1 and image.data:
            stem = f"{index:06d}_{recorded_ns}"
            try:
                cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
                filename = f"{stem}.png"
                if not cv2.imwrite(str(images_dir / filename), cv_image):
                    raise RuntimeError("OpenCV could not encode this image as PNG")
                image_info["file"] = f"images/{filename}"
                image_count += 1
            except Exception as exc:  # Keep original bytes when conversion fails.
                filename = f"{stem}.bin"
                (images_dir / filename).write_bytes(bytes(image.data))
                image_info["raw_file"] = f"images/{filename}"
                image_info["error"] = f"Image conversion failed: {exc}"

        header_ns = ros_stamp_ns(msg.header.stamp)
        data_bytes = bytes(msg.data)
        records.append(
            {
                "index": index,
                "topic": topic,
                "recorded_timestamp_ns": int(recorded_ns),
                "timestamp": iso_time(int(recorded_ns)),
                "header_timestamp_ns": header_ns,
                "header_timestamp": iso_time(header_ns) if header_ns else "",
                "frame_id": msg.header.frame_id,
                "node_name": msg.node_name,
                "importance": importance,
                "importance_name": IMPORTANCE_NAMES.get(
                    importance, f"UNKNOWN_{importance}"
                ),
                "type": msg_type,
                "type_name": TYPE_NAMES.get(msg_type, f"UNKNOWN_{msg_type}"),
                "message": msg.msg,
                "image": image_info,
                "data_type": msg.data_type,
                "data_length": len(data_bytes),
                "data_base64": base64.b64encode(data_bytes).decode("ascii"),
            }
        )

    return records, importance_counts, type_counts, image_count, topic_types


def write_machine_readable_outputs(
    output: Path,
    bag: Path,
    topic: str,
    records: list[dict],
    importance_counts: Counter,
    type_counts: Counter,
    image_count: int,
    topic_types: dict,
) -> dict:
    """Write the existing JSON, CSV, and summary report artifacts."""
    with (output / "messages.json").open("w", encoding="utf-8") as stream:
        json.dump(records, stream, indent=2, ensure_ascii=False)

    csv_fields = [
        "index",
        "topic",
        "recorded_timestamp_ns",
        "timestamp",
        "header_timestamp_ns",
        "header_timestamp",
        "frame_id",
        "node_name",
        "importance",
        "importance_name",
        "type",
        "type_name",
        "message",
        "data_type",
        "data_length",
        "data_base64",
        "image_file",
        "image_width",
        "image_height",
        "image_encoding",
    ]
    with (output / "messages.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields)
        writer.writeheader()
        for record in records:
            row = {field: record.get(field, "") for field in csv_fields}
            row.update(
                image_file=record["image"].get("file")
                or record["image"].get("raw_file"),
                image_width=record["image"]["width"],
                image_height=record["image"]["height"],
                image_encoding=record["image"]["encoding"],
            )
            writer.writerow(row)

    metadata = {
        "source_bag": str(bag.resolve()),
        "anomaly_topic": topic,
        "bag_topics": topic_types,
        "message_count": len(records),
        "image_count": image_count,
        "importance_counts": {
            IMPORTANCE_NAMES.get(key, str(key)): value
            for key, value in sorted(importance_counts.items())
        },
        "type_counts": {
            TYPE_NAMES.get(key, str(key)): value
            for key, value in sorted(type_counts.items())
        },
    }
    (output / "summary.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    return metadata


def extract(bag: Path, output: Path, topic: str) -> None:
    """Extract a bag and generate its machine-readable and HTML reports."""
    prepare_output_directory(output)
    (
        records,
        importance_counts,
        type_counts,
        image_count,
        topic_types,
    ) = read_records(bag, output, topic)
    metadata = write_machine_readable_outputs(
        output,
        bag,
        topic,
        records,
        importance_counts,
        type_counts,
        image_count,
        topic_types,
    )
    write_html(
        output,
        bag,
        records,
        importance_counts,
        type_counts,
        image_count,
    )
    print(json.dumps(metadata, indent=2))
    print(f"\nReport: {output / 'index.html'}")


def main() -> None:
    """Parse command-line arguments and extract the requested MCAP bag."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Input .mcap file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output directory (default: <bag-name>_report)",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default=DEFAULT_ANOMALY_TOPIC,
        help=f"AnomalyMsg topic to extract (default: {DEFAULT_ANOMALY_TOPIC})",
    )
    args = parser.parse_args()
    bag = args.bag.resolve()
    if not bag.is_file():
        parser.error(f"Bag does not exist: {bag}")
    output = (args.output or Path(f"{bag.stem}_report")).resolve()
    if output == bag.parent or output == bag:
        parser.error("Output must be a dedicated directory")
    extract(bag, output, args.topic)


if __name__ == "__main__":
    main()
