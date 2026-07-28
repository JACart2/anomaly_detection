#!/usr/bin/env python3
"""Extract anomaly_msg/AnomalyMsg records and embedded images from an MCAP bag."""

from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import shutil
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


IMPORTANCE_NAMES = {0: "INFO", 1: "WARNING", 2: "ERROR"}
TYPE_NAMES = {0: "TEXT", 1: "IMAGE", 2: "DATA"}


def iso_time(nanoseconds: int) -> str:
    seconds, nanos = divmod(nanoseconds, 1_000_000_000)
    stamp = datetime.fromtimestamp(seconds, timezone.utc)
    return f"{stamp:%Y-%m-%dT%H:%M:%S}.{nanos:09d}Z"


def ros_stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def text_cell(value: object) -> str:
    return html.escape(str(value), quote=True)


def write_html(
    output: Path,
    bag: Path,
    records: list[dict],
    importance_counts: Counter,
    type_counts: Counter,
    image_count: int,
) -> None:
    rows = []
    for record in records:
        image = record["image"]
        image_html = "—"
        if image.get("file"):
            image_html = (
                f'<a href="{text_cell(image["file"])}">'
                f'<img loading="lazy" src="{text_cell(image["file"])}" '
                f'alt="Image from message {record["index"]}"></a>'
            )
        elif image.get("raw_file"):
            image_html = f'<a href="{text_cell(image["raw_file"])}">raw bytes</a>'

        details = []
        if record["message"]:
            details.append(f'<div class="message">{text_cell(record["message"])}</div>')
        if record["data_type"] or record["data_base64"]:
            details.append(
                "<details><summary>Data payload "
                f'({record["data_length"]} bytes, {text_cell(record["data_type"])})'
                f'</summary><code>{text_cell(record["data_base64"])}</code></details>'
            )
        if image.get("error"):
            details.append(f'<div class="error">{text_cell(image["error"])}</div>')

        searchable = " ".join(
            [
                record["timestamp"],
                record["node_name"],
                record["importance_name"],
                record["type_name"],
                record["message"],
                record["data_type"],
            ]
        ).lower()
        rows.append(
            f"""<tr data-search="{text_cell(searchable)}"
                data-importance="{text_cell(record["importance_name"])}"
                data-type="{text_cell(record["type_name"])}">
              <td>{record["index"]}</td>
              <td><time>{text_cell(record["timestamp"])}</time></td>
              <td>{text_cell(record["node_name"])}</td>
              <td><span class="badge {record["importance_name"].lower()}">{text_cell(record["importance_name"])}</span></td>
              <td>{text_cell(record["type_name"])}</td>
              <td>{''.join(details) or "—"}</td>
              <td>{image_html}</td>
            </tr>"""
        )

    summary = ", ".join(
        f"{name}: {importance_counts.get(value, 0)}"
        for value, name in IMPORTANCE_NAMES.items()
    )
    type_summary = ", ".join(
        f"{name}: {type_counts.get(value, 0)}" for value, name in TYPE_NAMES.items()
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Anomaly bag report</title>
  <style>
    :root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
    body {{ margin: 0; padding: 1.25rem; background: #101418; color: #e8edf2; }}
    h1 {{ margin: 0 0 .3rem; }}
    .muted {{ color: #9aa7b3; }}
    .summary {{ display: flex; flex-wrap: wrap; gap: .7rem; margin: 1rem 0; }}
    .card {{ background: #1a2027; border: 1px solid #303943; border-radius: .6rem; padding: .7rem 1rem; }}
    .controls {{ position: sticky; top: 0; z-index: 2; display: flex; gap: .6rem;
                 flex-wrap: wrap; padding: .7rem 0; background: #101418ee; }}
    input, select {{ font: inherit; padding: .55rem; color: inherit; background: #1a2027;
                     border: 1px solid #46525e; border-radius: .4rem; }}
    input {{ min-width: min(28rem, 80vw); }}
    table {{ width: 100%; border-collapse: collapse; font-size: .9rem; }}
    th {{ position: sticky; top: 4.1rem; background: #1a2027; text-align: left; }}
    th, td {{ border-bottom: 1px solid #303943; padding: .55rem; vertical-align: top; }}
    tbody tr:hover {{ background: #192129; }}
    img {{ width: 220px; max-height: 180px; object-fit: contain; background: #050709; }}
    .message {{ white-space: pre-wrap; max-width: 42rem; }}
    code {{ white-space: pre-wrap; overflow-wrap: anywhere; font-size: .75rem; }}
    details {{ margin-top: .4rem; max-width: 42rem; }}
    .badge {{ padding: .18rem .38rem; border-radius: .3rem; font-weight: 650; }}
    .info {{ background: #17466b; }} .warning {{ background: #735c12; }}
    .error {{ background: #70232b; }} .error:not(.badge) {{ color: #ff919d; }}
    @media (max-width: 800px) {{ th:nth-child(3), td:nth-child(3) {{ display: none; }} }}
  </style>
</head>
<body>
  <h1>Anomaly bag report</h1>
  <div class="muted">{text_cell(bag.name)}</div>
  <div class="summary">
    <div class="card"><strong>{len(records):,}</strong> messages</div>
    <div class="card"><strong>{image_count:,}</strong> extracted images</div>
    <div class="card">{text_cell(summary)}</div>
    <div class="card">{text_cell(type_summary)}</div>
  </div>
  <div class="controls">
    <input id="search" type="search" placeholder="Filter timestamp, node, level, type, or message">
    <select id="importance"><option value="">All importance levels</option>
      <option>INFO</option><option>WARNING</option><option>ERROR</option>
    </select>
    <select id="type"><option value="">All message types</option>
      <option>TEXT</option><option>IMAGE</option><option>DATA</option>
    </select>
    <span id="visible" class="muted"></span>
  </div>
  <table>
    <thead><tr><th>#</th><th>Recorded time (UTC)</th><th>Node</th>
      <th>Importance</th><th>Type</th><th>Contents</th><th>Image</th></tr></thead>
    <tbody>{''.join(rows)}</tbody>
  </table>
  <script>
    const rows = [...document.querySelectorAll("tbody tr")];
    const search = document.querySelector("#search");
    const importance = document.querySelector("#importance");
    const type = document.querySelector("#type");
    const visible = document.querySelector("#visible");
    function filterRows() {{
      const query = search.value.trim().toLowerCase();
      let count = 0;
      for (const row of rows) {{
        const show = (!query || row.dataset.search.includes(query))
          && (!importance.value || row.dataset.importance === importance.value)
          && (!type.value || row.dataset.type === type.value);
        row.hidden = !show;
        if (show) count++;
      }}
      visible.textContent = `${{count.toLocaleString()}} shown`;
    }}
    search.addEventListener("input", filterRows);
    importance.addEventListener("change", filterRows);
    type.addEventListener("change", filterRows);
    filterRows();
  </script>
</body>
</html>
"""
    (output / "index.html").write_text(document, encoding="utf-8")


def extract(bag: Path, output: Path) -> None:
    if output.exists():
        shutil.rmtree(output)
    images_dir = output / "images"
    images_dir.mkdir(parents=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }
    bridge = CvBridge()
    records: list[dict] = []
    importance_counts: Counter = Counter()
    type_counts: Counter = Counter()
    image_count = 0

    while reader.has_next():
        topic, serialized, recorded_ns = reader.read_next()
        msg = deserialize_message(serialized, message_types[topic])
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
            except Exception as exc:  # Keep original bytes if conversion is unsupported.
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
        "topics": topic_types,
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Input .mcap file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output directory (default: <bag-name>_report)",
    )
    args = parser.parse_args()
    bag = args.bag.resolve()
    if not bag.is_file():
        parser.error(f"Bag does not exist: {bag}")
    output = (args.output or Path(f"{bag.stem}_report")).resolve()
    if output == bag.parent or output == bag:
        parser.error("Output must be a dedicated directory")
    extract(bag, output)


if __name__ == "__main__":
    main()
