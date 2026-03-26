"""Evaluation harness for the AAD (AI Anomaly Detection) node.
Iterates over a dataset CSV and a list of config files, plays each bag file,
collects alerts from /aad/alerts, and compares against ground truth labels.

Author: AAD Team Spring 26'
Version: 3/22/2026
"""
import os
import csv
import json
import time
import subprocess
import threading
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ALERT_TOPIC = "/aad/decisions"


class AlertCollector(Node):
    """
    Description
    ------------
        Lightweight ROS 2 node that subscribes to /aad/alerts and collects
        incoming alert strings for comparison against ground truth labels.

    Attributes
    ----------
        alerts (list[str]): List of raw alert strings received on /aad/alerts
                            since the last clear().

    Methods
    -------
        _callback(msg: String):
            Subscription callback. Appends incoming alert data to self.alerts.
    """

    def __init__(self):
        super().__init__("eval_alert_collector")
        self.alerts = []
        self.create_subscription(String, ALERT_TOPIC, self._callback, 10)

    def _callback(self, msg: String) -> None:
        """
        Subscription callback for /aad/alerts.

        Args:
            msg (String): Incoming alert message from the AAD node.
        """
        self.get_logger().info(f"Alert received: {msg.data}")
        self.alerts.append(msg.data)


def parse_alert(alert_str: str) -> dict:
    """
    Parses a raw AAD alert string into a structured dictionary.

    Expected format: '[AAD ALERT] severity=X action=Y summary=Z'

    Args:
        alert_str (str): The raw alert string published to /aad/alerts.

    Returns:
        dict: Parsed fields including severity, action, summary, anomaly flag,
              and the original raw string.
    """
    result = {"raw": alert_str, "anomaly": True}
    for field in ("severity", "action", "summary"):
        try:
            result[field] = alert_str.split(f"{field}=")[1].split(" ")[0]
        except IndexError:
            result[field] = "unknown"
    return result


def run_evaluation(csv_path: str, config_paths: list[str], bags_dir: str) -> list:
    """
    Main evaluation loop. For each config, launches the AAD node with that config,
    plays each bag file from the dataset CSV, collects alerts, and compares against
    ground truth.

    Args:
        csv_path (str): Path to the dataset outline CSV file.
        config_paths (list[str]): List of config.yaml paths to evaluate.
        bags_dir (str): Directory containing the .mcap bag files.

    Returns:
        list[dict]: All per-bag results across all configs, each containing:
                    config, bag_file, category, description, expected,
                    detected, correct, and parsed alerts.
    """
    rclpy.init()
    collector = AlertCollector()

    # Spin the collector in a background thread so it can receive alerts
    # while the main thread manages bag playback and evaluation logic
    spin_thread = threading.Thread(target=rclpy.spin, args=(collector,), daemon=True)
    spin_thread.start()

    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    all_results = []

    for config_path in config_paths:
        abs_config = os.path.abspath(config_path)
        print(f"\n{'='*60}")
        print(f"Config: {abs_config}")
        print(f"{'='*60}")

        # Pass config path via AAD_CONFIG_PATH so the node picks it up on startup
        # without requiring ROS parameter changes (matches _load_config() resolution order)
        env = {**os.environ, "AAD_CONFIG_PATH": abs_config}

        node_proc = subprocess.Popen(
            ["ros2", "run", "anomaly_detection", "anomaly_detection_node"],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        try:
            for row in rows:
                bag_file    = row["Bag file"].strip()
                category    = row["Anomaly Category"].strip()
                description = row["Description"].strip()
                expected    = row["Anomalous"].strip()  # "Yes" or "No"
                bag_path    = os.path.join(bags_dir, bag_file)

                if not os.path.exists(bag_path):
                    print(f"  ! Skipping — bag not found: {bag_path}")
                    continue

                # Clear alerts from the previous bag before playing the next one
                # so results are isolated per bag file
                collector.alerts.clear()

                print(f"  Playing {bag_file}...", end=" ", flush=True)
                bag_proc = subprocess.Popen(
                    ["ros2", "bag", "play", bag_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                bag_proc.wait()

                # Brief pause after bag finishes to allow the LLM timer to fire
                # and the alert to arrive on /aad/alerts before we check results.
                # Increase this if api_frequency_seconds in the config is large.
                time.sleep(1.0)

                # The AAD node only publishes to /aad/alerts when decision.anomaly
                # is True, so no alerts = no anomaly detected
                detected  = len(collector.alerts) > 0
                correct   = detected == (expected == "Yes")
                status    = "✓" if correct else "✗"
                parsed    = [parse_alert(a) for a in collector.alerts]

                print(f"{status} | expected={expected:3s} | detected={detected} | {category} — {description}")
                if collector.alerts:
                    for a in collector.alerts:
                        print(f"      → {a}")

                all_results.append({
                    "config":      abs_config,
                    "bag_file":    bag_file,
                    "category":    category,
                    "description": description,
                    "expected":    expected,
                    "detected":    detected,
                    "correct":     correct,
                    "alerts":      parsed,
                })

        finally:
            # Always terminate the node process before moving to the next config
            # to avoid stale subscriptions or duplicate alert publishers
            node_proc.terminate()
            node_proc.wait()

    rclpy.shutdown()

    # Print per-config summary with accuracy and confusion matrix counts
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    for config_path in config_paths:
        abs_config     = os.path.abspath(config_path)
        config_results = [r for r in all_results if r["config"] == abs_config]
        total          = len(config_results)
        if total == 0:
            continue
        correct = sum(r["correct"] for r in config_results)
        tp = sum(1 for r in config_results if r["detected"] and r["expected"] == "Yes")
        fp = sum(1 for r in config_results if r["detected"] and r["expected"] == "No")
        fn = sum(1 for r in config_results if not r["detected"] and r["expected"] == "Yes")
        print(f"  {os.path.basename(config_path)}: {correct}/{total} ({100*correct/total:.1f}%) | TP={tp} FP={fp} FN={fn}")

    return all_results

# Example Usage
# python3 eval.py \
#   --csv dataset_outline.csv \
#   --configs configs/gpt4o.yaml configs/ollama.yaml \
#   --bags /path/to/bags \
#   --output results.json
def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Eval AAD node across bags and configs.")
    parser.add_argument("--csv",     required=True,              help="Path to dataset_outline.csv")
    parser.add_argument("--configs", required=True, nargs="+",   help="One or more config.yaml paths")
    parser.add_argument("--bags",    default="bags",             help="Directory containing .mcap bag files")
    parser.add_argument("--output",  default="eval_results.json", help="Output JSON path")
    args = parser.parse_args()

    results = run_evaluation(args.csv, args.configs, args.bags)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()