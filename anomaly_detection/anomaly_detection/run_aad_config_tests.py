"""Evaluation harness for the AAD (AI Anomaly Detection) node.
Iterates over a dataset CSV and a list of config files, plays each bag file,
collects alerts from /aad/alerts, and compares against ground truth labels.

Author: AAD Team Spring 26'
Version: 3/22/2026 (FIXED)

MAJOR CHANGE: ROS context now lives in parent process, not subprocess.
Only "ros2 bag play" runs as subprocess. AAD node created once per config.
"""
import os
import csv
import json
import argparse
import time
import subprocess
import threading
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
from rclpy.executors import MultiThreadedExecutor

# Import your actual node
from anomaly_detection.anomaly_detection_node import AnomalyDetectionNode

ALERT_TOPIC = "/aad/decisions"
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))


class AlertCollector(Node):
    """
    Lightweight ROS 2 node that subscribes to /aad/decisions and collects
    incoming decision strings for comparison against ground truth labels.
    """

    def __init__(self):
        super().__init__("eval_alert_collector")
        self.alerts = []
        self.create_subscription(String, ALERT_TOPIC, self._callback, 10)

    def _callback(self, msg: String) -> None:
        """Subscription callback for /aad/decisions."""
        self.get_logger().info(f"Decision received: {msg.data}")
        self.alerts.append(msg.data)


def parse_alert(alert_str: str) -> dict:
    """
    Parses a raw AAD alert string into a structured dictionary.

    Expected format: 'anomaly=X severity=Y action=Z summary=W'

    Args:
        alert_str (str): The raw alert string published to /aad/decisions.

    Returns:
        dict: Parsed fields including severity, action, summary, anomaly flag,
              and the original raw string.
    """
    result = {"raw": alert_str, "anomaly": False}
    
    # Parse anomaly flag
    if "anomaly=True" in alert_str or "anomaly=true" in alert_str:
        result["anomaly"] = True
    
    # Parse fields
    for field in ("severity", "action", "summary"):
        try:
            result[field] = alert_str.split(f"{field}=")[1].split(" ")[0]
        except IndexError:
            result[field] = "unknown"
    
    return result


def run_evaluation(csv_path: str, config_paths: list[str], bags_dir: str) -> list:
    """
    Run evaluation across all configs and bags.
    
    Key architecture:
    - ROS context lives in parent process (this script)
    - AAD node created once per config and reused across bags
    - Only "ros2 bag play" runs as subprocess
    - Config passed via environment variable (AAD_CONFIG_PATH)
    """
    
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    all_results = []
    
    # Initialize ROS ONCE for entire evaluation
    rclpy.init()

    try:
        for config_path in config_paths:
            abs_config = os.path.abspath(config_path)

            print(f"\n{'='*60}")
            print(f"Config: {abs_config}")
            print(f"{'='*60}")

            # Set env var for this config
            # Child processes inherit this, but node reads it from environment at init time
            os.environ["AAD_CONFIG_PATH"] = abs_config
            
            # Create AAD node and collector for this config
            # Node reads AAD_CONFIG_PATH from os.environ in _load_config()
            aad_node = AnomalyDetectionNode()
            collector = AlertCollector()

            # Single executor for both nodes
            executor = MultiThreadedExecutor()
            executor.add_node(aad_node)
            executor.add_node(collector)

            # Spin in background thread (NOT daemon, so it can join cleanly)
            spin_thread = threading.Thread(target=executor.spin, daemon=False)
            spin_thread.start()

            # Small delay to let nodes initialize and subscribe
            time.sleep(0.5)

            try:
                for row in rows:
                    bag_file = row["Bag file"].strip()
                    category = row["Anomaly Category"].strip()
                    description = row["Description"].strip()
                    expected = row["Anomalous"].strip()
                    bag_path = os.path.join(bags_dir, bag_file)

                    if not os.path.exists(bag_path):
                        print(f"  ! Skipping — bag not found: {bag_path}")
                        continue

                    # Clear alerts from previous bag
                    collector.alerts.clear()

                    print(f"  Playing {bag_file}...", end=" ", flush=True)

                    # ONLY subprocess is bag playback
                    bag_proc = subprocess.Popen(
                        ["ros2", "bag", "play", bag_path],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    bag_proc.wait()

                    # Wait for alerts to arrive
                    start = time.time()
                    timeout_sec = 10
                    while time.time() - start < timeout_sec:
                        if collector.alerts:
                            break
                        time.sleep(0.5)

                    # Evaluate
                    detected = len(collector.alerts) > 0
                    correct = detected == (expected == "Yes")
                    status = "✓" if correct else "✗"
                    parsed = [parse_alert(a) for a in collector.alerts]

                    print(f"{status} | expected={expected:3s} | detected={detected} | {category} — {description}")

                    if collector.alerts:
                        for a in collector.alerts:
                            print(f"      → {a}")

                    all_results.append({
                        "config": abs_config,
                        "bag_file": bag_file,
                        "category": category,
                        "description": description,
                        "expected": expected,
                        "detected": detected,
                        "correct": correct,
                        "alerts": parsed,
                    })

            finally:
                # Clean shutdown: signal executor, join thread, destroy nodes
                executor.shutdown()
                
                # Join with timeout
                spin_thread.join(timeout=5)
                if spin_thread.is_alive():
                    print(f"WARNING: Spinner thread did not exit for config {os.path.basename(config_path)}")
                
                # Destroy nodes
                try:
                    collector.destroy_node()
                except Exception as e:
                    print(f"Warning destroying collector: {e}")
                
                try:
                    aad_node.destroy_node()
                except Exception as e:
                    print(f"Warning destroying aad_node: {e}")

        # Print summary
        print(f"\n{'='*60}")
        print("SUMMARY")
        print(f"{'='*60}")

        for config_path in config_paths:
            abs_config = os.path.abspath(config_path)
            config_results = [r for r in all_results if r["config"] == abs_config]

            total = len(config_results)
            if total == 0:
                continue

            correct = sum(r["correct"] for r in config_results)
            tp = sum(1 for r in config_results if r["detected"] and r["expected"] == "Yes")
            fp = sum(1 for r in config_results if r["detected"] and r["expected"] == "No")
            fn = sum(1 for r in config_results if not r["detected"] and r["expected"] == "Yes")

            pct = 100 * correct / total if total > 0 else 0
            print(f"  {os.path.basename(config_path)}: {correct}/{total} ({pct:.1f}%) | TP={tp} FP={fp} FN={fn}")

    finally:
        # Global ROS shutdown
        rclpy.shutdown()

    return all_results


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Eval AAD node across bags and configs.")
    parser.add_argument("--csv", required=True, help="Path to dataset_outline.csv")
    parser.add_argument("--configs", required=True, nargs="+", help="One or more config.yaml paths")
    parser.add_argument("--bags", default="bags", help="Directory containing .mcap bag files")
    parser.add_argument("--output", default="eval_results.json", help="Output JSON path")
    args = parser.parse_args()

    results = run_evaluation(args.csv, args.configs, args.bags)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()