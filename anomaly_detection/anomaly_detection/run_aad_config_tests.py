"""Evaluation harness for the AAD (AI Anomaly Detection) node.
Iterates over a dataset CSV and a list of config files, plays each bag file,
collects alerts from /aad/alerts, and compares against ground truth labels.

Author: AAD Team Spring 26'
Version: 3/22/2026 (FIXED)
"""
import os
import signal
import csv
import json
import time
import subprocess
import threading
import argparse
import glob

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor  # FIX: was used but never imported
from std_msgs.msg import String, Bool
from dotenv import load_dotenv


ALERT_TOPIC = "/aad/decisions"
LLM_CALLED_TOPIC = "/aad/llm_called"
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

class AlertCollector(Node):
    """
    Description
    ------------
        Lightweight ROS 2 node that subscribes to /aad/decisions and collects
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
        self.llm_called = 0
        self.alerts = []
        self.create_subscription(String, ALERT_TOPIC, self._callback, 10)
        self.create_subscription(Bool, LLM_CALLED_TOPIC, self._llm_callback, 10)

    def _callback(self, msg: String) -> None:
        """
        Subscription callback for /aad/alerts.

        Args:
            msg (String): Incoming alert message from the AAD node.
        """
        self.get_logger().info(f"Alert received: {msg.data}")
        self.alerts.append(msg.data)

    def _llm_callback(self, msg: Bool) -> None:
        """
        Subscription callback for /aad/llm_called.

        Args:
            msg (Bool): Used to count the number of times the LLM has been called.
        """
        self.get_logger().info(f"LLM called: {msg.data}")
        self.llm_called += 1

    def reset(self):
        self.llm_called = 0
        self.alerts.clear()


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



# How long to wait for the AAD node process to be ready before playing a bag.
NODE_STARTUP_GRACE_SEC = 3.0

# How long to wait for the spin thread to exit after stop_event is set.
SPIN_JOIN_TIMEOUT_SEC = 30.0
# How long to wait for the AAD node process to exit after SIGTERM before the
# harness calls rclpy.shutdown().  The node runs its own finally block which
# calls executor.shutdown() and rclpy.shutdown() — we must let it finish or
# the two shutdown sequences will race and corrupt shared ROS daemon state.
NODE_EXIT_TIMEOUT_SEC = 20.0


def _stop_proc(proc: subprocess.Popen, timeout: float = 5.0) -> None:
    """
    Terminate a process group gracefully, escalating to SIGKILL on timeout.

    ros2 run spawns the actual node as a child process — sending SIGTERM only
    to the launcher (proc.terminate()) leaves the node running as an orphan.
    Sending SIGTERM to the entire process group (os.killpg) ensures both the
    launcher and its children receive the signal.

    start_new_session=True must be passed to Popen so the child gets its own
    process group; otherwise os.killpg would also kill this harness process.
    """
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return  # already dead

    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


def _run_one_bag(
    row: dict,
    bags_dir: str,
    abs_config: str,
    collector: "AlertCollector",
) -> dict | None:
    """
    Play a single bag file and collect alerts from an already-running AAD node.

    The harness maintains a single rclpy context, a single AlertCollector, and
    a single spin thread for the entire evaluation run (created in
    run_evaluation).  This function only clears the collector before each bag
    so that alerts from a previous run do not bleed into the next one.

    The AAD node process is managed entirely by run_evaluation's try/finally;
    the node's own rclpy context is fully separate — it is a different OS
    process with its own ROS context.  There is no shared rclpy state between
    the harness process and the node process, so no shutdown race is possible.

    Returns the result dict, or None if the bag file was not found.
    """
    bag_file    = row["Bag file"].strip()
    category    = row["Anomaly Category"].strip()
    description = row["Description"].strip()
    expected    = row["Anomalous"].strip()
    bag_path    = os.path.join(bags_dir, bag_file)

    if not os.path.exists(bag_path):
        print(f"  ! Skipping — bag not found: {bag_path}")
        return None

    # Clear any alerts left over from the previous bag before playing this one.
    collector.reset()

    print(f"  Playing {bag_file}...", end=" ", flush=True)

    bag_proc = subprocess.Popen(
        ["ros2", "bag", "play", bag_path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    bag_proc.wait()

    while collector.llm_called != len(collector.alerts):
        time.sleep(0.5)
        
    detected = len(collector.alerts) > 0
    correct  = detected == (expected == "Yes")
    status   = "✓" if correct else "✗"
    parsed   = [parse_alert(a) for a in collector.alerts]

    print(f"{status} | expected={expected:3s} | detected={detected} | {category} — {description}")

    if collector.alerts:
        for a in collector.alerts:
            print(f"      → {a}")

    return {
        "config":      abs_config,
        "bag_file":    bag_file,
        "category":    category,
        "description": description,
        "expected":    expected,
        "detected":    detected,
        "correct":     correct,
        "alerts":      parsed,
    }


def run_evaluation(csv_path: str, config_paths: list[str], bags_dir: str) -> list:

    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    all_results = []

    # The harness uses a single rclpy context, collector, and spin thread for
    # the entire run.  The node under test runs in a separate OS process with
    # its own rclpy context — there is no shared ROS state to race over.
    rclpy.init()
    collector = AlertCollector()
    executor  = SingleThreadedExecutor()
    executor.add_node(collector)

    stop_event = threading.Event()

    def spin():
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        for config_path in config_paths:
            abs_config = os.path.abspath(config_path)

            print(f"\n{'='*60}")
            print(f"Config: {abs_config}")
            print(f"{'='*60}")

            env = {**os.environ, "AAD_CONFIG_PATH": abs_config}

            for row in rows:
                bag_path = os.path.join(bags_dir, row["Bag file"].strip())
                if not os.path.exists(bag_path):
                    print(f"  ! Skipping — bag not found: {bag_path}")
                    continue

                # start_new_session=True gives the node its own process group
                # so _stop_proc can SIGTERM the whole group (launcher + node +
                # trigger children) without also killing this harness process.
                node_proc = subprocess.Popen(
                    ["ros2", "run", "anomaly_detection", "anomaly_detection_node"],
                    env=env,
                    start_new_session=True,
                    # stdout=subprocess.DEVNULL,
                    # stderr=subprocess.DEVNULL,
                )
                time.sleep(NODE_STARTUP_GRACE_SEC)

                try:
                    result = _run_one_bag(row, bags_dir, abs_config, collector)
                    if result is not None:
                        all_results.append(result)
                finally:
                    # Kill the whole node process group and wait for it to exit.
                    # Because the node runs in a separate OS process its own
                    # rclpy.shutdown() (in its main() finally) is completely
                    # independent of the harness rclpy context — no race.
                    _stop_proc(node_proc, timeout=NODE_EXIT_TIMEOUT_SEC)

    finally:
        # Tear down the single harness-side ROS context once, after all bags.
        stop_event.set()
        spin_thread.join(timeout=SPIN_JOIN_TIMEOUT_SEC)
        if spin_thread.is_alive():
            print("WARNING: spin thread did not exit cleanly", flush=True)
        executor.shutdown(timeout_sec=5)
        collector.destroy_node()
        rclpy.shutdown()

    # 🔹 Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")

    for config_path in config_paths:
        abs_config     = os.path.abspath(config_path)
        config_results = [r for r in all_results if r["config"] == abs_config]

        total = len(config_results)
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
    parser.add_argument("--csv",        default="src/anomaly_detection/system_evaluation/aad_dataset.csv",              help="Path to dataset_outline.csv")
    parser.add_argument("--configs",    default=None, nargs="*", help="One or more config.yaml paths (or leave empty to use all in configs/)")
    parser.add_argument("--config_dir", default="src/anomaly_detection/system_evaluation/configs",                      help="Directory to search for configs if --configs is not provided.")
    parser.add_argument("--bags",       default="src/anomaly_detection/system_evaluation/bags",                         help="Directory containing .mcap bag files")
    parser.add_argument("--output",     default="src/anomaly_detection/system_evaluation/eval_results.json",           help="Output JSON path")
    args = parser.parse_args()

    if args.configs:
        config_files = list(args.configs)
    else:
        search_path = os.path.join(args.config_dir, "*.yaml")
        config_files = sorted(glob.glob(search_path))
        if not config_files:
            print(f"Warning: No .yaml files found in {args.config_dir}")

    print(f"Running evaluation with {len(config_files)} configs:")
    for c in config_files:
        print(f"  - {c}")

    results = run_evaluation(args.csv, config_files, args.bags)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {args.output}")


if __name__ == "__main__":
    main()