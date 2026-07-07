#!/usr/bin/env python3
# pip install flask flask-socketio eventlet opencv-python
import rclpy
from rclpy.node import Node
import threading
import os
import json
import re

from std_msgs.msg import String as ROSString

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FILTERED_MESSAGE_TOPIC = "/aad/filtered_messages"
DECISION_TOPIC = "/aad/decisions"

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


ACTION_LABELS = {
    "stop_cart": "Stop cart",
    "alert_admin": "Alert admin",
    "none": "None",
}


def format_cart_decision(raw_decision: str) -> str:
    """
    Return the cart decision/action plus summary from a decision topic payload.
    """
    decision = str(raw_decision).strip()
    original_decision = decision
    if not decision:
        return "None"

    summary = ""

    try:
        payload = json.loads(decision)
        if isinstance(payload, dict):
            decision = str(
                payload.get("action")
                or payload.get("response")
                or payload.get("decision")
                or decision
            ).strip()
            summary = str(payload.get("summary") or payload.get("reason") or "").strip()
    except json.JSONDecodeError:
        action_match = re.search(r"\baction=(.*?)(?:\s+summary=|$)", original_decision)
        if action_match:
            decision = action_match.group(1).strip()
        summary_match = re.search(r"\bsummary=(.*)$", original_decision)
        if summary_match:
            summary = summary_match.group(1).strip()

    readable_decision = ACTION_LABELS.get(decision, decision.replace("_", " ").capitalize())
    if summary:
        return f"{readable_decision}: {summary}"
    return readable_decision


def parse_formatted_field(formatted_message: str, field_name: str) -> str:
    match = re.search(
        rf"\b{re.escape(field_name)}=(.*?)(?=\s+\w+=|$)",
        formatted_message,
    )
    return match.group(1).strip() if match else ""


def parse_decision_field(decision: str, field_name: str) -> str:
    try:
        payload = json.loads(str(decision).strip())
        if isinstance(payload, dict):
            return str(payload.get(field_name) or "").strip()
    except json.JSONDecodeError:
        pass

    return parse_formatted_field(decision, field_name)


def severity_to_level(severity: str) -> int:
    normalized = severity.strip().lower()
    if normalized == "high":
        return 2
    if normalized == "medium":
        return 1
    return 0


def decision_has_anomaly(decision: str) -> bool:
    """
    Return True when a decision payload says the current message batch is anomalous.
    """
    normalized = str(decision).strip()
    if not normalized:
        return False

    try:
        payload = json.loads(normalized)
        if isinstance(payload, dict):
            return bool(payload.get("anomaly", False))
    except json.JSONDecodeError:
        pass

    anomaly_match = re.search(r"\banomaly=(true|false)\b", normalized, re.IGNORECASE)
    return bool(anomaly_match and anomaly_match.group(1).lower() == "true")


class AADBridge(Node):
    def __init__(self):
        super().__init__('aad_dashboard_bridge')
        self.filtered_message_pub = self.create_publisher(
            ROSString,
            FILTERED_MESSAGE_TOPIC,
            10,
        )

        self.create_subscription(
            ROSString,
            FILTERED_MESSAGE_TOPIC,
            self.filtered_message_callback,
            10,
        )
        self.create_subscription(ROSString, DECISION_TOPIC, self.decision_callback, 10)
        self.get_logger().info(f"Dashboard serving from: {SCRIPT_DIR}")
        self.get_logger().info(f"Listening for decisions on {DECISION_TOPIC}")
        self.get_logger().info(f"Publishing dashboard-filtered messages on {FILTERED_MESSAGE_TOPIC}")

    def filtered_message_callback(self, msg: ROSString):
        severity = parse_decision_field(msg.data, "severity")
        decision = format_cart_decision(msg.data)
        payload = {
            "node": "AAD Decision",
            "importance": severity_to_level(severity),
            "text": decision,
            "timestamp": self.get_clock().now().nanoseconds * 1e-9,
            "image": None,
        }
        socketio.emit("anomaly_event", payload)
        socketio.emit(
            "decision_event",
            {
                "decision": decision,
                "timestamp": payload["timestamp"],
            },
        )

    def decision_callback(self, msg: ROSString):
        if decision_has_anomaly(msg.data):
            filtered_msg = ROSString()
            filtered_msg.data = msg.data
            self.filtered_message_pub.publish(filtered_msg)

@app.route("/")
def index():
    # Explicitly serving from the script's directory
    return send_from_directory(SCRIPT_DIR, "aad_dashboard.html")

def ros_thread_fn():
    rclpy.init()
    node = AADBridge()
    rclpy.spin(node)
    rclpy.shutdown()


def main():
    threading.Thread(target=ros_thread_fn, daemon=True).start()
    # Host 0.0.0.0 makes it accessible on your network
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
