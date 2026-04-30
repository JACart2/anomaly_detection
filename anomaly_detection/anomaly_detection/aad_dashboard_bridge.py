#!/usr/bin/env python3
# pip install flask flask-socketio eventlet opencv-python
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import base64
import threading
import os

from flask import Flask, send_from_directory
from flask_socketio import SocketIO
from anomaly_msg.msg import AnomalyMsg

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class AADBridge(Node):
    def __init__(self):
        super().__init__('aad_dashboard_bridge')
        self.create_subscription(AnomalyMsg, '/ai_anomaly_logging', self.anomaly_callback, 10)
        self.create_subscription(AnomalyMsg, '/decision', self.decision_callback, 10)
        self.get_logger().info(f"Dashboard serving from: {SCRIPT_DIR}")

    def decode_raw_image(self, img_msg):
        try:
            img_np = np.frombuffer(img_msg.data, dtype=np.uint8)

            # Handle step safely (important for ROS images)
            channels = 3
            img_reshaped = img_np.reshape((img_msg.height, img_msg.step // channels, channels))
            img_reshaped = img_reshaped[:, :img_msg.width, :]

            # Convert color if needed
            if "rgb" in img_msg.encoding.lower():
                img_reshaped = cv2.cvtColor(img_reshaped, cv2.COLOR_RGB2BGR)

            # Downsample AFTER reshape
            img_reshaped = cv2.resize(img_reshaped, (320, 180))

            success, buffer = cv2.imencode('.jpg', img_reshaped)
            if success:
                return base64.b64encode(buffer).decode('utf-8')

        except Exception as e:
            self.get_logger().error(f"Decode error: {e}")

        return None

    def anomaly_callback(self, msg):

        payload = {
            "node": msg.node_name,
            "importance": int(msg.importance),
            "text": msg.msg,
            "timestamp": msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9),
            "image": self.decode_raw_image(msg.image) if msg.type == 1 else None
        }
        socketio.emit("anomaly_event", payload)

    def decision_callback(self, msg):
        payload = {
            "decision": msg.msg,
            "timestamp": self.get_clock().now().nanoseconds * 1e-9
        }
        socketio.emit("decision_event", payload)

@app.route("/")
def index():
    # Explicitly serving from the script's directory
    return send_from_directory(SCRIPT_DIR, "aad_dashboard.html")

def ros_thread_fn():
    rclpy.init()
    node = AADBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    threading.Thread(target=ros_thread_fn, daemon=True).start()
    # Host 0.0.0.0 makes it accessible on your network
    socketio.run(app, host="0.0.0.0", port=5000)