#!/usr/bin/env python3
# pip install flask flask-socketio eventlet opencv-python
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import base64
import threading
import os

from std_msgs.msg import String as ROSString

from flask import Flask, send_from_directory
from flask_socketio import SocketIO
from anomaly_msg.msg import AnomalyMsg
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

# Setup paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

class AADBridge(Node):
    def __init__(self):
        super().__init__('aad_dashboard_bridge')

        # BEST EFFORT (camera, fast streams)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # RELIABLE (logs, structured data)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_best_effort = self.create_subscription(
            AnomalyMsg,
            '/ai_anomaly_logging',
            self.anomaly_callback,
            best_effort_qos
        )

        self.sub_reliable = self.create_subscription(
            AnomalyMsg,
            '/ai_anomaly_logging',
            self.anomaly_callback,
            reliable_qos
        )
        self.create_subscription(ROSString, '/aad/decisions', self.decision_callback, 10)
        self.get_logger().info(f"Dashboard serving from: {SCRIPT_DIR}")

    def decode_raw_image(self, img_msg):
        try:
            encoding = img_msg.encoding.lower()

            if encoding in ['rgb8', 'bgr8']:
                channels = 3
                dtype = np.uint8
            elif encoding in ['rgba8', 'bgra8']:
                channels = 4
                dtype = np.uint8
            elif encoding == 'mono8':
                channels = 1
                dtype = np.uint8
            else:
                raise ValueError(f"Unsupported encoding: {img_msg.encoding}")

            img_np = np.frombuffer(img_msg.data, dtype=dtype)

            bpp = channels * np.dtype(dtype).itemsize
            row_width = img_msg.step // bpp

            img = img_np.reshape((img_msg.height, row_width, channels))
            img = img[:, :img_msg.width, :]

            if encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

            img = cv2.resize(img, (320, 180))

            success, buffer = cv2.imencode('.jpg', img)
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

    def decision_callback(self, msg: ROSString):
        payload = {
            "decision": msg.data,
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
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)