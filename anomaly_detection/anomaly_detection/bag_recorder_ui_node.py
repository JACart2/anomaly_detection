# anomaly_detection/bag_recorder_ui_node.py
import os
import signal
import subprocess
import threading
from datetime import datetime
from flask import Flask, jsonify, render_template_string, request
import re
import rclpy
from rclpy.node import Node

app = Flask(__name__)

# Shared state
recording_process = None
is_paused = False
current_bag_path = None
bag_dir = "/root/dev_ws/bags"

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>JACart Bag Recorder</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; }
        .card { background: #2a2a2a; padding: 2rem; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.5); text-align: center; width: 380px; }
        h2 { margin-top: 0; color: #dcdcdc; }
        .badge { display: inline-block; padding: 6px 14px; border-radius: 20px; font-weight: bold; margin-bottom: 1.5rem; }
        .idle { background: #444; color: #bbb; }
        .recording { background: #e53935; color: white; animation: pulse 1.5s infinite; }
        .paused { background: #fb8c00; color: white; }
        .btn-group { display: flex; flex-direction: column; gap: 10px; }
        button { padding: 12px; font-size: 16px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: 0.2s; }
        button:disabled { opacity: 0.4; cursor: not-allowed; }
        .btn-start { background: #43a047; color: white; }
        .btn-pause { background: #fb8c00; color: white; }
        .btn-stop { background: #e53935; color: white; }
        .info { margin-top: 15px; font-size: 13px; color: #aaa; word-break: break-all; }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.6; } 100% { opacity: 1; } }
    </style>
</head>
<body>
    <div class="card">
        <h2>ROS 2 Bag Control</h2>
        <div id="status-badge" class="badge idle">IDLE</div>
        <div class="btn-group">
            <button id="btn-start" class="btn-start" onclick="handleStart()">Start Recording</button>
            <button id="btn-pause" class="btn-pause" onclick="action('pause')" disabled>Pause</button>
            <button id="btn-stop" class="btn-stop" onclick="action('stop')" disabled>Stop Recording</button>
        </div>
        <div class="info" id="bag-info">Ready</div>
    </div>
    <script>
        async function updateStatus() {
            const res = await fetch('/status');
            const data = await res.json();
            const badge = document.getElementById('status-badge');
            const btnStart = document.getElementById('btn-start');
            const btnPause = document.getElementById('btn-pause');
            const btnStop = document.getElementById('btn-stop');
            const info = document.getElementById('bag-info');

            if (!data.recording) {
                badge.className = 'badge idle';
                badge.textContent = 'IDLE';
                btnStart.disabled = false;
                btnPause.disabled = true;
                btnStop.disabled = true;
                btnPause.textContent = 'Pause';
            } else if (data.paused) {
                badge.className = 'badge paused';
                badge.textContent = 'PAUSED';
                btnStart.disabled = true;
                btnPause.disabled = false;
                btnStop.disabled = false;
                btnPause.textContent = 'Resume';
            } else {
                badge.className = 'badge recording';
                badge.textContent = 'RECORDING';
                btnStart.disabled = true;
                btnPause.disabled = false;
                btnStop.disabled = false;
                btnPause.textContent = 'Pause';
            }
            if (data.bag_path) info.textContent = data.bag_path;
        }

        async function handleStart() {
            const userInput = prompt("Enter a bag prefix/name (optional):", "cart_run");
            if (userInput === null) {
                // User pressed cancel
                return;
            }

            await fetch('/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ prefix: userInput })
            });

            updateStatus();
        }

        async function action(type) {
            await fetch('/' + type, { method: 'POST' });
            updateStatus();
        }

        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route("/status", methods=["GET"])
def get_status():
    global recording_process, is_paused, current_bag_path
    return jsonify({
        "recording": recording_process is not None,
        "paused": is_paused,
        "bag_path": current_bag_path
    })

@app.route("/start", methods=["POST"])
def start_record():
    global recording_process, is_paused, current_bag_path
    if recording_process is not None:
        return jsonify({"error": "Already recording"}), 400

    data = request.get_json(silent=True) or {}
    user_prefix = data.get("prefix", "cart_run").strip()

    # Sanitize the input to alphanumeric, underscores, and dashes only
    clean_prefix = re.sub(r'[^a-zA-Z0-9_\-]', '_', user_prefix).strip('_')
    if not clean_prefix:
        clean_prefix = "cart_run"

    os.makedirs(bag_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"{clean_prefix}_{timestamp}"
    current_bag_path = os.path.join(bag_dir, bag_name)

    cmd = ["ros2", "bag", "record", "-a", "-o", current_bag_path]
    recording_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
    is_paused = False

    return jsonify({"status": "started", "path": current_bag_path})

@app.route("/pause", methods=["POST"])
def pause_record():
    global recording_process, is_paused
    if recording_process is None:
        return jsonify({"error": "Not recording"}), 400

    if not is_paused:
        os.killpg(os.getpgid(recording_process.pid), signal.SIGSTOP)
        is_paused = True
    else:
        os.killpg(os.getpgid(recording_process.pid), signal.SIGCONT)
        is_paused = False

    return jsonify({"status": "paused" if is_paused else "resumed"})

@app.route("/stop", methods=["POST"])
def stop_record():
    global recording_process, is_paused
    if recording_process is None:
        return jsonify({"error": "Not recording"}), 400

    try:
        pgid = os.getpgid(recording_process.pid)
        
        # 1. If paused, wake up the process group so it can process signals
        if is_paused:
            os.killpg(pgid, signal.SIGCONT)
        
        # 2. Send SIGINT to trigger clean bag closure
        os.killpg(pgid, signal.SIGINT)
        
        # 3. Wait with a timeout to avoid hanging indefinitely
        try:
            recording_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            # Force kill if ros2 bag doesn't close cleanly within 5s
            os.killpg(pgid, signal.SIGKILL)
            recording_process.wait()

    except ProcessLookupError:
        # Process already terminated
        pass
    finally:
        recording_process = None
        is_paused = False

    return jsonify({"status": "stopped"})

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_recorder_ui_node")
        self.get_logger().info("Bag Recorder UI Node Initialized on port 5000")

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()

    # Run Flask server in a daemon thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()