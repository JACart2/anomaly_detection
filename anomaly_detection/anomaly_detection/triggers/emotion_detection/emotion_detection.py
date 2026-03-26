"""Emotion Recognition of input video feed. Trigger script for AI Anomaly Detection (https://github.com/JACart2/anomaly_detection)

Author: John Rosario Cruz
Version: 3/9/2026
"""
## ROS2 packages
import os
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String as ROSString

## Multi-Threading
from threading import Thread
import threading
from queue import Queue

## Facial Expression Recognition (see README)
from fer.fer import FER

## Utility
import re
import time

class EmotionDetection(Node):
    """
    Description
    ------------
        Script for monitoring passenger emotions through a ROS2 topic subscription. 
        Also includes trigger methods for causing actions in the cart if needed.

    Attributes
    ----------
        bridge (obj): Used for converting ROS2 Image objects to an image matrix.

        subscription (obj): The subscription to the ROS2 topic for images. 
            Triggers listener_callback() on image receipt.

        frame_delay (float): The OPTIONAL time in seconds that 
            the main loop will wait to keep time with ZED FPS.

        primary (boolean): Determines which queue to use for processing. 
            This alternates between which queue/thread is given image data.

        emotion_data (list): Stores emotion data for frames.

        detector (obj): The FER detector. Used to detect emotion & faces in frames.

        frame_queue (queue): The first queue of frames coming from the ZED camera.

        stop_event (boolean): Forces the threads to begin termination.
            Forces listener_callback() to stop processing.

        thread_process (thread): The primary thread for processing. This method
            uses process_frames().

        _emotion_lock (threading.Lock): A lock for managing access to the emotion_data list.

        anomaly_message (str): The message that is sent to the backend if an anomaly is detected. This is monitored by the main loop of the backend and triggers a response if not empty.

    Methods:
    -------
        listener_callback()
            Triggered on image receipt from ROS2 topic. Populates thread queues and periodically
            calls monitor() based on self.emotion_data size.

        monitor()
            Analyze and clear all emotions in emotion_data to determine if openai call is required.

        process_frames()
            Primary thread for processing frames from frame queue from main.

        _set_anomaly_message()
            Set the anomaly message attr in a thread-safe way.
        
        consume_anomaly_message()
            Consume the anomaly message in a thread-safe way. This also clears the message after consumption.
    """

    def __init__(self):
        ## configure camera setup
        ## must match name of dir containing script
        super().__init__('emotion_detection')
        config = self._load_config()
        self.trigger_input_topic = config.get("trigger_input_topic", "/trigger_messages")
        self.bridge = CvBridge()

        # subscription
        self.subscription = self.create_subscription(
            Image,
            '/zed_rear/zed_node_1/rgb/color/rect/image', 
            self.listener_callback,
            10
        )

        # optional frame throttling
        fps = 15
        self.frame_delay = float(1 / fps)
        self.current_time = 0.0

        # sentiment labels
        self.emotion_data = []
        self.detector = FER(mtcnn=True)

        # threading init
        self.frame_queue = Queue(maxsize=30)
        self.stop_event = threading.Event()

        # starting threads
        self.thread_process = Thread(target=self.process_frames, daemon=True)
        self.thread_process.start()

        ## the error message that will be monitored and state management for message
        self._emotion_lock = threading.Lock()
        self.publisher = self.create_publisher(ROSString, self.trigger_input_topic, 10)

    def listener_callback(self, msg: object) -> None:
        """Callback method that handles the incoming images from the ROS2 topic.

        Args:
            msg (obj): Image object from ROS2 topic.
        """
        # force image throttle
        current_time = time.time()
        if current_time - self.current_time < self.frame_delay:
            return
        else:
            self.current_time = current_time

        if not self.stop_event.is_set():
            try:
                rgba_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                rgb_image = cv2.cvtColor(rgba_image, cv2.COLOR_BGRA2BGR)

                ## manual image flip until ROS2 topic does this
                #rgb_image = cv2.flip(rgb_image, 0)

                ## update queue in thread safe way
                if not self.frame_queue.full():
                    self.frame_queue.put(rgb_image)
                should_monitor = False
                with self._emotion_lock:
                    if len(self.emotion_data) >= 30:
                        should_monitor = True
                if should_monitor:
                    self.monitor()

            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def monitor(self) -> None:
        """Analyzes and clears all emotions in emotion_data to determine appropriate response.

        """
        with self._emotion_lock:
            if not self.emotion_data:
                return
            emotion_batch = list(self.emotion_data)
            self.emotion_data.clear()

        emotion_count = [re.match(r'(\w+):', item).group(1) for item in emotion_batch]

        ## find the top emotion
        top_emotion = "neutral"
        count = 0
        for emotion in set(emotion_count):
            if emotion_count.count(emotion) >= count:
                count = emotion_count.count(emotion)
                top_emotion = emotion

        print('')
        print(top_emotion, 'top_emotion')

        ## find average confidence of each frame of the top emotion
        confidence = []
        for emotion in emotion_batch:
            if top_emotion in emotion:
                match = re.search(r'(\d+)%$', emotion)
                if match:
                    confidence.append(int(match.group(1)))

        ## catch no faces being seen in the array
        if not confidence:
            print('no faces were seen in the frame?')
            return
        
        average_confidence = int(sum(confidence)/len(confidence))

        ## lowering this number will increase chatgpt calls
        if average_confidence >= 50:
            if top_emotion in ["fear", "sad", "surprise", "angry", "disgust"]:
                ## returning because something is bad
                alert = ROSString()
                alert.data = (f"emotion_detection trigger script claims passenger is experiencing {top_emotion} at a {average_confidence}% confidence")
                self.publish_anomaly_message(alert)

    def process_frames(self) -> None:
        """Primary thread for processing frames from frame queue from main.
        """
        while not self.stop_event.is_set():

            emotions = {}
            frame = self.frame_queue.get()
            response = self.detector.detect_emotions(frame)
            # Store the detected emotion for the current face in the emotions dictionary
            for i, passenger in enumerate(response):
                confidence = 0
                name = ""
                for emotion in passenger['emotions']:
                    if passenger['emotions'][emotion] > confidence:
                        confidence = passenger['emotions'][emotion]
                        name = emotion

                emotions[f"Passenger {i + 1}:"] = f"{name}: {int(confidence * 100)}%"

            if emotions:
                with self._emotion_lock:
                    self.emotion_data.append(emotions['Passenger 1:'])

    def publish_anomaly_message(self, alert: ROSString) -> None:
        self.publisher.publish(alert)
        return

    def _load_config(self) -> dict:
        env_path = os.getenv("AAD_CONFIG_PATH")
        if env_path and os.path.isfile(env_path):
            config_path = env_path
        else:
            config_path = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "..", "..", "config.yaml")
            )

        if not os.path.isfile(config_path):
            self.get_logger().warn(f"Config file not found at {config_path}. Using defaults.")
            return {}

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                self.get_logger().warn("Config file loaded but is not a YAML mapping. Using defaults.")
                return {}
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load config file {config_path}: {e}. Using defaults.")
            return {}

def main() -> None:
    """Main callpoint of the class.
    """
    rclpy.init()
    emotion_detection_node = EmotionDetection()

    try:
        rclpy.spin(emotion_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        ## ending threads, ending listener_callback() functionality
        emotion_detection_node.stop_event.set()
        emotion_detection_node.thread_process.join()
        emotion_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
