from collections import deque
import json
import threading

class JsonRingBuffer:
    def __init__(self, max_items=10_000):
        self.buffer = deque(maxlen=max_items)
        self.lock = threading.Lock()

    def add(self, data: dict):
        """Add a JSON object (dict)"""
        if not isinstance(data, dict):
            raise TypeError("Data must be a dict (JSON object)")
        with self.lock:
            self.buffer.append(data)

    def snapshot(self):
        """Get a copy of the current buffer"""
        with self.lock:
            return list(self.buffer)

    def clear(self):
        with self.lock:
            self.buffer.clear()
