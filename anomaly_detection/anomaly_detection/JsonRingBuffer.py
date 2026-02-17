from collections import deque
import threading

class StringRingBuffer:
    def __init__(self, max_items=10_000):
        # deque with maxlen automatically handles the "ring" behavior
        # (removes oldest item when a new one is added)
        self.buffer = deque(maxlen=max_items)
        self.lock = threading.Lock()

    def add(self, data: str):
        """Add a string to the buffer."""
        if not isinstance(data, str):
            # If you still get dicts occasionally, you could use:
            # data = json.dumps(data) 
            # but for performance, it's better to enforce string input.
            raise TypeError(f"Data must be a string, got {type(data)}")
            
        with self.lock:
            self.buffer.append(data)

    def snapshot(self):
        """Get a copy of the current buffer as a list of strings."""
        with self.lock:
            return list(self.buffer)

    def clear(self):
        """Remove all items from the buffer."""
        with self.lock:
            self.buffer.clear()

    def __len__(self):
        """Allow checking size with len(buffer_instance)"""
        return len(self.buffer)
