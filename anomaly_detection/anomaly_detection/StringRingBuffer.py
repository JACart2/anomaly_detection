## TODO document this, also can this be replaced with just an imported queue?
class StringRingBuffer:
    def __init__(self, max_items: int):
        # Maximum number of elements the buffer can hold
        self.max_items = max_items

        # Fixed-size list used for storage (circular array)
        # Elements will be overwritten when the buffer is full
        self.buffer = [None] * max_items

        # Index pointing to the oldest element in the buffer
        # When items are evicted, this moves forward
        self.head = 0

        # Index where the next element will be written
        # Moves forward after every insert
        self.tail = 0

        # Current number of valid items in the buffer
        # Always <= max_items
        self.size = 0


    def add(self, item: str) -> None:
        """
        Adds a string to the buffer.

        If the buffer is full, the oldest element (at head)
        is overwritten and head moves forward.

        This maintains FIFO eviction while keeping O(1) insertion.
        """

        # Write the new item at the current tail position
        self.buffer[self.tail] = item

        if self.size == self.max_items:
            # Buffer is full:
            # advancing head discards the oldest element
            self.head = (self.head + 1) % self.max_items
        else:
            # If not full yet, increase the size
            self.size += 1

        # Move tail forward to the next write position
        # Modulo wraps the index to the start of the buffer
        self.tail = (self.tail + 1) % self.max_items


    def snapshot(self) -> list[str]:
        """
        Returns a list of items in FIFO order (oldest → newest).

        This method does not modify the buffer. It simply
        reads items starting from the head and wraps around
        if necessary.
        """

        result = []

        # Iterate through all stored elements
        for i in range(self.size):
            # Compute the correct index relative to head
            # Modulo handles wrap-around
            index = (self.head + i) % self.max_items
            result.append(self.buffer[index])

        return result


    def clear(self) -> None:
        """
        Completely resets the buffer.

        All stored values are removed and indices are
        returned to their initial state.
        """

        # Reinitialize storage
        self.buffer = [None] * self.max_items

        # Reset pointers and size
        self.head = 0
        self.tail = 0
        self.size = 0


    def __len__(self) -> int:
        """
        Allows use of len(buffer) to get number of stored items.
        """
        return self.size
