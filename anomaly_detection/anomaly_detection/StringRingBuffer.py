class StringRingBuffer:
    def __init__(self, max_items: int):
        self.max_items = max_items

        # underlying storage
        self.buffer = [None] * max_items

        # index of oldest item
        self.head = 0

        # index where next item will be written
        self.tail = 0

        # current number of items
        self.size = 0


    def add(self, item: str) -> None:
        """
        Add a string to the buffer.
        If full, evicts oldest item (FIFO).
        """

        # write new item
        self.buffer[self.tail] = item

        if self.size == self.max_items:
            # buffer full → evict oldest
            self.head = (self.head + 1) % self.max_items
        else:
            self.size += 1

        # advance tail
        self.tail = (self.tail + 1) % self.max_items


    def snapshot(self) -> list[str]:
        """
        Returns items in FIFO order (oldest → newest)
        Does NOT modify buffer.
        """

        result = []

        for i in range(self.size):
            index = (self.head + i) % self.max_items
            result.append(self.buffer[index])

        return result


    def clear(self) -> None:
        """
        Clears buffer completely.
        """

        self.buffer = [None] * self.max_items
        self.head = 0
        self.tail = 0
        self.size = 0


    def __len__(self) -> int:
        return self.size
