"""One in-flight frame and one replaceable pending frame, never a FIFO."""

import threading


class LatestFrame:
    def __init__(self):
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self.received = 0
        self.overwritten = 0

    def put(self, value):
        with self._condition:
            if self._closed:
                return
            self.received += 1
            if self._pending is not None:
                self.overwritten += 1
            self._pending = value
            self._condition.notify()

    def take(self):
        with self._condition:
            self._condition.wait_for(lambda: self._closed or self._pending is not None)
            if self._closed:
                return None
            value, self._pending = self._pending, None
            return value

    def counts(self):
        with self._condition:
            return {"received": self.received, "overwritten": self.overwritten,
                    "pending": int(self._pending is not None)}

    def clear(self):
        """Drop a pending frame when the source identity changes."""
        with self._condition:
            self._pending = None

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()
