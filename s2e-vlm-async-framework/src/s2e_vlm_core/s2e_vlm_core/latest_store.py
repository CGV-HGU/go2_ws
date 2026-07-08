from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestStore(Generic[T]):
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._value: T | None = None

    def put(self, value: T) -> None:
        with self._condition:
            self._value = value
            self._condition.notify_all()

    def get(self) -> T | None:
        with self._condition:
            return self._value

    def pop(self) -> T | None:
        with self._condition:
            value = self._value
            self._value = None
            return value

    def wait_for(self, predicate: Callable[[T | None], bool], timeout: float) -> T | None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not predicate(self._value):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            return self._value
