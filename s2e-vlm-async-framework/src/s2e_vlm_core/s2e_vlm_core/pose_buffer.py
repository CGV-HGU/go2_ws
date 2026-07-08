from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    pose: Pose2D
    frame_id: str = "odom"
    child_frame_id: str = "base_link"


@dataclass(frozen=True)
class PoseLookupResult:
    found: bool
    pose: Pose2D | None
    sample: PoseSample | None
    age: float | None
    reason: str


class PoseBuffer:
    def __init__(self, max_samples: int = 256) -> None:
        if max_samples <= 0:
            raise ValueError("max_samples must be positive")
        self._max_samples = max_samples
        self._samples: list[PoseSample] = []

    def add(self, sample: PoseSample) -> None:
        stamps = self.stamps()
        index = bisect_right(stamps, sample.stamp)
        self._samples.insert(index, sample)
        if len(self._samples) > self._max_samples:
            del self._samples[: len(self._samples) - self._max_samples]

    def stamps(self) -> list[float]:
        return [sample.stamp for sample in self._samples]

    def latest(self) -> PoseSample | None:
        return self._samples[-1] if self._samples else None

    def lookup_latest_before(self, target_stamp: float, max_age: float) -> PoseLookupResult:
        if not self._samples:
            return PoseLookupResult(False, None, None, None, "EMPTY")
        index = bisect_right(self.stamps(), target_stamp) - 1
        if index < 0:
            return PoseLookupResult(False, None, None, None, "NO_PRIOR_POSE")
        sample = self._samples[index]
        age = target_stamp - sample.stamp
        if age > max_age:
            return PoseLookupResult(False, None, sample, age, "STALE")
        return PoseLookupResult(True, sample.pose, sample, age, "OK")
