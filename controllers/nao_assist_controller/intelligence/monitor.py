"""
Layer 4: Execution Monitor.

Detects when navigation is stuck, diverging, or failing,
and raises typed signals for the RobotBrain to act on.
"""

from enum import Enum, auto
from typing import List, Optional

LOG_PREFIX = "[MONITOR]"


class MonitorSignal(Enum):
    OK = auto()
    NO_PROGRESS = auto()   # distance not closing for N cycles
    DIVERGING = auto()     # distance actively increasing
    REPEATED_FALL = auto() # too many falls in one attempt
    MAX_RETRIES = auto()   # retry budget exhausted


_SIGNAL_MESSAGES = {
    MonitorSignal.NO_PROGRESS: "I am not making progress. I will replan my route.",
    MonitorSignal.DIVERGING: "The direct path seems blocked. I will adjust my approach.",
    MonitorSignal.REPEATED_FALL: "I have fallen too many times. I need to recover first.",
    MonitorSignal.MAX_RETRIES: "I cannot reach the target after multiple attempts.",
}


class ExecutionMonitor:
    """
    Tracks a navigation attempt and returns MonitorSignals.

    Usage:
        monitor.begin_attempt()
        signal = monitor.update(distance_to_target, fell=False)
        if signal != MonitorSignal.OK:
            ...
    """

    NO_PROGRESS_CYCLES = 6    # consecutive stale cycles before NO_PROGRESS
    DIVERGE_WINDOW = 5        # observation window for divergence check
    DIVERGE_THRESHOLD = 0.4   # metres increase over window
    FALL_LIMIT = 3            # falls per attempt before REPEATED_FALL

    def __init__(self, max_retries: int = 3):
        self._max_retries = max_retries
        self._retry_count = 0
        self._reset_attempt()

    # Public interface

    def begin_attempt(self):
        """Call before each navigation attempt."""
        self._reset_attempt()

    def update(self, distance: float, fell: bool = False) -> MonitorSignal:
        """
        Ingest one observation.
        Returns a signal: OK to continue, or a problem that needs replanning.
        """
        if fell:
            self._fall_count += 1
            if self._fall_count >= self.FALL_LIMIT:
                return MonitorSignal.REPEATED_FALL

        self._distances.append(distance)

        # Progress check: is the robot closing the gap?
        if self._prev_distance is not None:
            if distance < self._prev_distance - 0.05:
                self._stale_cycles = 0           # progress made
            else:
                self._stale_cycles += 1
                if self._stale_cycles >= self.NO_PROGRESS_CYCLES:
                    return MonitorSignal.NO_PROGRESS

        self._prev_distance = distance

        # Divergence check over a sliding window
        if len(self._distances) >= self.DIVERGE_WINDOW:
            window = self._distances[-self.DIVERGE_WINDOW:]
            if window[-1] > window[0] + self.DIVERGE_THRESHOLD:
                return MonitorSignal.DIVERGING

        return MonitorSignal.OK

    def record_retry(self) -> bool:
        """
        Register a retry. Returns True if still within budget, False if exhausted.
        """
        self._retry_count += 1
        self._reset_attempt()
        return self._retry_count <= self._max_retries

    @property
    def retry_count(self) -> int:
        return self._retry_count

    @property
    def max_retries(self) -> int:
        return self._max_retries

    def describe(self, signal: MonitorSignal) -> str:
        return _SIGNAL_MESSAGES.get(signal, "An unexpected navigation issue occurred.")

    # Internal

    def _reset_attempt(self):
        self._distances: List[float] = []
        self._prev_distance: Optional[float] = None
        self._stale_cycles = 0
        self._fall_count = 0
