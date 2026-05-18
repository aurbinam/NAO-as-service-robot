"""
Door-crossing finite state machine.

Wraps the existing Dijkstra/Waypoint navigation. Does NOT touch planner or
graph internals. It only sequences the existing primitives the executor
already exposes (rotate_toward_target + straight-line bursts) and emits
structured [DOOR] logs for every state transition.

States:
    IDLE     -> entry; no door work in progress
    DETECT   -> resolve approach/entry waypoints from house geometry
    ALIGN    -> rotate robot to face the entry waypoint (perpendicular to frame)
    CROSS    -> drive straight, in short bursts, through the door
    RESUME   -> hand control back to the room-level navigator
    FAILED   -> geometry missing or alignment impossible; navigator still runs

The FSM never modifies the navigation graph or waypoint list. Callers stay
responsible for invoking room-level navigation after RESUME.
"""

from typing import Callable, Optional, Tuple

LOG_PREFIX = "[DOOR]"

STATE_IDLE = "IDLE"
STATE_DETECT = "DETECT"
STATE_ALIGN = "ALIGN"
STATE_CROSS = "CROSS"
STATE_RESUME = "RESUME"
STATE_FAILED = "FAILED"


def _log(state: str, msg: str) -> None:
    print(f"{LOG_PREFIX}[{state}] {msg}")


class DoorCrossingFSM:
    """
    Stateful coordinator for a single door crossing.

    Construct once per crossing; call run() and inspect .final_state.
    """

    def __init__(
        self,
        door_id: str,
        approach_xy: Tuple[float, float],
        entry_xy: Tuple[float, float],
        rotate_fn: Callable[[Tuple[float, float, float]], bool],
        cross_fn: Callable[[float, float, float, int, int], bool],
        pose_fn: Callable[[], Optional[Tuple[float, float]]],
        arrive_dist: float = 0.40,
        burst_steps: int = 6,
        max_bursts: int = 24,
    ) -> None:
        self._door_id = door_id
        self._approach = approach_xy
        self._entry = entry_xy
        self._rotate_fn = rotate_fn
        self._cross_fn = cross_fn
        self._pose_fn = pose_fn
        self._arrive_dist = arrive_dist
        self._burst_steps = burst_steps
        self._max_bursts = max_bursts
        self._state = STATE_IDLE
        self.final_state = STATE_IDLE
        self.ok = False

    def _transition(self, new_state: str, reason: str = "") -> None:
        suffix = f" ({reason})" if reason else ""
        _log(self._state, f"-> {new_state}{suffix}")
        self._state = new_state

    def run(self) -> bool:
        """
        Execute the full state machine. Returns True if the cross completed
        within arrive tolerance, False otherwise. Always logs every step.
        """
        _log(STATE_IDLE, f"door={self._door_id} approach={self._approach} entry={self._entry}")

        if self._approach is None or self._entry is None:
            _log(STATE_IDLE, "missing geometry, abort")
            self.final_state = STATE_FAILED
            return False

        self._transition(STATE_DETECT, "geometry available")
        ex, ey = self._entry
        _log(STATE_DETECT, f"entry waypoint=({ex:.2f}, {ey:.2f}) arrive_dist={self._arrive_dist:.2f}")
        self._transition(STATE_ALIGN, "rotate toward entry")

        try:
            aligned = bool(self._rotate_fn((ex, ey, 0.0)))
        except Exception as exc:
            _log(STATE_ALIGN, f"rotate raised {exc!r}")
            aligned = False
        _log(STATE_ALIGN, f"aligned={aligned}")
        if not aligned:
            _log(STATE_ALIGN, "proceeding despite weak alignment (walk will correct)")

        self._transition(STATE_CROSS, "burst forward through frame")
        try:
            crossed = bool(self._cross_fn(
                ex, ey, self._arrive_dist, self._burst_steps, self._max_bursts,
            ))
        except Exception as exc:
            _log(STATE_CROSS, f"cross raised {exc!r}")
            crossed = False

        pose = None
        dist = -1.0
        try:
            pose = self._pose_fn()
            if pose is not None:
                dx = pose[0] - ex
                dy = pose[1] - ey
                dist = (dx * dx + dy * dy) ** 0.5
        except Exception:
            pass
        _log(STATE_CROSS, f"crossed={crossed} pose={pose} dist_to_entry={dist:.2f}")

        self.ok = crossed
        self._transition(STATE_RESUME if crossed else STATE_FAILED, "hand back to navigator")
        _log(STATE_RESUME if crossed else STATE_FAILED,
             "control returned to Dijkstra+Waypoint layer")
        self.final_state = STATE_RESUME if crossed else STATE_FAILED
        return crossed
