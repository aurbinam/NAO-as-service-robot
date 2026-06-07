"""
Waypoint Controller — follows Dijkstra-planned waypoints.

2-state FSM: robot is ALWAYS either TURNING or MOVING_FORWARD.
No dead zone — if not actively turning, the robot walks.

┌─────────┐  error > LARGE_TURN_DEG     ┌──────────────────┐
│ TURNING │ ◀──────────────────────────  │ MOVING_FORWARD   │
│         │  (stop, issue turn cmd,      │ fwd + curvature  │
│         │   enter cooldown)            │ correction       │
└─────────┘                             └──────────────────┘
     │  cooldown expired                        ▲
     └──────────────────────────────────────────┘

MOVING_FORWARD: pure forward only — motion(BASE_FWD_CMD * speed_f, 0, 0)
  No curvature: motion_file backend plays full TurnLeft40/TurnRight40 for any
  |theta| > 0.05, ignoring the x component. Sending curvature oscillates robot.

TURNING fires only when error > LARGE_TURN_DEG (25°).
After a turn the robot always returns to MOVING_FORWARD.

Velocity scaling (sonar-based):
  speed_factor = clamp(sonar / 0.80, 0.30, 1.0)

Door mode: sonar suppressed near known door centres.

StuckError raised if no progress for 3.5 s so NavAgent can replan.
"""

import math
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

from .object_classifier import ObjectClassifier

LOG = "[WP_CTRL]"

# FSM states
class _State(Enum):
    MOVING_FORWARD = auto()  # walking with curvature heading correction
    TURNING        = auto()  # post-turn cooldown (forward only, no new turns)


# Heading thresholds
LARGE_TURN_DEG      = 25.0   # enter ALIGNING / discrete turn above this
SMALL_TURN_DEG      =  8.0   # ignore heading error below this (dead-band)
LARGE_TURN_RAD      = math.radians(LARGE_TURN_DEG)
SMALL_TURN_RAD      = math.radians(SMALL_TURN_DEG)

# Control gains
ROT_KP           = 1.5     # P-gain: heading error -> rotation command (ALIGNING)
ROT_MAX          = 0.50    # max rotation command
WALK_TURN_KP     = 0.6     # P-gain for curvature while walking (gentle)
WALK_TURN_MAX    = 0.22    # max curvature command while walking
BASE_FWD_CMD     = 0.50    # base forward velocity command

# Cooldown and timing
TURN_COOLDOWN_S  = 3.5     # must exceed TurnLeft40/TurnRight40 duration (2880ms)

# Cross-track
CROSS_TRACK_M        = 0.40  # threshold before cross-track correction fires (raised to avoid oscillatory corrections)
CROSS_TRACK_ALIGN_DEG = 15.0  # only correct when heading error < this

# Sonar / velocity scaling
SLOWDOWN_THRESHOLD_M = 0.80
MIN_SPEED_FACTOR     = 0.30

# Door zone / avoidance hysteresis
DOOR_ZONE_M   = 0.80
ENTER_AVOID_M = 0.35
EXIT_AVOID_M  = 0.55
ARC_TURN_CMD  = 0.28
ARC_FWD_CMD   = 0.12

# Waypoint arrival
WP_ARRIVE_M       = 0.60
DOOR_PASS_DIST_M  = 0.30
ARRIVE_DEFAULT_M  = 0.60

# Stuck detection
STUCK_TIMEOUT_S  = 3.5
STUCK_PROGRESS_M = 0.08


class StuckError(Exception):
    """Raised when stuck detection fires; NavAgent should replan."""
    pass


class WaypointController:
    """
    Stable 3-state FSM waypoint follower.

    Parameters
    ----------
    nav_controller  NavigationController from go_to_target.py
    grid            occupancy grid — updated from sonar readings
    classifier      ObjectClassifier — arrive distances
    door_centres    List of (x, y) known door opening positions
    """

    def __init__(self, nav_controller, grid, classifier: ObjectClassifier,
                 door_centres: Optional[List[Tuple[float, float]]] = None,
                 door_detector=None):
        self._nav   = nav_controller
        self._grid  = grid
        self._cls   = classifier
        self._doors = door_centres or []
        self._detector = door_detector

    # Public API

    def execute(self,
                waypoints:     List[Tuple[float, float]],
                goal_id:       str,
                arrive_dist_m: float = ARRIVE_DEFAULT_M,
                timeout_s:     float = 90.0) -> bool:
        """
        Follow waypoints to the goal.

        Returns True on arrival, False on timeout.
        Raises StuckError if no progress for STUCK_TIMEOUT_S seconds.
        """
        if not waypoints:
            return False

        # Delegate per-segment motion to _navigate_to_position (go_to_target.py).
        # That function was designed for the motion_file backend (Forwards50 /
        # TurnLeft40 / TurnRight40) and handles turn sequencing, stride completion,
        # stuck detection, and fall recovery correctly.
        # WaypointController's job: supply Dijkstra-planned waypoints and check
        # inter-segment timeout + fall state.
        from skills.go_to_target import _navigate_to_position

        # Debug: report navigation backend and door-zone policy
        try:
            walk_cmd = getattr(self._nav, '_walk_cmd', None)
            movement_enabled = getattr(self._nav, '_movement_enabled', None)
            obstacle = getattr(self._nav, '_obstacle_detector', None)
            obstacle_status = repr(obstacle) if obstacle is not None else 'None'
            print(f"{LOG} NAV BACKEND: walk_cmd={walk_cmd} movement_enabled={movement_enabled} obstacle={obstacle_status}")
        except Exception:
            pass
        is_door_goal = bool(goal_id) and goal_id.startswith("door_")
        goal_type = self._cls.classify_target(goal_id)
        goal_target_class = self._cls.get_nav_target_class(goal_type)
        transit_target_class = self._cls.get_nav_target_class(
            self._cls.classify_node("transit")
        )

        t_start = time.monotonic()

        for i, wp in enumerate(waypoints):
            # Fall check between segments
            if self._nav.is_fallen(verbose=False):
                return False

            # Overall timeout
            elapsed = time.monotonic() - t_start
            if elapsed > timeout_s:
                print(f"{LOG} Timeout after {elapsed:.1f}s")
                return False

            # Arrive distance: tight for intermediate waypoints, caller's for last
            is_last = (i == len(waypoints) - 1)
            wp_arrive = arrive_dist_m if is_last else WP_ARRIVE_M
            if is_door_goal:
                wp_arrive = min(wp_arrive, 0.25)

            wp_target_class = goal_target_class if is_last else transit_target_class

            # Skip waypoints the robot has already reached
            _pos = self._nav.get_current_position()
            try:
                _heading = self._nav.get_current_heading()
                _clear = None
                try:
                    _clear = self._nav._obstacle_detector.clearance_ahead()
                except Exception:
                    _clear = None
                print(f"{LOG} Pre-WP state: pos=({_pos[0]:.2f},{_pos[1]:.2f}) heading={_heading if _heading is not None else 'N/A'} clearance={_clear if _clear is not None else 'N/A'}")
            except Exception:
                pass
            if _pos is not None and _dist2d((_pos[0], _pos[1]), wp) <= wp_arrive:
                print(f"{LOG} WP {i + 1}/{len(waypoints)} already within arrive={wp_arrive:.2f}m — skip")
                continue

            print(f"{LOG} WP {i + 1}/{len(waypoints)} -> "
                  f"({wp[0]:.2f}, {wp[1]:.2f})  arrive={wp_arrive:.2f}m")

            ok = _navigate_to_position(
                self._nav,
                (wp[0], wp[1], 0.0),
                arrive_distance=wp_arrive,
                target_class=wp_target_class,
            )

            # Post-call diagnostics
            try:
                pos_after = self._nav.get_current_position()
                heading_after = self._nav.get_current_heading()
                clearance_after = None
                try:
                    clearance_after = self._nav._obstacle_detector.clearance_ahead()
                except Exception:
                    clearance_after = None
                print(f"{LOG} WP {i + 1} result: ok={ok} pos_after=({pos_after[0]:.2f},{pos_after[1]:.2f}) heading={heading_after if heading_after is not None else 'N/A'} clearance={clearance_after if clearance_after is not None else 'N/A'}")
            except Exception:
                print(f"{LOG} WP {i + 1} result: ok={ok} (post-state unavailable)")

            if not ok:
                print(f"{LOG} WP {i + 1} unreachable — raising StuckError")
                raise StuckError(f"WP {i + 1}/{len(waypoints)} unreachable")

        return True

    # Cross-track error

    def _cross_track(self, pos, seg_start: Tuple[float, float],
                     seg_end: Tuple[float, float]) -> float:
        """
        Signed cross-track distance from pos to the line seg_start→seg_end.
        Positive = robot is left of the line (needs right correction).
        """
        ax, ay = seg_start
        bx, by = seg_end
        px, py = pos[0], pos[1]
        abx, aby = bx - ax, by - ay
        length = math.sqrt(abx*abx + aby*aby)
        if length < 1e-6:
            return 0.0
        # Signed distance: positive = left of line direction
        return (abx * (py - ay) - aby * (px - ax)) / length

    # Door zone

    def _is_door_zone(self, wp: Tuple[float, float]) -> bool:
        # Prefer DoorwayDetector proximity check when available (uses
        # FRAME_SUPPRESS_RADIUS_M). Fall back to static door centre list.
        if self._detector is not None:
            return self._detector.is_near_door_frame(wp[0], wp[1])
        for dx, dy in self._doors:
            if math.sqrt((wp[0]-dx)**2 + (wp[1]-dy)**2) < DOOR_ZONE_M:
                return True
        return False

    # Sonar -> world obstacle position

    def _sonar_to_world(self, pos, dist_m: float
                        ) -> Tuple[Optional[float], Optional[float]]:
        h = self._nav.get_current_heading()
        if h is None:
            return None, None
        return pos[0] + dist_m * math.cos(h), pos[1] + dist_m * math.sin(h)

    # Stuck detection

    def _tick_stuck(self, pos, t_ref: float,
                    pos_ref) -> Tuple[float, tuple]:
        if pos_ref is not None and _dist2d(pos, pos_ref) >= STUCK_PROGRESS_M:
            return time.monotonic(), pos
        return t_ref, pos_ref

    def _check_stuck(self, pos, t_ref: float, pos_ref):
        if pos_ref is None:
            return
        if time.monotonic() - t_ref > STUCK_TIMEOUT_S:
            d = _dist2d(pos, pos_ref) if pos_ref else 0.0
            if d < STUCK_PROGRESS_M:
                self._nav.stop_walking()
                raise StuckError(f"No progress ({d:.3f}m)")

    # Webots step

    def _step(self):
        self._nav._step()


# Utilities

def _dist2d(a: tuple, b: tuple) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def _pos2xy(pos) -> Tuple[float, float]:
    if pos is None:
        return (0.0, 0.0)
    return (pos[0], pos[1])

def _norm_angle(a: float) -> float:
    """Normalize angle to [-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
