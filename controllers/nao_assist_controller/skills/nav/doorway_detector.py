"""
DoorwayDetector — geometric doorway perception and crossing planner.

Uses known world geometry (door frame positions from the static world model)
to provide:
  - Door open/closed state via Webots Supervisor DEF node rotation
  - Proximity detection for sonar suppression during frame crossing
  - Optimised crossing waypoints that guarantee NAO actually passes through
  - Traversability check based on physical opening width vs NAO body width

Door frame data matches DEF node translations in My project.wbt.
All positions are in world XY coordinates (Z-up, metres).
"""

import math
from typing import Optional, Tuple, Dict

# Door frame geometry — must match _STATIC_WALLS / _DOOR_OPENINGS in
# DEF node translations in My project.wbt.
# Each entry: (frame_cx, frame_cy, wall_y, inner_half_gap_m)
# frame_cx/cy — world position of the door frame centre DEF node
# wall_y — Y coordinate of the dividing wall the door sits in
# inner_half_gap_m — half of the physical opening between post inner faces
_DOOR_FRAME_DATA: Dict[str, Tuple[float, float, float, float]] = {
    "door_hall_living":   (-2.36,  0.88,  0.88, 0.265),  # posts: x=-2.63, x=-2.09 gap 0.54m
    "door_hall_kitchen":  ( 1.84,  0.88,  0.88, 0.245),  # posts: x=1.57, x=2.11 gap 0.49m
    "door_hall_bedroom":  (-2.36, -1.52, -1.52, 0.265),  # same geometry as living
    "door_hall_bathroom": ( 1.84, -1.52, -1.52, 0.245),  # same geometry as kitchen
}

# NAO body half-width (shoulder width ~0.30m half = 0.15m)
NAO_BODY_HALF_WIDTH_M = 0.15

# Suppress sonar when NAO centre is within this radius of any door frame centre.
# Chosen so suppression begins ~0.15m before the frame wall.
FRAME_SUPPRESS_RADIUS_M = 0.65

# Through-door offset: crossing waypoint placed this far past the frame centre
# in the room direction. Must satisfy two constraints simultaneously:
# offset < sonar_suppress_dist (0.70m) sonar OFF at the frame
# offset > through_door_arrive_m NAO physically enters the room
THROUGH_DOOR_OFFSET_M = 0.55

# NAO considers itself "arrived" at the crossing waypoint when within this
# distance of it. Must be < THROUGH_DOOR_OFFSET_M so the robot ends up
# (THROUGH_DOOR_OFFSET_M - THROUGH_DOOR_ARRIVE_M) = 0.30m inside the room.
THROUGH_DOOR_ARRIVE_M = 0.25

# Minimum heading alignment before committing to door crossing (radians).
CROSSING_ALIGN_TOLERANCE_RAD = math.radians(15.0)

# Hinge DEF names (from house_config.json)
_DOOR_HINGE_DEFS: Dict[str, str] = {
    "door_hall_kitchen":  "DOOR_KITCHEN_HINGE",
    "door_hall_bedroom":  "DOOR_BEDROOM_HINGE",
    "door_hall_bathroom": "DOOR_BATHROOM_HINGE",
    "door_hall_living":   "DOOR_LIVING_HINGE",
}

# |hinge angle| must exceed this to be considered "open"
_DOOR_OPEN_THRESHOLD_RAD = 0.8


class DoorwayDetector:
    """
    Geometric doorway perception and crossing planner.

    Answers:
      1. Is NAO near a door frame?      → is_near_door_frame()
      2. Is the door physically open?   → is_door_open()
      3. Is the opening wide enough?    → is_passable()
      4. Where should NAO aim to cross? → get_through_waypoint()
      5. What heading is perpendicular? → compute_approach_heading()
    """

    def __init__(self, robot=None):
        self._robot = robot

    # Proximity queries (real-time sonar suppression)

    def nearest_door_frame(
        self, x: float, y: float
    ) -> Optional[Tuple[str, float, float, float]]:
        """Return (door_id, cx, cy, distance_m) for the nearest door frame."""
        best: Optional[Tuple[str, float, float, float]] = None
        best_dist = float("inf")
        for door_id, (cx, cy, _, _) in _DOOR_FRAME_DATA.items():
            d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            if d < best_dist:
                best_dist = d
                best = (door_id, cx, cy, d)
        return best

    def is_near_door_frame(
        self, x: float, y: float, radius_m: float = FRAME_SUPPRESS_RADIUS_M
    ) -> bool:
        """
        True when NAO is within radius_m of any door frame centre.
        Used as safety-net sonar suppression independent of target distance.
        """
        result = self.nearest_door_frame(x, y)
        return result is not None and result[3] <= radius_m

    def frame_centre(self, door_id: str) -> Optional[Tuple[float, float]]:
        """World (x, y) of a door frame centre, or None if unknown."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        return (entry[0], entry[1]) if entry is not None else None

    # Traversability

    def is_passable(self, door_id: str) -> bool:
        """True if the opening is wide enough for NAO (>=5 cm clearance each side)."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return True
        # inner_half_gap is already measured to post inner face — no further subtraction.
        # clearance each side = inner_half_gap - NAO_half_width
        clearance = entry[3] - NAO_BODY_HALF_WIDTH_M
        return clearance >= 0.05

    def opening_width_m(self, door_id: str) -> float:
        """Physical opening width between post inner faces (metres)."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return 0.50
        # inner_half_gap * 2 = full gap between inner faces
        # (was (entry[3] - 0.025) * 2 — double-subtracted post half-width)
        return entry[3] * 2.0

    # Door state via Supervisor

    def is_door_open(self, door_id: str) -> bool:
        """
        Query the Webots Supervisor hinge node to check door state.
        Degrades to True (assume open) when Supervisor unavailable.
        """
        if self._robot is None:
            return True
        hinge_def = _DOOR_HINGE_DEFS.get(door_id)
        if hinge_def is None:
            return True
        try:
            node = self._robot.getFromDef(hinge_def)
            if node is None:
                return True
            pos_field = node.getField("position")
            if pos_field is None:
                return True
            return abs(pos_field.getSFFloat()) >= _DOOR_OPEN_THRESHOLD_RAD
        except Exception:
            return True

    # Crossing waypoint

    def get_through_waypoint(
        self,
        approach_pos: Tuple[float, float, float],
        target_pos: Tuple[float, float, float],
        offset_m: float = THROUGH_DOOR_OFFSET_M,
    ) -> Tuple[float, float, float]:
        """
        Compute the through-door crossing waypoint.

        Placed offset_m past approach_pos in the direction of target_pos so:
          - At the frame, dist-to-waypoint = offset_m < 0.70m → sonar suppressed
          - With arrive_distance = THROUGH_DOOR_ARRIVE_M, NAO ends up
            (offset_m - THROUGH_DOOR_ARRIVE_M) = 0.30m inside the room

        Args:
            approach_pos: door frame centre world position
            target_pos:   final room target world position
            offset_m:     distance past frame to place waypoint
        """
        dx = target_pos[0] - approach_pos[0]
        dy = target_pos[1] - approach_pos[1]
        seg = math.sqrt(dx * dx + dy * dy)
        if seg < 0.1:
            return target_pos
        ux, uy = dx / seg, dy / seg
        return (
            approach_pos[0] + ux * offset_m,
            approach_pos[1] + uy * offset_m,
            0.0,
        )

    # Alignment helpers

    # Geometric helpers: door centre + posts derived from _DOOR_FRAME_DATA.
    # Use these instead of "frame DEF translation" so callers don't conflate
    # a particular DEF placement with the actual geometric centre.

    def door_center(self, door_id: str) -> Optional[Tuple[float, float]]:
        """Geometric midpoint of the two posts (world XY, metres)."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return None
        cx, cy, _wall_y, _half = entry
        return (cx, cy)

    def door_posts(self, door_id: str) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Return (west_post, east_post) world XY positions of the door posts."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return None
        cx, cy, _wall_y, half = entry
        return ((cx - half, cy), (cx + half, cy))

    def door_gap_width(self, door_id: str) -> Optional[float]:
        """Inner clear width of the opening (metres) = 2 * inner_half_gap."""
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return None
        return 2.0 * entry[3]

    def compute_approach_heading(self, door_id: str) -> Optional[float]:
        """
        Perpendicular approach heading for a door (radians, world frame).
        North-wall doors (y=0.88): π/2 (facing north into room).
        South-wall doors (y=-1.52): -π/2 (facing south into room).
        """
        entry = _DOOR_FRAME_DATA.get(door_id)
        if entry is None:
            return None
        return math.pi / 2.0 if entry[2] > 0 else -math.pi / 2.0

    def is_aligned_for_crossing(
        self,
        current_heading: float,
        door_id: str,
        tolerance_rad: float = CROSSING_ALIGN_TOLERANCE_RAD,
    ) -> bool:
        """True if NAO is within tolerance_rad of the door's perpendicular axis."""
        target = self.compute_approach_heading(door_id)
        if target is None:
            return True
        error = abs(math.atan2(
            math.sin(current_heading - target),
            math.cos(current_heading - target),
        ))
        return error <= tolerance_rad

    def __repr__(self) -> str:
        sup = "Supervisor" if self._robot is not None else "no Supervisor"
        return f"DoorwayDetector({len(_DOOR_FRAME_DATA)} doors, {sup})"
