"""
Layer 2: Symbolic Task Planner  (uses explicit house topology + Dijkstra)

The planner uses the explicit route_doors defined in house_config.json,
which correctly specifies which doors lead to which rooms. This respects
the actual house geometry without trying to infer connectivity.

The topological graph is built for future use (e.g., replanning around
blocked areas), but we rely on the explicit config for correctness.

Fine-grained pathfinding (wall avoidance) happens in the executor via
the grid planner, which respects occupancy maps.
"""

from typing import Any, Dict, List, Optional, Tuple
from skills.nav.hierarchical_planner import TopologicalGraph

LOG_PREFIX = "[PLANNER]"
Action = Tuple[str, ...]

# ---------------------------------------------------------------------------
# Interaction precondition table.
#
# Maps each interaction action type -> list of navigation steps that MUST
# precede it in the plan.  Adding a new interaction type here is all that
# is required to make the planner respect its spatial prerequisites.
# ---------------------------------------------------------------------------
_INTERACTION_PRECONDITIONS: Dict[str, List[str]] = {
    "open_door": ["navigate_door_approach", "verify_safe_approach"],
    "close_door": ["navigate_door_approach", "verify_safe_approach"],
    # Future actions follow the same pattern:
    # "press_button": ["navigate_to_object", "verify_arrival"],
    # "pick_object":  ["navigate_to_object", "verify_arrival"],
}


class TaskPlanner:

    def __init__(self, house_config):
        self._house = house_config
        self._safety_margin = 0.30
        self._interaction_range = 1.00
        
        self._graph = TopologicalGraph()
        self._build_navigation_graph()

    def _build_navigation_graph(self):
        """Construct the topological graph from house configuration.

        Nodes: all targets (rooms, furniture) + all door approach positions
        Edges: target <-> its route_doors; all doors mutually connected
        """
        def get_position(item_id: str, kind: str) -> Optional[Tuple[float, float, float]]:
            if kind == "target":
                return self._house.get_target_translation(item_id, use_cache=False)
            elif kind == "door":
                return self._house.get_door_approach_position(item_id)
            return None
        
        self._graph.build_from_house_config(self._house, get_position)
        print(f"{LOG_PREFIX} Navigation graph built — ready for Dijkstra planning")

    def set_safety_margin(self, margin: float):
        self._safety_margin = max(margin, 0.30)
        print(f"{LOG_PREFIX} Safety margin updated: {self._safety_margin:.2f}m")

    def reset_safety_margin(self):
        self._safety_margin = 0.30

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def plan(self, intent: Dict[str, Any], current_xy: Optional[Tuple[float, float]] = None) -> List[Action]:
        kind = intent.get("intent")
        if kind == "navigate":
            return self._plan_navigate(intent["targets"][0], current_xy=current_xy)
        if kind == "multi_step":
            actions: List[Action] = []
            # Track the predicted end pose across legs so each navigate can
            # plan its own room exit (a chained leg starts where the previous
            # one ended — the target room centre).
            pose = current_xy
            for step in intent.get("steps", []):
                if step["action"] == "navigate":
                    actions.extend(self._plan_navigate(step["target"], current_xy=pose))
                    tpos = self._house.get_target_translation(step["target"], use_cache=False)
                    pose = (tpos[0], tpos[1]) if tpos else None
            return actions
        if kind == "door_operation":
            # Even standalone door operations need spatial preconditions
            return self._plan_door_operation(intent["action"], intent["door"])
        raise ValueError(f"Unknown intent kind: '{kind}'")

    def validate(self, plan: List[Action]) -> List[str]:
        errors = []
        known_targets = set(self._house.list_targets())
        known_doors = set(self._house.list_doors())
        for action in plan:
            atype = action[0]
            if atype == "navigate" and action[1] not in known_targets:
                errors.append(f"Unknown target: '{action[1]}'")
            elif atype in ("open_door", "close_door") and action[1] not in known_doors:
                errors.append(f"Unknown door: '{action[1]}'")
        return errors

    # ------------------------------------------------------------------ #
    #  Core planning: spatial preconditions enforced here, at plan time   #
    # ------------------------------------------------------------------ #

    def _plan_navigate(self, target_id: str, current_xy: Optional[Tuple[float, float]] = None) -> List[Action]:
        """
        Decompose navigation into segments through door checkpoints.

        Uses the explicit route_doors from house_config.json, which correctly
        specifies house topology. This respects the actual geometry without
        trying to infer connectivity through walls.

        For each door on the route:
            1. Navigate to door approach position
            2. Verify safety preconditions
            3. Open door
            4. Continue to next segment

        Fine-grained pathfinding (obstacle avoidance, wall respect) happens
        in the executor via the grid planner.

        Example: go to kitchen
            route_doors = ["door_hall_kitchen"]
            Plan:
              1. navigate_door_approach(door_hall_kitchen)
              2. verify_safe_approach(door_hall_kitchen)
              3. open_door(door_hall_kitchen)
              4. navigate(kitchen)
        """
        known = self._house.list_targets()
        if target_id not in known:
            raise ValueError(f"Target '{target_id}' not found. Known: {known}")

        print(f"\n{LOG_PREFIX} ═══════════════════════════════════════════════════════")
        print(f"{LOG_PREFIX} PLAN: navigate to '{target_id}'")
        print(f"{LOG_PREFIX} ═══════════════════════════════════════════════════════")

        actions: List[Action] = []
        
        # Get explicit route_doors from config (correct topology)
        route_doors = self._house.get_route_doors_for_target(target_id)
        print(f"{LOG_PREFIX} [PLAN] Target '{target_id}' requires {len(route_doors)} doors: {route_doors}")

        # If NAO is currently INSIDE a room, leave it first with a controlled,
        # centred crossing of that room's door — the exact inverse of entering.
        # Without this, exiting the current room is only a side-effect of
        # approaching the destination door, with no centering/alignment for the
        # door being left, so NAO clips the frame. Same pipeline as entry; no
        # per-room special cases.
        if current_xy is not None:
            cur_door = self._current_room_door(current_xy)
            if cur_door is not None and cur_door not in route_doors:
                print(f"{LOG_PREFIX} [PLAN] NAO is inside the room of '{cur_door}' "
                      f"-> prepending controlled exit (inverse of entry)")
                actions.append(("navigate_door_approach", cur_door))
                actions.append(("verify_safe_approach", cur_door,
                                self._safety_margin, self._interaction_range))
                actions.append(("open_door", cur_door))
                actions.append(("cross_doorway", cur_door, target_id))

        if not route_doors:
            # Direct navigation — no doors on route
            print(f"{LOG_PREFIX} [PLAN] No doors required - direct navigation")
            actions.append(("navigate", target_id))
            print(f"{LOG_PREFIX} [PLAN] Plan: 1 step (direct navigate)")
            return actions

        # Multi-segment route: decompose through each door checkpoint
        print(f"{LOG_PREFIX} [PLAN] Route: {' -> '.join(route_doors)}")
        print(f"{LOG_PREFIX} [PLAN] Generating approach steps for each door...")
        
        
        for i, door_id in enumerate(route_doors, 1):
            door = self._house.find_door_by_id(door_id)
            if door is None:
                print(f"{LOG_PREFIX} [PLAN] ✗ ERROR: Door '{door_id}' not in config — SKIPPING")
                continue

            # Get and log approach position
            approach_pos = self._house.get_door_approach_position(door_id)
            if approach_pos:
                print(f"{LOG_PREFIX} [PLAN]   Door {i}: {door_id}")
                print(f"{LOG_PREFIX} [PLAN]     └─ approach position: ({approach_pos[0]:.2f}, {approach_pos[1]:.2f})")
            else:
                print(f"{LOG_PREFIX} [PLAN]   ✗ Door {i}: {door_id} - NO APPROACH POSITION")

            # Segment 1: Navigate to safe approach point for this door
            # The executor will use grid planner to respect walls
            actions.append(("navigate_door_approach", door_id))

            # Segment 2: Verify spatial preconditions before interacting
            actions.append(("verify_safe_approach", door_id,
                            self._safety_margin, self._interaction_range))

            # Segment 3: Open door only after verification passes
            actions.append(("open_door", door_id))

            # Segment 4: Cross the doorway in a straight line into the room.
            # Without this, the final navigate runs Dijkstra toward the room
            # centre and drifts laterally through the door frame.
            actions.append(("cross_doorway", door_id, target_id))

        # Final segment: enter destination room
        # Again, executor will use grid planner for wall-aware pathfinding
        actions.append(("navigate", target_id))
        print(f"{LOG_PREFIX} [PLAN] ✓ Plan generated: {len(actions)} steps ({len(route_doors)} doors)")
        print(f"{LOG_PREFIX} [PLAN] Steps: {[a[0] for a in actions]}")
        return actions

    def _plan_navigate_segment(self, target_id: str) -> List[Action]:
        return [("navigate", target_id)]

    def _current_room_door(self, current_xy: Tuple[float, float]) -> Optional[str]:
        """
        Return the door of the room NAO is currently inside, or None if NAO is
        in the hall corridor (no exit needed).

        The hall is the band between the two dividing walls. The walls coincide
        with the door frame Y lines, so the band is derived from the door
        geometry itself — no hardcoded coordinates, no per-room logic. When NAO
        is beyond a wall it is inside a room; the room's door is the one on that
        wall nearest in X.
        """
        frames = []  # (door_id, cx, cy)
        for door_id in self._house.list_doors():
            ap = self._house.get_door_approach_position(door_id)
            if ap is not None:
                frames.append((door_id, ap[0], ap[1]))
        if not frames:
            return None

        x, y = current_xy
        ys = [cy for _, _, cy in frames]
        north_y, south_y = max(ys), min(ys)

        # Inside the corridor between the dividing walls -> already in the hall.
        if south_y <= y <= north_y:
            return None

        # Beyond a wall -> inside a room. Pick the door on that wall nearest in X.
        wall_y = north_y if y > north_y else south_y
        same_wall = [(d, cx) for d, cx, cy in frames if abs(cy - wall_y) < 1e-6]
        if not same_wall:
            return None
        best = min(same_wall, key=lambda t: abs(t[1] - x))
        return best[0]

    def _plan_door_operation(self, operation: str, door_label: str) -> List[Action]:
        """
        Plan a standalone open/close door command.
        Looks up the door by label and inserts spatial preconditions.
        """
        door = self._find_door_by_label(door_label)
        if door is None:
            raise ValueError(f"Door '{door_label}' not found.")
        door_id = door.get("id", door_label)
        action_type = "open_door" if operation == "open" else "close_door"
        actions = list(self._precondition_steps(action_type, door_id))
        actions.append((action_type, door_id))
        return actions

    # ------------------------------------------------------------------ #
    #  Precondition expansion                                              #
    # ------------------------------------------------------------------ #

    def _precondition_steps(
        self, interaction_type: str, target_id: str
    ) -> List[Action]:
        """
        Return the navigation/verification steps required before
        `interaction_type` on `target_id`, derived from the precondition table.

        This is the single authoritative place where spatial ordering is defined.
        Adding a new interaction type to _INTERACTION_PRECONDITIONS is sufficient
        to make the planner respect its prerequisites everywhere.
        """
        required = _INTERACTION_PRECONDITIONS.get(interaction_type, [])
        steps: List[Action] = []
        for step_type in required:
            steps.append((step_type, target_id))
        return steps

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _find_door_by_label(self, label: str):
        for door in self._house.get_all_doors():
            if door.get("label", "").lower() == label.lower():
                return door
        return None
