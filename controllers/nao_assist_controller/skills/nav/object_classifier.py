"""
Object Classification Layer — context-aware navigation policy per object type.

Each DEF node / target ID is classified into one of three types:
  SOLID_OBSTACLE  → avoid always
  DOOR            → traverse (pass through), may need open_door skill
  INTERACTABLE    → approach when it IS the goal; avoid otherwise

Policy and arrive distance are derived from type + is_goal flag so the
navigation layer never needs to hardcode distances per object.
"""

from enum import Enum


class ObjectType(Enum):
    SOLID_OBSTACLE = "solid_obstacle"
    DOOR           = "door"
    INTERACTABLE   = "interactable"
    ROOM           = "room"           # floor node — navigable destination


class BehaviorPolicy(Enum):
    AVOID    = "avoid"
    TRAVERSE = "traverse"   # pass through (doors, open waypoints)
    APPROACH = "approach"   # stop at interaction distance


# Arrive distances (metres) per type when used as goal
_ARRIVE_DIST: dict = {
    ObjectType.ROOM:          0.70,
    ObjectType.DOOR:          1.00,   # stop before frame; enough to open + pass
    ObjectType.INTERACTABLE:  0.55,   # grabbing / interaction range
    ObjectType.SOLID_OBSTACLE: 0.70,  # fallback — shouldn't be a goal
}

# TARGET_CLASS strings expected by NavigationController.walk_to_target
# (mirrors constants in go_to_target.py)
_NAV_TARGET_CLASS: dict = {
    ObjectType.ROOM:           "room",
    ObjectType.DOOR:           "door",
    ObjectType.INTERACTABLE:   "furniture",
    ObjectType.SOLID_OBSTACLE: "transit",
}

_ROOM_DEF_PREFIXES  = ("FLOOR_",)
_DOOR_DEF_PREFIXES  = ("DOOR_FRAME_",)
_INTERACTABLE_DEFS  = frozenset({
    "SOFA_MAIN", "COFFEE_TABLE", "DINING_TABLE",
    "KITCHEN_COUNTER", "COUNTER", "TABLE_MAIN",
})


class ObjectClassifier:
    """
    Classifies Webots DEF nodes and house-config target IDs into ObjectType,
    then derives navigation policy and arrive distance.

    Pass the loaded HouseConfig so door DEF names are extracted dynamically.
    """

    def __init__(self, house_config):
        self._door_approach_defs: frozenset = frozenset(
            d.get("approach_def", "") for d in house_config.doors
        )
        self._target_def: dict = {
            t["id"]: t["def"] for t in house_config.targets
        }

    # ------------------------------------------------------------------
    # Classification
    # ------------------------------------------------------------------

    def classify_def(self, def_name: str) -> ObjectType:
        """Classify a raw Webots DEF node name."""
        if def_name in self._door_approach_defs:
            return ObjectType.DOOR
        if any(def_name.startswith(p) for p in _DOOR_DEF_PREFIXES):
            return ObjectType.DOOR
        if any(def_name.startswith(p) for p in _ROOM_DEF_PREFIXES):
            return ObjectType.ROOM
        if def_name in _INTERACTABLE_DEFS:
            return ObjectType.INTERACTABLE
        return ObjectType.SOLID_OBSTACLE

    def classify_target(self, target_id: str) -> ObjectType:
        """Classify a house-config target ID (e.g. 'sofa', 'kitchen')."""
        def_name = self._target_def.get(target_id, "")
        if def_name:
            return self.classify_def(def_name)
        return ObjectType.ROOM  # unknown IDs assumed to be room destinations

    def classify_node(self, node_type_str: str) -> ObjectType:
        """Classify from a TopologicalGraph node.type string."""
        return {
            "room":      ObjectType.ROOM,
            "door":      ObjectType.DOOR,
            "furniture": ObjectType.INTERACTABLE,
        }.get(node_type_str, ObjectType.SOLID_OBSTACLE)

    # ------------------------------------------------------------------
    # Policy derivation
    # ------------------------------------------------------------------

    def get_policy(self, obj_type: ObjectType, is_goal: bool) -> BehaviorPolicy:
        """
        DOOR           → TRAVERSE  (always pass through)
        ROOM           → APPROACH if goal, TRAVERSE if waypoint
        INTERACTABLE   → APPROACH if goal, AVOID if waypoint
        SOLID_OBSTACLE → AVOID     (always)
        """
        if obj_type == ObjectType.DOOR:
            return BehaviorPolicy.TRAVERSE
        if obj_type == ObjectType.ROOM:
            return BehaviorPolicy.APPROACH if is_goal else BehaviorPolicy.TRAVERSE
        if obj_type == ObjectType.INTERACTABLE:
            return BehaviorPolicy.APPROACH if is_goal else BehaviorPolicy.AVOID
        return BehaviorPolicy.AVOID

    def get_arrive_dist(self, obj_type: ObjectType) -> float:
        return _ARRIVE_DIST.get(obj_type, 0.60)

    def get_nav_target_class(self, obj_type: ObjectType) -> str:
        """Return TARGET_CLASS string for NavigationController.walk_to_target."""
        return _NAV_TARGET_CLASS.get(obj_type, "transit")
