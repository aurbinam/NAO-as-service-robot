"""
NAO Navigation Module v2 - Hierarchical Dijkstra Planning

Hierarchical layers:
  Layer 1: TopologicalGraph + Dijkstra (room-level planning)
  Layer 2: RoomPathPlanner (waypoint generation)
  Layer 3: WaypointController (waypoint execution + reactive avoidance)
  Layer 4: NavAgent (behavior tree orchestration)
"""

from .object_classifier import ObjectClassifier, ObjectType, BehaviorPolicy
from .behavior_tree import BTStatus, BTNode, Sequence, Selector, Condition, Action
from .waypoint_controller import WaypointController, StuckError
from .doorway_detector import DoorwayDetector
from .hierarchical_planner import HierarchicalPlanner, TopologicalGraph, RoomPathPlanner
from .nav_agent_v2 import NavAgent

__all__ = [
    'NavAgent',
    'HierarchicalPlanner',
    'TopologicalGraph',
    'RoomPathPlanner',
    'WaypointController',
    'ObjectClassifier',
    'ObjectType',
    'BehaviorPolicy',
    'DoorwayDetector',
    'BTStatus',
    'BTNode',
    'Sequence',
    'Selector',
    'Condition',
    'Action',
    'StuckError',
]
