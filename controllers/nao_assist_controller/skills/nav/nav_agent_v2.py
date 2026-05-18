"""
NAV AGENT V2 — Hierarchical navigation orchestrator for NAO robot.

================================================================================
FILE PURPOSE & ROLE IN NAVIGATION SYSTEM
================================================================================
This file provides the main interface between high-level task planning and low-level
motion control. It orchestrates the hierarchical planning pipeline:

  HIGH-LEVEL FLOW:
  ───────────────
  Voice Command: "Go to kitchen"
       ↓
  go_to_target.py:go_to_target()
       ↓
  NavAgent.navigate_to("kitchen")
       ↓
  Layer 1 Dijkstra: corridor → kitchen (find room-level path)
       ↓
  Layer 2 Waypoints: Generate doorway and center waypoints
       ↓
  Layer 3 Execution: Move robot along waypoints using WaypointController
       ↓
  ROBOT ARRIVES

This module's job is to:
  1. Load configuration (hierarchical_config.json)
  2. Initialize 3-layer planner (TopologicalGraph, RoomPathPlanner, HierarchicalPlanner)
  3. Provide clean API: navigate_to(target_name) → boolean success/failure
  4. Handle motion callbacks and state transitions
  5. Adapt old coordinate-based API (start_xy, goal_xy) to new room-based API
  
================================================================================
HOW THIS FILE RELATES TO OTHER MODULES
================================================================================
INPUTS:
  1. hierarchical_config.json
     - Loaded by: NavAgent.__init__()
     - Location: controllers/nao_assist_controller/house/hierarchical_config.json
     - Contains: rooms, hallways, entry_points, room_targets
  
  2. task_planner.py
     - Calls: NavAgent (via executor.py)
     - Calls it with: robot, house_config, nav_controller
  
  3. executor.py
     - Creates: NavAgent(robot, house_config, nav_controller)
     - Calls: agent.get_planner().plan(start_xy, goal_xy)
  
  4. go_to_target.py
     - High-level API wrapper calling NavAgent.navigate_to()
     - Handles voice commands, door opening, recovery logic

OUTPUTS:
  1. WaypointController (waypoint_controller.py)
     - Receives: waypoint list from HierarchicalPlanner.plan()
     - Executes: Moves robot along waypoints, reactive avoidance
  
  2. ObjectClassifier (object_classifier.py)
     - Used by: NavAgent for room-specific behaviors
     - Provides: Object detection policies, furniture localization
  
  3. BehaviorTree (behavior_tree.py)
     - Orchestrates: Planning → Execution → Recovery state machine

CALL HIERARCHY:
  ──────────────
  voice_listener.py (TCP listener)
       ↓ receives "go to kitchen"
  parse_command() [ai_command_parser.py]
       ↓ parses into Task(type="navigate", target="kitchen")
  RobotBrain.handle_command() [robot_brain.py]
       ↓
  executor._navigate_to_point(target_id) [executor.py]
       ↓
  nav_agent.get_planner().plan(start_xy, goal_xy) [THIS FILE]
       ↓ returns waypoints
  WaypointController.execute(waypoints) [waypoint_controller.py]
       ↓
  NAO MOVES

================================================================================
THE _PLANNERADAPTER CLASS — BRIDGING OLD & NEW APIs
================================================================================
PROBLEM: Old code expects plan(start_xy, goal_xy) to accept world coordinates.
         New hierarchical planner expects plan(start_xy, goal_room) with room names.

SOLUTION: _PlannerAdapter wraps HierarchicalPlanner and translates:
  1. Input: executor calls plan(start_xy=(1.5, 2.0), goal_xy=(1.99, 2.63))
  2. Adapter: Maps goal_xy to nearest room ("kitchen")
  3. Adapter: Calls hierarchical_planner.plan(start_xy, goal_room="kitchen")
  4. Adapter: Returns waypoints unchanged
  5. Output: executor receives waypoints as before

This allows executor.py to work unchanged without knowing about rooms!

_PlannerAdapter Methods:
  - _find_room_containing(xy): Finds which room contains coordinate
  - _find_nearest_room_by_entry(xy): If not in room, find nearest entry point
  - plan(start_xy, goal_xy): Main entry point (called by executor)

================================================================================
DATA STRUCTURES
================================================================================
hierarchical_config.json:
  {
    "rooms": {
      "corridor": {
        "center": [-0.36, -0.32],
        "entry_points": [[0.5, -0.32], [-1.5, -0.32], ...],
        "bounds": [-4.71, 3.99, -1.27, 0.63],
        "type": "hallway"
      },
      "kitchen": {
        "center": [1.99, 2.63],
        "entry_points": [[1.84, 1.40], [1.84, 0.40]],
        "bounds": [0.93, 3.05, 1.40, 3.85],
        "door_frame": "door_frame_kitchen"
      },
      ...
    },
    "hallways": [
      ["corridor", "kitchen", 1.5],
      ["corridor", "living_room", 1.5],
      ["corridor", "bedroom", 1.5],
      ["corridor", "bathroom", 1.5]
    ],
    "room_targets": {
      "kitchen": {"label": "Kitchen", "furniture": ["counter", "table"], "doors": ["door_hall_kitchen"]},
      ...
    }
  }

Execution Flow:
  1. load hierarchical_config.json
  2. HierarchicalPlanner.load_house_config(config) → initializes TopologicalGraph
  3. _PlannerAdapter(hierarchical_planner) → wraps planner for backward compatibility
  4. executor calls get_planner().plan(start_xy, goal_xy)
  5. _PlannerAdapter converts goal_xy to goal_room
  6. HierarchicalPlanner returns waypoints
  7. WaypointController moves robot

================================================================================
INITIALIZATION SEQUENCE
================================================================================
1. NavAgent.__init__(robot, house_config, nav_controller)
   - Loads hierarchical_config.json from disk
   - Creates HierarchicalPlanner and calls load_house_config()
   - Initializes ObjectClassifier (or MockObjectClassifier if no house_config)
   - Creates BehaviorTree for state machine

2. BehaviorTree construction
   - Root: Selector(TryToNavigate, HandleRecovery)
   - TryToNavigate: Sequence(PlanAction, ExecuteAction)
   - Handles replanning on StuckError

3. First navigate_to() call
   - Executes BehaviorTree (PLAN → EXECUTE → RECOVER loop)
   - Returns True on success, False on timeout/failure

================================================================================
CLASS HIERARCHY
================================================================================
NavAgent (Main orchestrator)
  ├─ HierarchicalPlanner (3-layer planning engine)
  │   ├─ TopologicalGraph (Layer 1: Room-level Dijkstra)
  │   └─ RoomPathPlanner (Layer 2: Waypoint generation)
  ├─ _PlannerAdapter (Translates old API to new)
  ├─ ObjectClassifier (Object detection & behavior policies)
  ├─ WaypointController (Layer 3: Waypoint execution)
  └─ BehaviorTree (State machine for PLAN/EXECUTE/RECOVER)

================================================================================
IMPORTANT NOTES FOR DEBUGGING
================================================================================
1. If planner fails:
   - Check hierarchical_config.json exists in controllers/nao_assist_controller/house/
   - Verify room IDs match: "kitchen", "corridor", "bedroom", "bathroom"
   - Check entry_points are not empty (doors must be defined)

2. If robot takes inefficient "big round" paths:
   - Check hallway costs in hierarchical_config.json (all 1.5 currently)
   - Verify room centers are centrally located (not on walls)
   - Check entry_point selection logic in _PlannerAdapter

3. If walls are hit:
   - Hallway costs don't prevent collisions (that's WaypointController's sonar job)
   - Check room bounds in config match actual Webots geometry
   - Increase sonar thresholds in WaypointController if too aggressive

4. If replanning is triggered:
   - WaypointController detects stuck state (sonar blocked, progress stalled)
   - Calls replanner automatically (up to MAX_REPLAN_ATTEMPTS=3)
   - After 3 attempts, fails and returns False to executor
"""

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

from .behavior_tree import Action, BTStatus, Condition, Selector, Sequence
from .object_classifier import BehaviorPolicy, ObjectClassifier, ObjectType
from .waypoint_controller import WaypointController, StuckError
from .doorway_detector import DoorwayDetector
from .hierarchical_planner import HierarchicalPlanner, TopologicalGraph, RoomPathPlanner
import json

LOG = "[NAV_AGENT_V2]"

MAX_REPLAN_ATTEMPTS = 3


class NavAgent:
    """
    Hierarchical navigation agent for fully-known indoor environments.
    """
    
    def __init__(self, robot, house_config=None, nav_controller=None):
        """
        Initialize NavAgent with hierarchical planning.
        
        Args:
            robot: Webots robot supervisor
            house_config: House config object (from house_loader.load_house) - required for ObjectClassifier
            nav_controller: NavigationController instance (required for waypoint execution)
            
        The nav_controller provides get_current_position() and other motion methods
        needed by WaypointController. If not provided, will create a basic one.
        """
        self.robot = robot
        self.nav_controller = nav_controller
        self.house_config = house_config
        
        # Load hierarchical config from expected path
        import os
        # Navigate from skills/nav/ to nao_assist_controller/house/
        # nav_agent_v2.py is at: controllers/nao_assist_controller/skills/nav/nav_agent_v2.py
        # hierarchical_config.json is at: controllers/nao_assist_controller/house/hierarchical_config.json
        nav_file = os.path.abspath(__file__)
        nao_controller_dir = os.path.dirname(os.path.dirname(os.path.dirname(nav_file)))  # skills/nav -> skills -> nao_assist_controller
        hierarchical_config_path = os.path.join(nao_controller_dir, 'house', 'hierarchical_config.json')
        
        print(f"\n{LOG} ===== INITIALIZING HIERARCHICAL NAV AGENT =====")
        print(f"{LOG} Loading config from: {hierarchical_config_path}")
        with open(hierarchical_config_path, 'r') as f:
            self.hierarchical_config = json.load(f)
        print(f"{LOG} Loaded hierarchical config with {len(self.hierarchical_config['rooms'])} rooms")
        
        # Initialize planning layers
        self.planner = HierarchicalPlanner()
        self.planner.load_house_config(self.hierarchical_config)
        print(f"{LOG} [LAYER 1] TopologicalGraph initialized")
        print(f"{LOG} [LAYER 2] RoomPathPlanner initialized")
        
        # Object classifier for room-specific behaviors
        # Requires house_config - use a mock if not provided
        if house_config is None:
            print(f"{LOG} WARNING: house_config not provided - creating mock ObjectClassifier")
            self.classifier = _MockObjectClassifier()
        else:
            self.classifier = ObjectClassifier(house_config)
        print(f"{LOG} [LAYER 3] ObjectClassifier initialized")
        
        # Waypoint controller for following the plan
        # Use provided nav_controller or create a minimal one
        if nav_controller is None:
            # Create minimal nav controller for testing (should not happen in practice)
            from skills.go_to_target import NavigationController
            nav_controller = NavigationController(robot, house_config)
        
        self.waypoint_controller = WaypointController(nav_controller, None, self.classifier)
        print(f"{LOG} [LAYER 3] WaypointController initialized")
        
        # Door detector
        self.doorway_detector = DoorwayDetector()
        print(f"{LOG} [LAYER 3] DoorwayDetector initialized")
        
        # State
        self.current_goal = None
        self.waypoint_queue: Deque[Tuple[float, float]] = deque()
        self.replan_attempts = 0
        
        # Behavior tree
        self._build_behavior_tree()
    
    def _build_behavior_tree(self):
        """
        Build behavior tree for navigation.
        
        Selector (root):
          ├─ IsAtGoal       → SUCCESS if at goal
          ├─ Sequence: Navigate
          │   ├─ PlanPath       → plan waypoints
          │   └─ ExecutePath    → follow waypoints
          └─ Sequence: Recovery
              ├─ IsStuck
              └─ ForceReplan
        """
        
        # Leaf conditions/actions
        at_goal = Condition("IsAtGoal", self._is_at_goal)
        stuck = Condition("IsStuck", self._is_stuck)
        
        plan_path = Action("PlanPath", self._action_plan_path)
        execute_path = Action("ExecutePath", self._action_execute_path)
        force_replan = Action("ForceReplan", self._action_force_replan)
        
        # Sequences
        navigate_seq = Sequence([plan_path, execute_path])
        recovery_seq = Sequence([stuck, force_replan])
        
        # Root selector
        self.behavior_tree = Selector([at_goal, navigate_seq, recovery_seq])
    
    def navigate_to(self, goal_room: str) -> bool:
        """
        Navigate to a room.
        
        Returns:
            True if reached goal, False if failed or stuck.
        """
        print(f"\n{LOG} ===== NAVIGATE TO '{goal_room}' =====")
        self.current_goal = goal_room
        self.replan_attempts = 0
        
        # Run behavior tree to completion
        while True:
            status = self.behavior_tree.tick()
            
            if status == BTStatus.SUCCESS:
                print(f"{LOG} SUCCESS: Reached {goal_room}")
                return True
            elif status == BTStatus.FAILURE:
                print(f"{LOG} FAILURE: Could not reach {goal_room}")
                return False
            
            # RUNNING - continue
            self.robot.step(32)
    
    # ========================================================================
    # BEHAVIOR TREE CONDITIONS
    # ========================================================================
    
    def _is_at_goal(self) -> BTStatus:
        """Check if robot is at goal location."""
        if not self.current_goal:
            return BTStatus.FAILURE
        
        goal_center = self.planner.graph.get_room_center(self.current_goal)
        if not goal_center:
            return BTStatus.FAILURE
        
        robot_x, robot_y = self.robot.getSelf().getPosition()[:2]
        dist = math.sqrt((robot_x - goal_center[0])**2 + (robot_y - goal_center[1])**2)
        
        if dist < 0.5:  # within 0.5m of goal center
            print(f"{LOG} At goal: dist={dist:.2f}m")
            return BTStatus.SUCCESS
        
        return BTStatus.FAILURE
    
    def _is_stuck(self) -> BTStatus:
        """Check if robot is stuck (repeated plan failures)."""
        if self.replan_attempts >= MAX_REPLAN_ATTEMPTS:
            print(f"{LOG} Stuck: max replans reached")
            return BTStatus.SUCCESS
        
        return BTStatus.FAILURE
    
    # ========================================================================
    # BEHAVIOR TREE ACTIONS
    # ========================================================================
    
    def _action_plan_path(self) -> BTStatus:
        """
        Plan path using hierarchical layers.
        
        Layers:
          1. TopologicalGraph + Dijkstra → room sequence
          2. RoomPathPlanner → waypoints
          3. Populate waypoint queue
        """
        print(f"\n{LOG} [ACTION] PlanPath")
        
        robot_x, robot_y = self.robot.getSelf().getPosition()[:2]
        waypoints = self.planner.plan(
            start_xy=(robot_x, robot_y),
            goal_room=self.current_goal
        )
        
        if not waypoints:
            print(f"{LOG} [ACTION] PlanPath FAILED")
            return BTStatus.FAILURE
        
        # Populate queue
        self.waypoint_queue = deque(waypoints)
        self.replan_attempts += 1
        
        print(f"{LOG} [ACTION] PlanPath SUCCESS: {len(waypoints)} waypoints")
        print(f"{LOG} [LAYER 1] Room path: {self.planner.last_plan['room_path']}")
        
        return BTStatus.SUCCESS
    
    def _action_execute_path(self) -> BTStatus:
        """
        Execute waypoint path using WaypointController.
        
        If StuckError is raised, allow replanning.
        """
        print(f"\n{LOG} [ACTION] ExecutePath")
        
        while self.waypoint_queue:
            next_waypoint = self.waypoint_queue[0]
            
            try:
                # Execute with reactive avoidance
                success = self.waypoint_controller.execute(
                    waypoint=next_waypoint,
                    target_id=self.current_goal,
                    target_class=ObjectType.ROOM
                )
                
                if success:
                    self.waypoint_queue.popleft()
                    print(f"{LOG} [ACTION] Reached waypoint, {len(self.waypoint_queue)} remaining")
                else:
                    print(f"{LOG} [ACTION] Waypoint execution running...")
                    return BTStatus.RUNNING
            
            except StuckError as e:
                print(f"{LOG} [ACTION] StuckError at waypoint: {e}")
                return BTStatus.FAILURE
        
        print(f"{LOG} [ACTION] ExecutePath SUCCESS")
        return BTStatus.SUCCESS
    
    def _action_force_replan(self) -> BTStatus:
        """
        Force replanning after stuck error.
        
        - Mark detected sonar obstacles in occupancy grid
        - Re-run layers 1-2 for new route
        - Reset waypoint queue
        """
        print(f"\n{LOG} [ACTION] ForceReplan (attempt {self.replan_attempts})")
        
        if self.replan_attempts >= MAX_REPLAN_ATTEMPTS:
            print(f"{LOG} [ACTION] Max replan attempts reached")
            return BTStatus.FAILURE
        
        # Return to plan path action
        return self._action_plan_path()
    
    # ========================================================================
    # PUBLIC QUERY API (for inspection)
    # ========================================================================
    
    def get_planner(self):
        """
        Return a planner adapter for backward compatibility with executor.
        
        The executor expects: agent.get_planner().plan((x1, y1), (x2, y2))
        This adapter detects which room contains (x2, y2) and routes to hierarchical planner.
        """
        return _PlannerAdapter(self.planner, self.robot)
    
    @property
    def _wp_ctrl(self):
        """Property for backward compatibility with executor."""
        return self.waypoint_controller
    
    def get_plan_info(self) -> Optional[dict]:
        """Return information about current plan (for debugging)."""
        return self.planner.last_plan
    
    def print_layer_info(self):
        """Print detailed info about planning layers."""
        print(f"\n{LOG} ===== PLANNING LAYER INFO =====")
        
        print(f"\n{LOG} [LAYER 1] TOPOLOGICAL GRAPH")
        print(f"  Rooms: {list(self.planner.graph.rooms.keys())}")
        print(f"  Hallways: {len(self.planner.graph.hallways)} connections")
        for (a, b), cost in self.planner.graph.hallways.items():
            print(f"    {a} <-> {b}: cost={cost}")
        
        print(f"\n{LOG} [LAYER 2] ROOM ENTRY POINTS")
        for room_id, room_data in self.planner.graph.rooms.items():
            entries = room_data['entry_points']
            print(f"  {room_id}: {len(entries)} entry points")
            for i, ep in enumerate(entries):
                print(f"    [{i}] {ep}")
        
        if self.planner.last_plan:
            print(f"\n{LOG} [LAYER 3] LAST PLAN")
            print(f"  Room sequence: {self.planner.last_plan['room_path']}")
            print(f"  Waypoints ({len(self.planner.last_plan['waypoints'])}):")
            for i, wp in enumerate(self.planner.last_plan['waypoints']):
                print(f"    [{i}] {wp}")


# ============================================================================
# Mock ObjectClassifier for backward compatibility
# ============================================================================

class _MockObjectClassifier:
    """Mock ObjectClassifier when house_config is not available."""
    
    def classify_def(self, def_name):
        return ObjectType.ROOM
    
    def classify_target(self, target_id):
        return ObjectType.ROOM
    
    def classify_node(self, node_type_str):
        return ObjectType.ROOM
    
    def get_policy(self, obj_type, is_goal):
        return BehaviorPolicy.APPROACH if is_goal else BehaviorPolicy.TRAVERSE
    
    def get_arrive_dist(self, obj_type):
        return 0.70
    
    def get_nav_target_class(self, obj_type):
        return "room"


# ============================================================================
# Adapter for backward compatibility with executor's grid planner API
# ============================================================================

class _PlannerAdapter:
    """Adapts HierarchicalPlanner to the old GridPlanner API (coordinate-based)."""
    
    def __init__(self, hierarchical_planner, robot):
        self.hierarchical_planner = hierarchical_planner
        self.robot = robot
    
    def plan(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan from start coordinates to goal coordinates.
        
        Detects which room contains the goal coordinates and uses hierarchical planning.
        For door approach points (in corridor), finds the nearest room and routes to it.
        """
        print(f"\n{LOG} ════════════════════════════════════════════════════════════")
        print(f"{LOG} [ADAPTER] plan(start_xy={start_xy}, goal_xy={goal_xy})")
        print(f"{LOG} ════════════════════════════════════════════════════════════")
        
        # First try direct containment
        print(f"{LOG} [ADAPTER] Checking which room contains goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})...")
        goal_room = self._find_room_containing(goal_xy)
        
        if goal_room:
            print(f"{LOG} [ADAPTER] ✓ Goal found directly in room: '{goal_room}'")
        else:
            print(f"{LOG} [ADAPTER] ✗ Goal NOT in any room bounds")
            # If not found in any room, find the nearest room
            # (likely a door approach point in corridor)
            print(f"{LOG} [ADAPTER] Finding nearest room by entry points...")
            goal_room = self._find_nearest_room(goal_xy)
            if goal_room is None:
                print(f"{LOG} [ADAPTER] ✗✗ ERROR: Goal not in any room and no nearest room found!")
                return None
            print(f"{LOG} [ADAPTER] ✓ Nearest room found: '{goal_room}'")
        
        print(f"{LOG} [ADAPTER] → Calling hierarchical_planner.plan(start_xy={start_xy}, goal_room='{goal_room}')...")
        try:
            waypoints = self.hierarchical_planner.plan(start_xy=start_xy, goal_room=goal_room)
            if waypoints:
                print(f"{LOG} [ADAPTER] ✓ Plan returned {len(waypoints)} waypoints")
                # Append goal point as final waypoint
                waypoints.append(goal_xy)
                print(f"{LOG} [ADAPTER] ✓ Appended final goal: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
                print(f"{LOG} [ADAPTER] ✓ Total waypoints: {len(waypoints)}")
                return waypoints
            else:
                print(f"{LOG} [ADAPTER] ✗ Hierarchical plan returned None/empty")
                return None
        except Exception as e:
            print(f"{LOG} [ADAPTER] ✗ Hierarchical plan exception: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _find_room_containing(self, point: Tuple[float, float]) -> Optional[str]:
        """Find which room contains this point based on bounds."""
        x, y = point
        print(f"{LOG} [ADAPTER] [BOUNDS CHECK] Testing point ({x:.2f}, {y:.2f}) against all room bounds:")
        for room_id, room_data in self.hierarchical_planner.graph.rooms.items():
            if 'bounds' in room_data:
                bounds = room_data['bounds']
                x_min, x_max, y_min, y_max = bounds
                inside = (x_min <= x <= x_max and y_min <= y <= y_max)
                status = "✓ INSIDE" if inside else "✗ outside"
                print(f"{LOG} [ADAPTER]   {room_id}: bounds=({x_min:.2f},{x_max:.2f}, {y_min:.2f},{y_max:.2f}) {status}")
                if inside:
                    return room_id
        return None
    
    def _find_nearest_room(self, point: Tuple[float, float]) -> Optional[str]:
        """Find the room with entry point closest to this point."""
        x, y = point
        nearest_room = None
        nearest_dist = float('inf')
        
        print(f"{LOG} [ADAPTER] [NEAREST ROOM] Finding nearest entry point to ({x:.2f}, {y:.2f}):")
        for room_id, room_data in self.hierarchical_planner.graph.rooms.items():
            for ep in room_data.get('entry_points', []):
                dx = ep[0] - x
                dy = ep[1] - y
                dist = (dx*dx + dy*dy) ** 0.5
                print(f"{LOG} [ADAPTER]   {room_id} entry ({ep[0]:.2f},{ep[1]:.2f}): dist={dist:.3f}m")
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_room = room_id
        
        print(f"{LOG} [ADAPTER] → Best match: '{nearest_room}' at distance {nearest_dist:.3f}m")
        return nearest_room if nearest_dist < 2.0 else None  # 2m max threshold


# ============================================================================
# Placeholder for BehaviorTree (reuse from before if available)
# ============================================================================

class BTStatus:
    """Behavior tree status enum."""
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"


class Condition:
    def __init__(self, name: str, check_fn):
        self.name = name
        self.check_fn = check_fn
    
    def tick(self) -> str:
        return self.check_fn()


class Action:
    def __init__(self, name: str, execute_fn):
        self.name = name
        self.execute_fn = execute_fn
    
    def tick(self) -> str:
        return self.execute_fn()


class Sequence:
    def __init__(self, children: List):
        self.children = children
        self.current_child = 0
    
    def tick(self) -> str:
        while self.current_child < len(self.children):
            status = self.children[self.current_child].tick()
            if status != BTStatus.SUCCESS:
                return status
            self.current_child += 1
        self.current_child = 0
        return BTStatus.SUCCESS


class Selector:
    def __init__(self, children: List):
        self.children = children
    
    def tick(self) -> str:
        for child in self.children:
            status = child.tick()
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE
