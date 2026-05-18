"""
HIERARCHICAL PLANNER — 3-layer path planning for NAO indoor navigation.

================================================================================
FILE PURPOSE & ARCHITECTURE
================================================================================
This module implements a hierarchical path planner that replaces the old A* grid search.
It divides path planning into 3 independent layers:

  LAYER 1: TopologicalGraph (Room-level routing via Dijkstra)
  ─────────────────────────────────────────────────────────
    - Nodes: rooms (kitchen, corridor, bedroom, etc.)
    - Edges: hallway connections with traversal costs (e.g., 1.5 units/hallway)
    - Algorithm: Dijkstra's algorithm for guaranteed shortest path
    - Input: room_id (start), room_id (goal)
    - Output: [start_room, intermediate_rooms..., goal_room]
    - Guarantee: Shortest path in room graph (no approximation, no heuristic)

  LAYER 2: RoomPathPlanner (Waypoint generation within rooms)
  ──────────────────────────────────────────────────────────
    - For each room in topological path, determine entry/exit points
    - Connect waypoints: previous_room_exit → room_center → next_room_entry
    - Algorithm: Simple line-of-sight through room geometry
    - Input: room_sequence, robot_position
    - Output: [(x1,y1), (x2,y2), ..., (goal_x, goal_y)] — full waypoint list
    - Post-processing: Smooth by removing waypoints < 0.3m apart

  LAYER 3: WaypointExecutor (Execution with reactive avoidance)
  ──────────────────────────────────────────────────────────────
    - Runs in WaypointController (separate module)
    - Follows waypoint list, adjusts heading/speed continuously
    - Detects obstacles via sonar and locally adjusts heading
    - Re-triggers Layer 1-2 replanning only if stuck (not on every obstacle)

================================================================================
KEY ADVANTAGES OVER A* GRID SEARCH
================================================================================
✓ Dijkstra on room graph guarantees shortest path (no heuristic approximation)
✓ Room-level planning is O(rooms) ≈ O(5-10) vs O(grid_cells) ≈ O(10,000)
✓ Scales to any house size without memory overhead for grid storage
✓ Waypoints are human-readable (room names) and pre-aligned to geometry
✓ Dynamic obstacles trigger LOCAL avoidance, not full global replan
✓ Easy to add semantic costs (prefer living room over basement)

================================================================================
DATA SOURCES & CALLERS
================================================================================
INPUT: hierarchical_config.json (in nao_assist_controller/house/)
  ├─ rooms: Dict[room_id → {center, entry_points, bounds, ...}]
  ├─ hallways: List[(room_A, room_B, cost), ...]
  └─ room_targets: Dict[room_id → {label, furniture, doors, ...}]

LOADED BY:
  1. nav_agent_v2.py:NavAgent.__init__()
     → Calls: HierarchicalPlanner.load_house_config(config_dict)
  
  2. RoomPathPlanner.__init__() receives TopologicalGraph from HierarchicalPlanner

CALLED BY:
  1. nav_agent_v2.py:NavAgent.navigate_to(target_name)
     → Calls: self.planner.plan(start_xy, goal_room)
     → Returns: List of waypoints [(x1,y1), (x2,y2), ...]
  
  2. WaypointController.execute() (in waypoint_controller.py)
     → Receives waypoint list from layer 2
     → Moves robot along waypoints

FULL CALL CHAIN:
  go_to_target.py:go_to_target(target_id)
    ↓
  NavAgent.__init__() [nav_agent_v2.py]
    ├─ loads hierarchical_config.json
    └─ creates HierarchicalPlanner
    ↓
  NavAgent.navigate_to(target_name)
    ├─ calls HierarchicalPlanner.plan(start_xy, goal_room)
    │   ├─ Layer 1: TopologicalGraph.dijkstra()
    │   ├─ Layer 2: RoomPathPlanner.plan_room_sequence()
    │   └─ returns: [(x1,y1), (x2,y2), ..., (goal_x,goal_y)]
    └─ passes waypoints to WaypointController.execute()

================================================================================
COORDINATE SYSTEMS & CONVENTIONS
================================================================================
- World coordinates: (x, y) in Webots world frame (meters)
- Room coordinates: room_id string (e.g., "kitchen", "corridor")
- Entry points: (x, y) tuples representing door/passage locations
- Cost units: Abstract Dijkstra distance (hallways currently 1.5, could be 1.0-5.0)
- All room centers and entry_points extracted from Webots world at startup

================================================================================
CLASS SUMMARY
================================================================================
1. TopologicalGraph: Manages room graph, runs Dijkstra
   - add_room(room_id, center, entry_points, bounds)
   - add_hallway(room_a, room_b, cost)
   - dijkstra(start_room, goal_room) → [room1, room2, ...]

2. RoomPathPlanner: Converts room sequence to waypoints
   - plan_room_sequence(room_sequence, robot_pos) → [(x,y), ...]
   - _smooth_waypoints() removes close waypoints

3. HierarchicalPlanner: Top-level orchestrator combining layers 1-2
   - load_house_config(config_dict) initializes graph
   - plan(start_xy, goal_room) → full waypoint list
"""

import math
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
import heapq

LOG = "[HIERARCHICAL_PLANNER]"

# ============================================================================
# LAYER 1: TOPOLOGICAL GRAPH (Room-level routing)
# ============================================================================

class TopologicalGraph:
    """
    Graph of rooms and hallway connections.
    
    Usage:
        graph = TopologicalGraph()
        graph.add_room("kitchen", center=(1.0, 1.0), entry_points=[(0.5, 1.0)])
        graph.add_room("bedroom", center=(3.0, 1.0), entry_points=[(2.5, 1.0)])
        graph.add_hallway("kitchen", "bedroom", cost=2.0)
        
        route = graph.dijkstra("kitchen", "bedroom")
        # returns: ["kitchen", "bedroom"]
    """
    
    def __init__(self):
        self.rooms: Dict[str, Dict] = {}  # room_id -> {center, entry_points, furniture}
        self.hallways: Dict[Tuple[str, str], float] = {}  # (room_a, room_b) -> cost
        self.adjacency: Dict[str, List[str]] = defaultdict(list)
    
    def add_room(self, room_id: str, center: Tuple[float, float], 
                 entry_points: List[Tuple[float, float]], 
                 bounds: Optional[Tuple[float, float, float, float]] = None):
        """
        Add a room to the graph.
        
        Args:
            room_id: unique room name
            center: (x, y) center of room
            entry_points: list of (x, y) where robot can enter/exit
            bounds: (x_min, x_max, y_min, y_max) room extents for LOS checks
        """
        self.rooms[room_id] = {
            "center": center,
            "entry_points": entry_points,
            "bounds": bounds or self._compute_bounds(entry_points),
        }
        print(f"{LOG} Added room '{room_id}' at {center}")
    
    def add_hallway(self, room_a: str, room_b: str, cost: float):
        """Add bidirectional connection between rooms."""
        if room_a not in self.rooms or room_b not in self.rooms:
            raise ValueError(f"Room not found: {room_a} or {room_b}")
        
        self.hallways[(room_a, room_b)] = cost
        self.hallways[(room_b, room_a)] = cost
        self.adjacency[room_a].append(room_b)
        self.adjacency[room_b].append(room_a)
        print(f"{LOG} Added hallway '{room_a}' <-> '{room_b}' (cost={cost})")
    
    def build_from_house_config(self, house_config, get_position_func):
        """
        Placeholder for backward compatibility with TaskPlanner.
        
        The old TaskPlanner tried to build a graph of targets/doors, but the new
        hierarchical system works with rooms. This method is kept as a no-op
        since TaskPlanner doesn't actually use the graph for navigation anymore.
        The hierarchical planner is built from hierarchical_config.json instead.
        """
        # No-op: hierarchical planner is built from hierarchical_config.json, not from house_config
        pass
    
    def dijkstra(self, start_room: str, goal_room: str) -> Optional[List[str]]:
        """
        Compute shortest room-level path using Dijkstra's algorithm.
        
        Returns:
            List of room IDs from start to goal, or None if no path exists.
        """
        if start_room == goal_room:
            return [start_room]
        
        if start_room not in self.rooms or goal_room not in self.rooms:
            print(f"{LOG} ERROR: Invalid rooms {start_room} or {goal_room}")
            return None
        
        # Dijkstra: (cost, current_room, path)
        pq = [(0, start_room, [start_room])]
        visited = set()
        
        while pq:
            cost, room, path = heapq.heappop(pq)
            
            if room in visited:
                continue
            visited.add(room)
            
            if room == goal_room:
                print(f"{LOG} DIJKSTRA: found path {' -> '.join(path)} (cost={cost:.2f})")
                return path
            
            # Explore neighbors
            for neighbor in self.adjacency.get(room, []):
                if neighbor not in visited:
                    edge_cost = self.hallways.get((room, neighbor), 1.0)
                    new_cost = cost + edge_cost
                    new_path = path + [neighbor]
                    heapq.heappush(pq, (new_cost, neighbor, new_path))
        
        print(f"{LOG} ERROR: No path from {start_room} to {goal_room}")
        return None
    
    def get_room_center(self, room_id: str) -> Optional[Tuple[float, float]]:
        """Get (x, y) center of room."""
        return self.rooms.get(room_id, {}).get("center")
    
    def get_entry_points(self, room_id: str) -> Optional[List[Tuple[float, float]]]:
        """Get list of entry/exit waypoints for a room."""
        return self.rooms.get(room_id, {}).get("entry_points")
    
    @staticmethod
    def _compute_bounds(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Compute axis-aligned bounding box from entry points."""
        if not points:
            return (0, 0, 0, 0)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return (min(xs) - 1.0, max(xs) + 1.0, min(ys) - 1.0, max(ys) + 1.0)


# ============================================================================
# LAYER 2: ROOM PATH PLANNER (Waypoint generation within rooms)
# ============================================================================

class RoomPathPlanner:
    """
    For a sequence of rooms, generate waypoint paths.
    
    Strategy:
      - Within a room: straight line from entry → center → exit
      - Between rooms: connect exit of room_i to entry of room_i+1
      - Merge nearby waypoints (< 0.3 m)
    """
    
    MIN_WAYPOINT_SPACING = 0.30
    
    def __init__(self, graph: TopologicalGraph):
        self.graph = graph
    
    def plan_room_sequence(self, room_sequence: List[str], 
                          robot_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Generate full waypoint list for a sequence of rooms.
        
        Args:
            room_sequence: ["kitchen", "hallway", "bedroom", ...]
            robot_pos: (x, y) current robot position
        
        Returns:
            List of (x, y) waypoints from current position through all rooms.
        """
        waypoints = []
        current_pos = robot_pos
        
        for i, room_id in enumerate(room_sequence):
            room_center = self.graph.get_room_center(room_id)
            entry_points = self.graph.get_entry_points(room_id)
            
            if not room_center or not entry_points:
                print(f"{LOG} ERROR: missing data for room {room_id}")
                continue
            
            # Find closest entry point to current position
            entry = min(entry_points, key=lambda p: self._dist(current_pos, p))
            
            # Within-room path: current → entry → center (if not last room)
            if i == 0 and self._dist(current_pos, entry) > self.MIN_WAYPOINT_SPACING:
                waypoints.append(entry)
            
            waypoints.append(room_center)
            
            # Exit waypoint for connecting to next room
            if i < len(room_sequence) - 1:
                next_room = room_sequence[i + 1]
                next_entry = self.graph.get_entry_points(next_room)
                if next_entry:
                    # Use exit point facing next room
                    exit_pt = max(entry_points, 
                                 key=lambda p: self._dist(p, next_entry[0]))
                    if self._dist(room_center, exit_pt) > self.MIN_WAYPOINT_SPACING:
                        waypoints.append(exit_pt)
            
            current_pos = room_center
        
        # Smooth waypoints (merge close ones)
        smoothed = self._smooth_waypoints(waypoints)
        print(f"{LOG} Generated {len(smoothed)} waypoints for room sequence: {room_sequence}")
        return smoothed
    
    @staticmethod
    def _dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Euclidean distance."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _smooth_waypoints(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Merge waypoints closer than MIN_WAYPOINT_SPACING."""
        if len(waypoints) < 2:
            return waypoints
        
        smoothed = [waypoints[0]]
        for wp in waypoints[1:]:
            if self._dist(smoothed[-1], wp) > self.MIN_WAYPOINT_SPACING:
                smoothed.append(wp)
        
        print(f"{LOG} Smoothed waypoints: {len(waypoints)} -> {len(smoothed)}")
        return smoothed


# ============================================================================
# LAYER 3: HIERARCHICAL PLANNER (Top-level orchestrator)
# ============================================================================

class HierarchicalPlanner:
    """
    Combines all three layers for end-to-end path planning.
    
    Public API:
        planner = HierarchicalPlanner()
        planner.load_house_config(config)
        
        waypoints = planner.plan(start_xy=(x, y), goal_room="kitchen")
    """
    
    def __init__(self):
        self.graph = TopologicalGraph()
        self.room_planner = RoomPathPlanner(self.graph)
        self.last_plan = None
    
    def load_house_config(self, config: Dict):
        """
        Load house topology from config dict.
        
        Expected format:
        {
            "rooms": {
                "kitchen": {
                    "center": (1.0, 1.0),
                    "entry_points": [(0.5, 1.0), (1.5, 1.0)],
                    "bounds": (0, 2, 0.5, 1.5)
                },
                ...
            },
            "hallways": [
                ("kitchen", "hallway", 2.0),
                ("hallway", "bedroom", 2.0),
                ...
            ]
        }
        """
        # Add rooms
        for room_id, room_data in config.get("rooms", {}).items():
            self.graph.add_room(
                room_id,
                center=tuple(room_data["center"]),
                entry_points=[tuple(p) for p in room_data["entry_points"]],
                bounds=tuple(room_data.get("bounds")) if "bounds" in room_data else None
            )
        
        # Add hallways
        for room_a, room_b, cost in config.get("hallways", []):
            self.graph.add_hallway(room_a, room_b, cost)
        
        print(f"{LOG} Loaded house config with {len(config.get('rooms', {}))} rooms")
    
    def plan(self, start_xy: Tuple[float, float], goal_room: str) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start position to goal room.
        
        Returns:
            List of (x, y) waypoints, or None if no path exists.
        """
        print(f"\n{LOG} ════════════════════════════════════════════════════════════")
        print(f"{LOG} [LAYER 1] PLANNING: start_xy={start_xy}, goal_room='{goal_room}'")
        print(f"{LOG} ════════════════════════════════════════════════════════════")
        
        # Layer 1: Find nearest room to start position
        start_room = self._find_nearest_room(start_xy)
        print(f"{LOG} [LAYER 1] Start position ({start_xy[0]:.2f}, {start_xy[1]:.2f}) → detected room: '{start_room}'")
        
        if not start_room:
            print(f"{LOG} [LAYER 1] ✗ ERROR: Could not determine start room from position")
            return None
        
        # Layer 1: Dijkstra room-level path
        room_path = self.graph.dijkstra(start_room, goal_room)
        if not room_path:
            print(f"{LOG} [LAYER 1] ✗ ERROR: No room-level path found from '{start_room}' to '{goal_room}'")
            return None
        print(f"{LOG} [LAYER 1] ✓ Dijkstra room path: {' → '.join(room_path)}")
        
        # Layer 2: Generate waypoints for room sequence
        print(f"{LOG} [LAYER 2] Generating waypoints for room sequence...")
        waypoints = self.room_planner.plan_room_sequence(room_path, start_xy)
        print(f"{LOG} [LAYER 2] ✓ Generated {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"{LOG} [LAYER 2]   [{i}] ({wp[0]:.2f}, {wp[1]:.2f})")
        
        self.last_plan = {
            "room_path": room_path,
            "waypoints": waypoints,
            "start": start_xy,
            "goal": goal_room
        }
        
        print(f"{LOG} [PLANNING] ✓ COMPLETE - Ready for execution\\n")
        return waypoints
    
    def _find_nearest_room(self, pos: Tuple[float, float]) -> Optional[str]:
        """
        Find nearest room to position (within room bounds).
        """
        best_room = None
        best_dist = float('inf')
        
        for room_id, room_data in self.graph.rooms.items():
            center = room_data["center"]
            dist = math.sqrt((pos[0] - center[0])**2 + (pos[1] - center[1])**2)
            if dist < best_dist:
                best_dist = dist
                best_room = room_id
        
        return best_room
    
    def get_plan_info(self) -> Optional[Dict]:
        """Return last computed plan."""
        return self.last_plan
