"""
House Loader - Load static house configuration and target positions.
Iteration 3: Static house knowledge + doors via DEF nodes in Webots world.

This module:
1. Loads house_config.json with target and door definitions
2. Uses Supervisor API to get actual positions of DEF nodes ON-DEMAND
3. Provides get_target_translation(supervisor, target_id) for reliable position fetching
4. Supports route_doors for navigation through doors

NO mapping, NO exploration, NO learning - just static ground-truth from world.
"""

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# PATHS
HOUSE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = HOUSE_DIR / "house_config.json"

LOG_PREFIX = "[HOUSE]"


class HouseConfig:
    """
    Static house configuration loaded from JSON.
    
    Attributes:
        targets: List of target definitions
        doors: List of door definitions
        arrive_distance_m: Distance threshold to consider arrival
        door_arrive_distance_m: Distance to stop before door
        timeout_s: Navigation timeout in seconds
        rotation_tolerance_rad: Rotation accuracy threshold
        step_duration_s: Time per movement step
        door_wait_time_s: Time to wait after sending door command
        emitter_channel: Channel for NAO emitter to door manager
    """
    
    def __init__(self):
        self.targets: List[Dict] = []
        self.doors: List[Dict] = []
        self.arrive_distance_m: float = 0.3
        self.door_arrive_distance_m: float = 0.3
        self.timeout_s: int = 90
        self.rotation_tolerance_rad: float = 0.15
        self.step_duration_s: float = 0.8
        self.door_wait_time_s: float = 2.0
        self.emitter_channel: int = 1
        
        # Supervisor reference for on-demand position fetching
        self._supervisor = None
        
        # Lookup tables (populated by load_config)
        self._target_by_id: Dict[str, Dict] = {}
        self._target_by_label: Dict[str, Dict] = {}
        
        # Door data
        self._door_by_id: Dict[str, Dict] = {}
        self._door_by_label: Dict[str, Dict] = {}
        
        # Position cache (populated on-demand, can be refreshed)
        self._target_positions: Dict[str, Tuple[float, float, float]] = {}
        self._door_approach_positions: Dict[str, Tuple[float, float, float]] = {}
        
        self._loaded = False
    
    def load_config(self) -> bool:
        """
        Load configuration from house_config.json.
        
        Returns:
            True if loaded successfully, False otherwise.
        """
        if not CONFIG_PATH.exists():
            print(f"[HOUSE] ERROR: Config file not found: {CONFIG_PATH}")
            return False
        
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            # Load targets
            self.targets = data.get("targets", [])
            self.arrive_distance_m = data.get("arrive_distance_m", 0.6)
            self.door_arrive_distance_m = data.get("door_arrive_distance_m", 0.5)
            self.timeout_s = data.get("timeout_s", 90)
            self.rotation_tolerance_rad = data.get("rotation_tolerance_rad", 0.15)
            self.step_duration_s = data.get("step_duration_s", 0.8)
            self.door_wait_time_s = data.get("door_wait_time_s", 2.0)
            self.emitter_channel = data.get("emitter_channel", 1)
            
            # Load doors
            self.doors = data.get("doors", [])
            
            # Build target lookup tables
            for target in self.targets:
                tid = target.get("id", "").lower()
                label = target.get("label", "").lower()
                if tid:
                    self._target_by_id[tid] = target
                if label:
                    self._target_by_label[label] = target
            
            # Build door lookup tables
            for door in self.doors:
                did = door.get("id", "").lower()
                label = door.get("label", "").lower()
                if did:
                    self._door_by_id[did] = door
                if label:
                    self._door_by_label[label] = door
            
            print(f"[HOUSE] Config loaded: {len(self.targets)} targets, {len(self.doors)} doors")
            for t in self.targets:
                route_doors = t.get("route_doors", [])
                print(f"[HOUSE]   Target: {t.get('id')} (route_doors: {route_doors})")
            for d in self.doors:
                print(f"[HOUSE]   Door: {d.get('id')} -> hinge: {d.get('hinge_def')}")
            
            return True
            
        except json.JSONDecodeError as e:
            print(f"[HOUSE] ERROR: Invalid JSON in config: {e}")
            return False
        except Exception as e:
            print(f"[HOUSE] ERROR: Failed to load config: {e}")
            return False
    
    def load_from_world(self, supervisor) -> int:
        """
        Store supervisor reference and preload positions from Webots world.
        
        Args:
            supervisor: Webots Supervisor instance (Robot with supervisor=True)
        
        Returns:
            Number of targets successfully loaded.
        """
        # Store supervisor for on-demand position fetching
        self._supervisor = supervisor
        
        # Check if supervisor has getFromDef
        if not hasattr(supervisor, 'getFromDef'):
            print(f"{LOG_PREFIX} ERROR: Robot does not have Supervisor API")
            print(f"{LOG_PREFIX}   -> Set 'supervisor TRUE' on NAO Robot node in Webots")
            return 0
        
        if not self.targets:
            print(f"{LOG_PREFIX} WARNING: No targets defined in config")
            return 0
        
        loaded_count = 0
        
        # Preload target positions (also validates DEF nodes exist)
        print(f"{LOG_PREFIX} Loading targets from world...")
        for target in self.targets:
            def_name = target.get("def", "")
            tid = target.get("id", "")
            
            if not def_name:
                print(f"{LOG_PREFIX} WARNING: Target '{tid}' has no DEF name in config")
                continue
            
            pos = self._fetch_def_translation(def_name, f"target '{tid}'")
            if pos:
                self._target_positions[tid] = pos
                loaded_count += 1
                print(f"{LOG_PREFIX}   ✓ {tid}: DEF={def_name} -> ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            else:
                print(f"{LOG_PREFIX}   ✗ {tid}: DEF={def_name} -> MISSING")
        
        # Preload door approach positions
        doors_loaded = 0
        print(f"{LOG_PREFIX} Loading door approaches from world...")
        for door in self.doors:
            did = door.get("id", "")
            approach_def = door.get("approach_def", "")
            
            if not approach_def:
                print(f"{LOG_PREFIX} WARNING: Door '{did}' has no approach_def")
                continue
            
            pos = self._fetch_def_translation(approach_def, f"door approach '{did}'")
            if pos:
                self._door_approach_positions[did] = pos
                doors_loaded += 1
                print(f"{LOG_PREFIX}   ✓ {did}: DEF={approach_def} -> ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            else:
                print(f"{LOG_PREFIX}   ✗ {did}: DEF={approach_def} -> MISSING")
        
        self._loaded = loaded_count > 0
        print(f"{LOG_PREFIX} Summary: {loaded_count}/{len(self.targets)} targets, {doors_loaded}/{len(self.doors)} door approaches")
        
        return loaded_count
    
    def _fetch_def_translation(self, def_name: str, context: str = "") -> Optional[Tuple[float, float, float]]:
        """
        Fetch translation from a DEF node in Webots world.
        
        Args:
            def_name: The DEF name (case-sensitive)
            context: Description for error messages
        
        Returns:
            (x, y, z) tuple or None if not found.
        """
        if self._supervisor is None:
            print(f"{LOG_PREFIX} ERROR: No supervisor - call load_from_world first")
            return None
        
        try:
            node = self._supervisor.getFromDef(def_name)
            if node is None:
                print(f"{LOG_PREFIX} ERROR: DEF '{def_name}' not found in world ({context})")
                print(f"{LOG_PREFIX}   -> Add: DEF {def_name} Transform {{ translation X Y Z }}")
                return None
            
            trans_field = node.getField("translation")
            if trans_field is None:
                print(f"{LOG_PREFIX} ERROR: DEF '{def_name}' has no 'translation' field")
                return None
            
            pos = trans_field.getSFVec3f()
            return (pos[0], pos[1], pos[2])
            
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR fetching DEF '{def_name}': {e}")
            return None
    
    def get_target_translation(self, target_id: str, use_cache: bool = True) -> Optional[Tuple[float, float, float]]:
        """
        Get target position, fetching from Webots if needed.
        
        This is the RECOMMENDED way to get target positions - it fetches
        on-demand from the world if not cached, providing reliable access.
        
        Args:
            target_id: Target ID or label (e.g., "kitchen" or "Kitchen")
            use_cache: If True, use cached position if available
        
        Returns:
            (x, y, z) tuple or None if target not found.
        """
        # Resolve to canonical ID
        key = target_id.strip().lower()
        resolved_id = self.resolve_target_id(key)
        
        if resolved_id is None:
            print(f"{LOG_PREFIX} ERROR: Unknown target '{target_id}'")
            print(f"{LOG_PREFIX}   Known targets: {list(self._target_by_id.keys())}")
            return None
        
        # Check cache first
        if use_cache and resolved_id in self._target_positions:
            return self._target_positions[resolved_id]
        
        # Fetch from world
        target = self._target_by_id.get(resolved_id)
        if not target:
            print(f"{LOG_PREFIX} ERROR: Target '{resolved_id}' not in config")
            return None
        
        def_name = target.get("def", "")
        if not def_name:
            print(f"{LOG_PREFIX} ERROR: Target '{resolved_id}' has no DEF name in config")
            return None
        
        # Fetch position from Webots
        pos = self._fetch_def_translation(def_name, f"target '{resolved_id}'")
        if pos:
            # Update cache
            self._target_positions[resolved_id] = pos
        
        return pos
    
    def get_target_pose(self, target_id: str) -> Optional[Tuple[float, float, float]]:
        """
        Get position of a target by ID or label (uses on-demand fetching).
        
        Args:
            target_id: Target ID (e.g., "kitchen") or label (e.g., "Kitchen")
        
        Returns:
            (x, y, z) tuple or None if not found.
        """
        # Use the new on-demand fetching method
        return self.get_target_translation(target_id, use_cache=True)
    
    def resolve_target_id(self, target_ref: str) -> Optional[str]:
        """
        Resolve a target reference (ID or label) to its canonical ID.
        
        Args:
            target_ref: Target ID or label (case-insensitive)
        
        Returns:
            Canonical target ID or None if not found.
        """
        key = target_ref.strip().lower()
        
        if key in self._target_by_id:
            return key
        
        if key in self._target_by_label:
            return self._target_by_label[key].get("id", "").lower()
        
        return None
    
    def get_all_targets(self) -> List[Dict]:
        """
        Get list of all defined targets.
        
        Returns:
            List of target dictionaries with id, def, label.
        """
        return self.targets
    
    def get_available_targets(self) -> List[str]:
        """
        Get list of target IDs that have valid positions loaded.
        
        Returns:
            List of target IDs.
        """
        return list(self._target_positions.keys())
    
    # DOOR METHODS (Iteration 3)
    
    def find_door_by_label(self, label: str) -> Optional[Dict]:
        """
        Find a door by its spoken label (e.g., "kitchen" -> door_hall_kitchen).
        
        Args:
            label: Door label as spoken by user (e.g., "kitchen", "bedroom")
        
        Returns:
            Door dictionary or None if not found.
        """
        key = label.strip().lower()
        return self._door_by_label.get(key)
    
    def find_door_by_id(self, door_id: str) -> Optional[Dict]:
        """
        Find a door by its ID.
        
        Args:
            door_id: Door ID (e.g., "door_hall_kitchen")
        
        Returns:
            Door dictionary or None if not found.
        """
        key = door_id.strip().lower()
        return self._door_by_id.get(key)
    
    def get_route_doors_for_target(self, target_id: str) -> List[str]:
        """
        Get list of door IDs that must be opened to reach a target.
        
        Args:
            target_id: Target ID (e.g., "kitchen")
        
        Returns:
            List of door IDs in order (may be empty).
        """
        key = target_id.strip().lower()
        target = self._target_by_id.get(key)
        if target:
            return target.get("route_doors", [])
        return []
    
    def get_door_approach_position(self, door_id: str) -> Optional[Tuple[float, float, float]]:
        """
        Get the approach position for a door.
        
        Args:
            door_id: Door ID
        
        Returns:
            (x, y, z) position in front of the door, or None.
        """
        key = door_id.strip().lower()
        # Check cached positions from world first
        pos = self._door_approach_positions.get(key)
        if pos:
            return pos

        # Deterministic fallback: if world Supervisor isn't available or the
        # approach DEF was not found, try to use the static doorway frame
        # geometry from the nav doorway detector (frame centres).
        try:
            from skills.nav.doorway_detector import _DOOR_FRAME_DATA
            entry = _DOOR_FRAME_DATA.get(key)
            if entry is not None:
                # entry: (frame_cx, frame_cy, wall_y, inner_half_gap_m)
                return (entry[0], entry[1], 0.0)
        except Exception:
            pass

        return None
    
    def get_door_hinge_def(self, door_id: str) -> Optional[str]:
        """
        Get the hinge DEF name for supervisor door control.
        
        Args:
            door_id: Door ID
        
        Returns:
            Hinge DEF string (e.g., "DOOR_KITCHEN_HINGE") or None.
        """
        door = self.find_door_by_id(door_id)
        if door:
            return door.get("hinge_def")
        return None
    
    def get_all_doors(self) -> List[Dict]:
        """Get list of all defined doors."""
        return self.doors
    
    def list_targets(self) -> List[str]:
        """Get list of all configured target IDs."""
        return list(self._target_by_id.keys())
    
    def list_doors(self) -> List[str]:
        """Get list of all door IDs."""
        return [d.get("id", "") for d in self.doors]
    
    def is_loaded(self) -> bool:
        """Check if at least one target was loaded from world."""
        return self._loaded
    
    def print_targets_status(self):
        """
        Print diagnostic information about all targets.
        Shows id -> def_name -> translation (or MISSING).
        """
        print(f"\n{LOG_PREFIX} " + "=" * 50)
        print(f"{LOG_PREFIX} TARGETS STATUS")
        print(f"{LOG_PREFIX} " + "=" * 50)
        
        for target in self.targets:
            tid = target.get("id", "")
            def_name = target.get("def", "")
            label = target.get("label", "")
            
            # Try to get position (on-demand)
            pos = self.get_target_translation(tid, use_cache=False)
            
            if pos:
                status = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
            else:
                status = "MISSING"
            
            print(f"{LOG_PREFIX}   {tid}: DEF={def_name} -> {status}")
        
        print(f"{LOG_PREFIX} " + "=" * 50 + "\n")


# Global instance for easy access
_house_config: Optional[HouseConfig] = None


def get_house_config() -> HouseConfig:
    """
    Get or create the global house config instance.
    
    Returns:
        HouseConfig instance.
    """
    global _house_config
    if _house_config is None:
        _house_config = HouseConfig()
    return _house_config


def load_house(supervisor) -> HouseConfig:
    """
    Convenience function to load house config and positions.
    
    Args:
        supervisor: Webots Supervisor instance
    
    Returns:
        Loaded HouseConfig instance.
    """
    config = get_house_config()
    config.load_config()
    config.load_from_world(supervisor)
    return config
