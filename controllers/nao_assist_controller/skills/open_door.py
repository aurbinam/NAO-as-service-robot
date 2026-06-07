"""
Open Door Skill - Control doors via Supervisor API.
Iteration 3: NAO opens/closes doors by directly setting HingeJoint angles.

Since doors in this world are Solids (not Robots), we cannot use motors/sensors.
Instead, NAO as a Supervisor directly sets the joint position field.

How it works:
1. Get the hinge Solid node by DEF (e.g., DOOR_KITCHEN_HINGE)
2. Find the HingeJoint child node
3. Get jointParameters.position field
4. Set the angle (open: ~1.5 rad, closed: 0.0 rad)

Requires:
- NAO Robot with supervisor TRUE in Webots
- House config with hinge_def, open_angle_rad, close_angle_rad
"""

import time
from typing import TYPE_CHECKING, Optional, Callable

if TYPE_CHECKING:
    from house.house_loader import HouseConfig

# CONSTANTS
LOG_PREFIX = "[OPEN_DOOR]"


class SupervisorDoorController:
    """
    Controls doors by directly setting HingeJoint position via Supervisor API.
    """
    
    def __init__(self, robot, house_config: "HouseConfig"):
        """
        Initialize door controller.
        
        Args:
            robot: Webots Supervisor instance (Robot with supervisor=True)
            house_config: Loaded HouseConfig with door definitions
        """
        self.robot = robot
        self.house_config = house_config
        self.timestep = int(robot.getBasicTimeStep())
        self._door_state: dict = {}  # door_id -> "open" | "closed"
        
        # Verify supervisor capability
        if not hasattr(robot, 'getFromDef'):
            print(f"{LOG_PREFIX} ERROR: Robot is not a Supervisor!")
            print(f"{LOG_PREFIX}   -> Set 'supervisor TRUE' on NAO node in Webots")
    
    def _step(self, duration_s: float) -> bool:
        """
        Step simulation for specified seconds.
        
        Returns:
            False if simulation ended, True otherwise.
        """
        steps = int((duration_s * 1000) / self.timestep)
        for _ in range(max(1, steps)):
            if self.robot.step(self.timestep) == -1:
                return False
        return True
    
    def _find_hinge_joint(self, hinge_solid_node):
        """
        Find the HingeJoint child node inside a Solid.
        
        Args:
            hinge_solid_node: The Solid node containing the HingeJoint
        
        Returns:
            HingeJoint node or None.
        """
        try:
            children_field = hinge_solid_node.getField("children")
            if children_field is None:
                print(f"{LOG_PREFIX} ERROR: Hinge solid has no 'children' field")
                return None
            
            count = children_field.getCount()
            for i in range(count):
                child = children_field.getMFNode(i)
                if child is None:
                    continue
                type_name = child.getTypeName()
                if type_name == "HingeJoint":
                    return child
            
            print(f"{LOG_PREFIX} ERROR: No HingeJoint found in hinge solid children")
            return None
            
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR finding HingeJoint: {e}")
            return None
    
    def _set_hinge_angle(self, hinge_def: str, angle_rad: float) -> bool:
        """
        Set the angle of a door hinge.
        
        Args:
            hinge_def: DEF name of the hinge Solid (e.g., "DOOR_KITCHEN_HINGE")
            angle_rad: Target angle in radians
        
        Returns:
            True if successful, False otherwise.
        """
        print(f"{LOG_PREFIX} Setting {hinge_def} -> {angle_rad:.2f} rad")
        
        try:
            # Get the hinge solid node
            hinge_solid = self.robot.getFromDef(hinge_def)
            if hinge_solid is None:
                print(f"{LOG_PREFIX} ERROR: DEF '{hinge_def}' not found in world")
                print(f"{LOG_PREFIX}   -> Add: DEF {hinge_def} Solid {{ ... HingeJoint ... }}")
                return False
            
            # Find the HingeJoint child
            hinge_joint = self._find_hinge_joint(hinge_solid)
            if hinge_joint is None:
                return False
            
            # Get jointParameters node
            jp_field = hinge_joint.getField("jointParameters")
            if jp_field is None:
                print(f"{LOG_PREFIX} ERROR: HingeJoint has no 'jointParameters' field")
                return False
            
            jp_node = jp_field.getSFNode()
            if jp_node is None:
                print(f"{LOG_PREFIX} ERROR: jointParameters is NULL")
                print(f"{LOG_PREFIX}   -> Ensure HingeJoint has: jointParameters HingeJointParameters {{ position 0 }}")
                return False
            
            # Get the position field
            pos_field = jp_node.getField("position")
            if pos_field is None:
                print(f"{LOG_PREFIX} ERROR: jointParameters has no 'position' field")
                return False
            
            # Set the angle
            pos_field.setSFFloat(angle_rad)
            print(f"{LOG_PREFIX} ✓ Set {hinge_def} position to {angle_rad:.2f} rad")
            
            return True
            
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR setting hinge angle: {e}")
            return False
    
    def open_door(self, door_id: str, wait: bool = True) -> bool:
        """
        Open a door by ID.
        
        Args:
            door_id: Door ID (e.g., "door_hall_kitchen")
            wait: Whether to wait briefly after setting angle
        
        Returns:
            True if successful, False on error.
        """
        if self._door_state.get(door_id) == "open":
            print(f"{LOG_PREFIX} Door '{door_id}' already open")
            return True

        door = self.house_config.find_door_by_id(door_id)
        if not door:
            print(f"{LOG_PREFIX} ERROR: Unknown door '{door_id}'")
            available = self.house_config.list_doors()
            print(f"{LOG_PREFIX}   Known doors: {available}")
            return False

        hinge_def = door.get("hinge_def", "")
        if not hinge_def:
            print(f"{LOG_PREFIX} ERROR: Door '{door_id}' has no hinge_def in config")
            return False

        open_angle = door.get("open_angle_rad", 1.5)
        success = self._set_hinge_angle(hinge_def, open_angle)

        if success:
            self._door_state[door_id] = "open"
            if wait:
                wait_time = getattr(self.house_config, 'door_wait_time_s', 1.5)
                print(f"{LOG_PREFIX} Waiting {wait_time}s for door animation...")
                self._step(wait_time)

        return success
    
    def close_door(self, door_id: str, wait: bool = True) -> bool:
        """
        Close a door by ID.
        
        Args:
            door_id: Door ID
            wait: Whether to wait briefly after setting angle
        
        Returns:
            True if successful, False on error.
        """
        if self._door_state.get(door_id) == "closed":
            print(f"{LOG_PREFIX} Door '{door_id}' already closed")
            return True

        door = self.house_config.find_door_by_id(door_id)
        if not door:
            print(f"{LOG_PREFIX} ERROR: Unknown door '{door_id}'")
            return False

        hinge_def = door.get("hinge_def", "")
        if not hinge_def:
            print(f"{LOG_PREFIX} ERROR: Door '{door_id}' has no hinge_def in config")
            return False

        close_angle = door.get("close_angle_rad", 0.0)
        success = self._set_hinge_angle(hinge_def, close_angle)

        if success:
            self._door_state[door_id] = "closed"
            if wait:
                wait_time = getattr(self.house_config, 'door_wait_time_s', 1.5)
                print(f"{LOG_PREFIX} Waiting {wait_time}s for door animation...")
                self._step(wait_time)

        return success


# Global controller instance
_door_controller: Optional[SupervisorDoorController] = None


def get_door_controller(robot, house_config: "HouseConfig") -> SupervisorDoorController:
    """Get or create door controller instance."""
    global _door_controller
    if _door_controller is None:
        _door_controller = SupervisorDoorController(robot, house_config)
    return _door_controller


def open_door(door_id: str, robot, house_config: "HouseConfig", 
              say_func: Optional[Callable] = None, wait: bool = True) -> bool:
    """
    Open a door by ID.
    
    Args:
        door_id: Door ID (e.g., "door_hall_kitchen")
        robot: Webots Supervisor instance
        house_config: Loaded HouseConfig
        say_func: Optional function to make NAO speak
        wait: Whether to wait for door to open
    
    Returns:
        True if door opened, False on error.
    """
    print(f"\n{LOG_PREFIX} " + "=" * 40)
    print(f"{LOG_PREFIX} OPEN DOOR: '{door_id}'")
    print(f"{LOG_PREFIX} " + "=" * 40)
    
    # Get door info
    door = house_config.find_door_by_id(door_id)
    if not door:
        print(f"{LOG_PREFIX} ERROR: Unknown door '{door_id}'")
        if say_func:
            say_func(f"I don't know the door {door_id}.")
        return False
    
    label = door.get("label", door_id)
    
    if say_func:
        say_func(f"Opening the {label} door.")
    
    controller = get_door_controller(robot, house_config)
    success = controller.open_door(door_id, wait=wait)
    
    if success:
        print(f"{LOG_PREFIX} Door '{door_id}' opened")
    else:
        print(f"{LOG_PREFIX} Failed to open door '{door_id}'")
        if say_func:
            say_func(f"I could not open the {label} door.")
    
    return success


def close_door(door_id: str, robot, house_config: "HouseConfig",
               say_func: Optional[Callable] = None, wait: bool = True) -> bool:
    """
    Close a door by ID.
    
    Args:
        door_id: Door ID
        robot: Webots Supervisor instance
        house_config: Loaded HouseConfig
        say_func: Optional function to make NAO speak
        wait: Whether to wait for door to close
    
    Returns:
        True if door closed, False on error.
    """
    print(f"\n{LOG_PREFIX} " + "=" * 40)
    print(f"{LOG_PREFIX} CLOSE DOOR: '{door_id}'")
    print(f"{LOG_PREFIX} " + "=" * 40)
    
    door = house_config.find_door_by_id(door_id)
    if not door:
        print(f"{LOG_PREFIX} ERROR: Unknown door '{door_id}'")
        return False
    
    label = door.get("label", door_id)
    
    if say_func:
        say_func(f"Closing the {label} door.")
    
    controller = get_door_controller(robot, house_config)
    success = controller.close_door(door_id, wait=wait)
    
    if success:
        print(f"{LOG_PREFIX} Door '{door_id}' closed")
    
    return success


def open_door_by_label(label: str, robot, house_config: "HouseConfig",
                       say_func: Optional[Callable] = None) -> bool:
    """
    Open a door by its spoken label (e.g., "kitchen" for kitchen door).
    
    Args:
        label: Door label as spoken by user
        robot: Webots Supervisor instance  
        house_config: Loaded HouseConfig
        say_func: Optional function to make NAO speak
    
    Returns:
        True if door opened, False on error.
    """
    door = house_config.find_door_by_label(label)
    if not door:
        print(f"{LOG_PREFIX} ERROR: No door with label '{label}'")
        if say_func:
            available = [d.get("label", "") for d in house_config.get_all_doors()]
            say_func(f"I don't know the {label} door. I know: {', '.join(available)}")
        return False
    
    door_id = door.get("id", "")
    return open_door(door_id, robot, house_config, say_func)


def close_door_by_label(label: str, robot, house_config: "HouseConfig",
                        say_func: Optional[Callable] = None) -> bool:
    """
    Close a door by its spoken label.
    
    Args:
        label: Door label as spoken by user
        robot: Webots Supervisor instance
        house_config: Loaded HouseConfig
        say_func: Optional function to make NAO speak
    
    Returns:
        True if door closed, False on error.
    """
    door = house_config.find_door_by_label(label)
    if not door:
        print(f"{LOG_PREFIX} ERROR: No door with label '{label}'")
        return False
    
    door_id = door.get("id", "")
    return close_door(door_id, robot, house_config, say_func)
