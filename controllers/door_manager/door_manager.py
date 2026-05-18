"""
Door Manager Controller - Webots controller for motorized doors.
Iteration 3: Receives commands from NAO via Emitter/Receiver, actuates door motors.

This controller:
1. Owns a Receiver device to get commands from NAO
2. Controls multiple door motors (HingeJoint with RotationalMotor)
3. Responds to OPEN:<door_key> and CLOSE:<door_key> messages

WEBOTS SETUP:
- Create a Robot node (DEF DOOR_MANAGER) with controller "door_manager"
- Add a Receiver device with name "door_receiver", channel 1
- Each door should be a HingeJoint with:
  - DEF name matching manager_key (e.g., KITCHEN_DOOR)
  - device: RotationalMotor (name: "motor")
  - device: PositionSensor (name: "sensor")
  
Alternative: Put door hinges inside this Robot node's children.
"""

from controller import Robot

# =============================================================================
# CONFIGURATION
# =============================================================================
LOG_PREFIX = "[DOOR]"
RECEIVER_NAME = "door_receiver"
RECEIVER_CHANNEL = 1

# Door definitions: manager_key -> (open_angle, close_angle)
# These must match the manager_key values in house_config.json
DOOR_CONFIGS = {
    "KITCHEN_DOOR": {"open": 1.57, "close": 0.0},
    "BEDROOM_DOOR": {"open": 1.57, "close": 0.0},
    "BATHROOM_DOOR": {"open": 1.57, "close": 0.0},
}


class DoorManager:
    """
    Manages multiple door motors via Receiver commands.
    
    Commands:
    - "OPEN:<door_key>" - Opens door to configured angle
    - "CLOSE:<door_key>" - Closes door to 0 position
    """
    
    def __init__(self):
        """Initialize door manager."""
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        if self.timestep == 0:
            self.timestep = 32
        
        # Receiver
        self._receiver = None
        self._init_receiver()
        
        # Door motors: door_key -> Motor device
        self._motors = {}
        self._sensors = {}
        self._init_door_motors()
        
        print(f"{LOG_PREFIX} Door Manager initialized")
        print(f"{LOG_PREFIX} Listening on channel {RECEIVER_CHANNEL}")
        print(f"{LOG_PREFIX} Doors registered: {list(self._motors.keys())}")
    
    def _init_receiver(self):
        """Initialize receiver device."""
        try:
            self._receiver = self.robot.getDevice(RECEIVER_NAME)
            if self._receiver:
                self._receiver.enable(self.timestep)
                # Set channel if configurable
                try:
                    self._receiver.setChannel(RECEIVER_CHANNEL)
                except:
                    pass
                print(f"{LOG_PREFIX} Receiver '{RECEIVER_NAME}' enabled")
            else:
                print(f"{LOG_PREFIX} ERROR: Receiver '{RECEIVER_NAME}' not found!")
                print(f"{LOG_PREFIX}   -> Add a Receiver device named '{RECEIVER_NAME}' to DOOR_MANAGER Robot")
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR initializing receiver: {e}")
    
    def _init_door_motors(self):
        """
        Initialize door motors.
        
        Tries multiple approaches:
        1. Look for motors as direct devices on this Robot
        2. Each motor should be named with the door key or have device name 'motor'
        """
        # Try to find motors as devices on this robot
        # Common naming: KITCHEN_DOOR_motor or just the door key
        
        for door_key in DOOR_CONFIGS.keys():
            motor = None
            sensor = None
            
            # Try direct device names
            device_names_to_try = [
                f"{door_key}_motor",
                f"{door_key.lower()}_motor", 
                door_key,
                door_key.lower(),
            ]
            
            for name in device_names_to_try:
                try:
                    device = self.robot.getDevice(name)
                    if device:
                        motor = device
                        print(f"{LOG_PREFIX} Found motor for '{door_key}': device '{name}'")
                        break
                except:
                    continue
            
            if motor:
                self._motors[door_key] = motor
                # Try to find corresponding sensor
                sensor_names = [f"{door_key}_sensor", f"{door_key.lower()}_sensor"]
                for sname in sensor_names:
                    try:
                        s = self.robot.getDevice(sname)
                        if s:
                            s.enable(self.timestep)
                            self._sensors[door_key] = s
                            break
                    except:
                        continue
            else:
                print(f"{LOG_PREFIX} WARNING: No motor found for '{door_key}'")
                print(f"{LOG_PREFIX}   -> Add a RotationalMotor device named '{door_key}_motor'")
    
    def _open_door(self, door_key: str) -> bool:
        """
        Open a door.
        
        Args:
            door_key: Door manager key (e.g., "KITCHEN_DOOR")
        
        Returns:
            True if command sent, False if door not found.
        """
        door_key = door_key.upper()
        
        if door_key not in self._motors:
            print(f"{LOG_PREFIX} ERROR: Unknown door '{door_key}'")
            return False
        
        motor = self._motors[door_key]
        config = DOOR_CONFIGS.get(door_key, {"open": 1.57})
        target_angle = config.get("open", 1.57)
        
        try:
            motor.setPosition(target_angle)
            print(f"{LOG_PREFIX} OPEN '{door_key}' -> angle {target_angle:.2f} rad")
            return True
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR opening '{door_key}': {e}")
            return False
    
    def _close_door(self, door_key: str) -> bool:
        """
        Close a door.
        
        Args:
            door_key: Door manager key
        
        Returns:
            True if command sent, False if door not found.
        """
        door_key = door_key.upper()
        
        if door_key not in self._motors:
            print(f"{LOG_PREFIX} ERROR: Unknown door '{door_key}'")
            return False
        
        motor = self._motors[door_key]
        config = DOOR_CONFIGS.get(door_key, {"close": 0.0})
        target_angle = config.get("close", 0.0)
        
        try:
            motor.setPosition(target_angle)
            print(f"{LOG_PREFIX} CLOSE '{door_key}' -> angle {target_angle:.2f} rad")
            return True
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR closing '{door_key}': {e}")
            return False
    
    def _process_messages(self):
        """Process incoming messages from receiver."""
        if self._receiver is None:
            return
        
        while self._receiver.getQueueLength() > 0:
            try:
                # Get message data
                data = self._receiver.getData()
                if isinstance(data, bytes):
                    message = data.decode("utf-8").strip()
                else:
                    message = str(data).strip()
                
                print(f"{LOG_PREFIX} Received: '{message}'")
                
                # Parse command
                if message.startswith("OPEN:"):
                    door_key = message[5:].strip()
                    self._open_door(door_key)
                elif message.startswith("CLOSE:"):
                    door_key = message[6:].strip()
                    self._close_door(door_key)
                else:
                    print(f"{LOG_PREFIX} Unknown command: '{message}'")
                
            except Exception as e:
                print(f"{LOG_PREFIX} Error processing message: {e}")
            
            # Move to next message
            self._receiver.nextPacket()
    
    def run(self):
        """Main control loop."""
        print(f"{LOG_PREFIX} " + "=" * 50)
        print(f"{LOG_PREFIX} DOOR MANAGER RUNNING")
        print(f"{LOG_PREFIX} Waiting for commands: OPEN:<key>, CLOSE:<key>")
        print(f"{LOG_PREFIX} " + "=" * 50)
        
        while self.robot.step(self.timestep) != -1:
            self._process_messages()
        
        print(f"{LOG_PREFIX} Door Manager ended")


def main():
    """Entry point."""
    manager = DoorManager()
    manager.run()


if __name__ == "__main__":
    main()
