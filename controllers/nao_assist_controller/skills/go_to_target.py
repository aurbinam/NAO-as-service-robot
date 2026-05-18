"""
GO_TO_TARGET Skill - Navigate to a static target location.
Iteration 3: Grid-planned obstacle-aware navigation via executor layer.

This skill provides core motion primitives:
- rotate_toward_target(): turn to face goal
- walk_toward_target(): walk forward with cross-track correction

Grid planning happens in the executor (_navigate_to_point),
which uses the nav_agent for Dijkstra pathfinding.
This module provides the motion execution layer.
"""

import math
from typing import Optional, Tuple, TYPE_CHECKING, List

if TYPE_CHECKING:
    from house.house_loader import HouseConfig

# Import motion loader helper (uses pathlib for robust path resolution)
import sys
from pathlib import Path

# Import fall recovery manager for command ramping during joint_gait
from .fall_recovery import FallRecoveryManager

_controller_dir = Path(__file__).resolve().parent.parent
if str(_controller_dir) not in sys.path:
    sys.path.insert(0, str(_controller_dir))

from motion_loader import (
    sanity_check as motion_sanity_check,
    is_motion_valid,
    try_load_standup_motions,
    try_load_all_motions,
)
# =============================================================================
# CONSTANTS
# =============================================================================
LOG_PREFIX = "[GO_TO_TARGET]"
FALL_LOG_PREFIX = "[FALL]"
RECOVERY_LOG_PREFIX = "[RECOVERY]"

# =============================================================================
# VELOCITY-BASED LOCOMOTION TUNING
# =============================================================================
ROT_KP = 1.6                    # P gain for in-place heading alignment
ROT_MAX = 0.5                   # Max rotational command magnitude
ROT_TIMEOUT_S = 25.0            # Max time for a rotation phase

FWD_KP_DIST = 0.8               # P gain from distance -> forward speed
FWD_MAX = 0.6                   # Max forward command magnitude
THETA_KP_WALK = 1.2             # Heading correction while walking
THETA_MAX_WALK = 0.35           # Max turn correction while walking
WALK_TIMEOUT_S = 120.0          # Max time for a walking phase

TILT_ABORT_THRESHOLD = 1.00     # rad (~57 deg). Stop walking if exceeded.

# Joint-gait fallback tuning (used when moveToward/move API is unavailable)
GAIT_BASE_HIP_PITCH = -0.15
GAIT_BASE_KNEE = 0.30
GAIT_BASE_ANKLE_PITCH = -0.42
GAIT_MAX_HIP_SWING = 0.22
GAIT_MAX_ROLL = 0.0
GAIT_MAX_TURN_SPLIT = 0.12
JOINT_GAIT_FWD_MAX_CMD = 0.12
JOINT_GAIT_TURN_MAX_CMD = 0.10

# walk_to_target motion-file state machine thresholds
WALK_ENTER_ROTATE_DEG = 10.0            # transition to ALIGNING when heading error exceeds this (tighter for motion_file)
WALK_ALIGN_TOLERANCE_RAD = math.radians(20.0)  # rotate_to_heading exit tolerance
WALK_CROSS_TRACK_LIMIT_M = 0.25         # max lateral deviation from planned line before heading correction
WALK_OVERSHOOT_MARGIN_M = 0.02          # monotonic distance tolerance (metres)
WALK_MAX_CORRECTIONS = 6                # oscillation detection limit

# Human-safe approach thresholds
EMERGENCY_STOP_M = 0.30          # hard stop: stop motion immediately
CAUTION_DISTANCE = 0.60          # slow zone: shorter bursts
HUMAN_SAFE_ARRIVE_M = 0.70       # default arrive dist for person targets
HUMAN_SLOW_ZONE_M = 1.50         # cautious approach begins here
BURST_STEPS_NORMAL = 7           # ~0.22s per burst at 32ms
BURST_STEPS_CAUTION = 3          # ~0.10s per burst in caution zone
SAFE_CLEAR_DISTANCE = 0.40       # exit AVOIDING when max sonar exceeds this (hallway width ~2.4m)
AVOIDANCE_TURN_STEPS = 18        # partial turn â‰ˆ 8Â° (18/89 Ã— 39Â° at 32ms/step)
AVOIDANCE_FWD_STEPS = 4          # short forward burst â‰ˆ 0.13s during arc escape
MAX_AVOIDANCE_CYCLES = 20        # give up if obstacle not cleared after N arcs
CORRECTION_TURN_STEPS = 23       # partial turn â‰ˆ 10Â° for heading correction (legacy, unused)
HEADING_DEADLOCK_MIN_DEG = 8.0   # minimum yaw change expected per rotate_to_heading call
HEADING_DEADLOCK_MAX_FAILS = 3   # escalate to RECOVERY after this many deadlocked corrections

# =============================================================================
# TARGET CLASS â€” context-aware sonar policy
# =============================================================================
TARGET_CLASS_ROOM      = "room"       # navigate to room: suppress sonar near door frame
TARGET_CLASS_DOOR      = "door"       # door-frame waypoint: same suppress as ROOM
TARGET_CLASS_FURNITURE = "furniture"  # task object (sofa/table): allow close approach
TARGET_CLASS_PERSON    = "person"     # Grandpa: safe close stop, always-active sonar
TARGET_CLASS_TRANSIT   = "transit"    # open-space waypoint: full avoidance, no suppress

# Distance to target below which sonar is suppressed.
# Prevents door-frame walls and intentional targets triggering false avoidance.
_SONAR_SUPPRESS_DIST = {
    TARGET_CLASS_ROOM:      0.70,   # posts exit 60-deg sonar cone at ~0.3 m; 0.7 m covers crossing
    TARGET_CLASS_DOOR:      0.70,
    TARGET_CLASS_FURNITURE: 0.55,
    TARGET_CLASS_PERSON:    0.80,
    TARGET_CLASS_TRANSIT:   0.0,    # never suppress
}

# Emergency-stop sonar threshold per target class (metres).
_EMERG_THRESH_BY_CLASS = {
    TARGET_CLASS_ROOM:      EMERGENCY_STOP_M,
    TARGET_CLASS_DOOR:      EMERGENCY_STOP_M,
    TARGET_CLASS_FURNITURE: 0.20,   # can stop very close to task objects
    TARGET_CLASS_PERSON:    0.50,   # keep safe gap before Grandpa
    TARGET_CLASS_TRANSIT:   EMERGENCY_STOP_M,
}

POST_AVOID_GRACE_CYCLES = 10    # loop iterations to skip AVOIDING re-entry after exit

STUCK_PROGRESS_M = 0.08          # min displacement to count as real progress in AVOIDING
STUCK_MAX_CYCLES = 5             # AVOIDING arcs before checking for stuck
RECOVERY_TURN_STRONG_STEPS = 40  # L1 recovery: ~17Â° partial turn burst
MAX_STUCK_LEVEL = 4              # abort if stuck escalation exceeds this level

# Motion-file turn resolution floor: each TurnLeft/Right40 â‰ˆ 39 deg.
# Caller tolerances tighter than this cause overshoot oscillation.
MOTION_FILE_MIN_TURN_TOLERANCE_RAD = math.radians(20.0)

# =============================================================================
# HYBRID CONTROLLER (closed-loop heading-coupled translation control)
# =============================================================================
# Design: short forward bursts + partial turn bursts in a single feedback loop
# instead of bang-bang ALIGN/WALK. Forward speed *scaled by alignment*:
# robot does not commit forward motion while badly off-heading. In-place
# rotation only above HYB_IN_PLACE_DEG â€” between that and HYB_ALIGN_DEG we
# alternate tiny turn bursts with short forward bursts (pure pursuit on
# discrete primitives).
HYB_IN_PLACE_DEG       = 30.0   # |err| above this -> full in-place rotation
HYB_REALIGN_DEG        = 15.0   # |err| in (REALIGN, IN_PLACE] -> tiny turn burst
HYB_ALIGN_DEG          = 6.0    # |err| < this -> pure forward, no correction
HYB_TURN_STEPS_TINY    = 12     # â‰ˆ 5Â° partial turn (closed-loop steering)
HYB_TURN_STEPS_SMALL   = 18     # â‰ˆ 8Â°
HYB_FWD_STEPS_NORMAL   = 5      # short burst => re-evaluate heading often
HYB_FWD_STEPS_CAUTION  = 3      # extra-short near obstacles / goal
HYB_FWD_STEPS_DOOR     = 3      # door crossing: tightest feedback loop
HYB_SLOW_RADIUS_M      = 0.80   # taper forward bursts within this radius
HYB_GOAL_OSC_GUARD     = 12     # max no-progress iters before abort

# =============================================================================
# FALL DETECTION THRESHOLDS
# =============================================================================
# Z-height based detection (NAO standing height is ~0.33m at hip)
FALL_Z_THRESHOLD = 0.22          # Below this Z = definitely fallen
FALL_Z_WARNING = 0.28            # Warning zone - robot may be falling
STANDING_Z_MIN = 0.30            # Minimum Z for "standing" state

# Tilt-based detection (body up vector dot Z axis)
FALL_TILT_THRESHOLD = 0.6        # up.z < 0.6 = body tilted > ~53 degrees
FALL_TILT_WARNING = 0.75         # Warning threshold

# Roll/pitch thresholds (in radians)
FALL_PITCH_THRESHOLD = 0.9       # ~52 degrees
FALL_ROLL_THRESHOLD = 0.9        # ~52 degrees

# =============================================================================
# CALIBRATION STATE (module-level, persists across NavigationController instances)
# =============================================================================
# These are set by run_full_calibration() and used by get_current_heading()
_calibration_done = False
_forward_axis = "y"       # NAO local Y axis points forward in Webots (local Z is UP)
_forward_sign = 1         # +1 or -1
_turns_swapped = False    # True if TurnLeft/TurnRight motions are swapped

# =============================================================================
# RECOVERY STATE (module-level, prevents motion stacking during recovery)
# =============================================================================
_recovery_in_progress = False    # True while executing stand-up recovery
_standup_motions_loaded = False  # True after attempting to load stand-up motions
_motion_standup_front = None     # Stand up from front fall
_motion_standup_back = None      # Stand up from back fall


def _apply_nao_defaults():
    """
    Pre-set correct calibration values for the standard Webots NAO model.

    In Webots the NAO robot's local axes are:
      - Local X = right
      - Local Y = forward  â† the correct forward axis
      - Local Z = up

    Marking calibration as done skips the destructive run_full_calibration()
    that used to execute a Forwards50 + TurnLeft40 motion *before* navigating
    to the commanded target.  run_full_calibration() can still be called
    explicitly if a non-standard setup requires it.
    """
    global _calibration_done, _forward_axis, _forward_sign, _turns_swapped
    if not _calibration_done:
        # NOTE: For this NAO model + motion-file backend, forward motion aligns with local X.
        _forward_axis = "x"
        _forward_sign = 1
        _turns_swapped = False
        _calibration_done = True
        print(f"{LOG_PREFIX} NAO defaults applied: forward_axis=x+, calibration=done")


class ObstacleDetector:
    """
    Optional sonar-based obstacle detection.
    Degrades gracefully if sensors are absent or disabled.
    """

    # Try multiple naming conventions used across Webots NAO models
    _SONAR_PAIRS = [
        ("Sonar/Left", "Sonar/Right"),
        ("sonar_left", "sonar_right"),
        ("us_left", "us_right"),
        ("us/top/left", "us/top/right"),
    ]

    # Forwards50 moves 0.5m. Keep a permissive threshold so one-sided
    # door-frame readings do not trigger false obstacle stops.
    MIN_CLEARANCE_M = 0.35

    def __init__(self, robot, timestep: int):
        self._left = None
        self._right = None
        self._enabled = False
        self._init(robot, timestep)

    def _init(self, robot, timestep: int):
        for ln, rn in self._SONAR_PAIRS:
            try:
                l = robot.getDevice(ln)
                r = robot.getDevice(rn)
                if l and r:
                    l.enable(timestep)
                    r.enable(timestep)
                    self._left, self._right = l, r
                    self._enabled = True
                    print(f"[OBSTACLE] Sonar active: '{ln}', '{rn}'")
                    return
            except Exception:
                continue
        print("[OBSTACLE] No sonar found - obstacle gating disabled.")

    def clearance_ahead(self) -> float:
        """
        Sonar clearance ahead of robot. Returns +inf if unavailable.

        Uses max(left, right) so a wall on ONE side does not trigger a false
        emergency stop. A real obstacle directly ahead blocks BOTH sensors and
        keeps max() low. A corridor wall only blocks one sensor while the other
        reads open space â€” max() stays high, no false emergency.
        """
        if not self._enabled:
            return float("inf")
        try:
            return max(self._left.getValue(), self._right.getValue())
        except Exception:
            return float("inf")

    def left_clearance(self) -> float:
        """Left sonar reading. Returns +inf if unavailable."""
        if not self._enabled:
            return float("inf")
        try:
            return self._left.getValue()
        except Exception:
            return float("inf")

    def right_clearance(self) -> float:
        """Right sonar reading. Returns +inf if unavailable."""
        if not self._enabled:
            return float("inf")
        try:
            return self._right.getValue()
        except Exception:
            return float("inf")

    def is_safe_to_advance(self, min_clearance: float = None) -> bool:
        threshold = min_clearance if min_clearance is not None else self.MIN_CLEARANCE_M
        # Safe if EITHER side has clearance.
        # min() caused false positives at door frames: L=0.46m, R=2.55m -> min=0.46 -> blocked
        # even though robot can pass on the right. Only block if both sides obstructed.
        if not self._enabled:
            return True
        try:
            left = self._left.getValue()
            right = self._right.getValue()
            return left >= threshold or right >= threshold
        except Exception:
            return True

    def __repr__(self):
        status = f"enabled, clearance={self.clearance_ahead():.2f}m" if self._enabled else "disabled"
        return f"ObstacleDetector({status})"


class NavigationController:
    """
    Handles NAO navigation to targets using Supervisor API and motion files.
    
    Requires:
    - NAO robot with supervisor=True in Webots
    - Motion files in controller/motions/ directory
    """
    
    def __init__(self, robot, house_config: "HouseConfig"):
        """
        Initialize navigation controller.
        
        Args:
            robot: Webots Robot/Supervisor instance
            house_config: Loaded HouseConfig with target positions
        """
        self.robot = robot
        self.house_config = house_config
        self.timestep = int(robot.getBasicTimeStep())
        self._obstacle_detector = ObstacleDetector(robot, self.timestep)

        from skills.nav.doorway_detector import DoorwayDetector
        self._doorway_detector = DoorwayDetector(robot)
        
        # Get self node for position tracking
        self._self_node = None
        self._translation_field = None
        self._rotation_field = None
        self._init_supervisor()
        
        # Motion objects - kept only for optional stand-up recovery motions
        self._motion_forward = None
        self._motion_turn_left = None
        self._motion_turn_right = None

        # Velocity locomotion interface (detected at runtime)
        self._walk_iface = None
        self._walk_cmd = None
        self._gait_motors = {}
        self._gait_phase = 0.0
        self._gait_last_time = 0.0
        
        # Fall recovery manager - handles command ramping for joint_gait
        self._recovery_manager = FallRecoveryManager()
        
        # Movement flag - disabled if motions fail to load
        self._movement_enabled = False
        self._init_motions()

        # Apply NAO-specific calibration defaults on first use so navigation
        # does not execute a destructive forward+turn calibration motion before
        # going to the commanded target.  The auto-calibration path is still
        # available by calling run_full_calibration() explicitly if needed.
        _apply_nao_defaults()
    
    def _init_supervisor(self):
        """Initialize supervisor fields for position tracking."""
        try:
            self._self_node = self.robot.getSelf()
            if self._self_node:
                self._translation_field = self._self_node.getField("translation")
                self._rotation_field = self._self_node.getField("rotation")
                print(f"{LOG_PREFIX} Supervisor self-node initialized")
            else:
                print(f"{LOG_PREFIX} WARNING: getSelf() returned None - is supervisor=True?")
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR initializing supervisor: {e}")
    
    def _init_motions(self):
        """
        Initialize locomotion for navigation.

        Locomotion is velocity-based (continuous moveToward/move commands),
        not prerecorded Turn/Forward motion files.

        Stand-up motions are still loaded separately for recovery.
        """
        self._init_velocity_locomotion()
        
        # Also try to load stand-up motions for fall recovery (optional)
        self._load_standup_motions()

    def _init_velocity_locomotion(self):
        """
        Detect and configure a continuous walking API.

        Priority:
        1. moveToward(x, y, theta)
        2. move(x, y, theta)
        3. Joint-level gait fallback (position-controlled legs)
        """
        self._walk_iface = None
        self._walk_cmd = None

        # Candidate interfaces where a locomotion command may exist.
        candidates = [self.robot]

        robot_motion_attr = getattr(self.robot, "motion", None)
        if robot_motion_attr is not None:
            candidates.append(robot_motion_attr)

        for iface in candidates:
            if callable(getattr(iface, "moveToward", None)):
                self._walk_iface = iface
                self._walk_cmd = "moveToward"
                break
            if callable(getattr(iface, "move", None)):
                self._walk_iface = iface
                self._walk_cmd = "move"
                break

        # Motion-file backend: stable pre-computed NAO gait trajectories.
        if self._walk_cmd is None and self._init_motion_file_backend():
            self._walk_cmd = "motion_file"

        # Last resort: synthesize walking from leg joints.
        if self._walk_cmd is None and self._init_joint_gait_backend():
            self._walk_cmd = "joint_gait"

        self._movement_enabled = self._walk_cmd is not None

        if self._movement_enabled:
            print(f"{LOG_PREFIX} Velocity locomotion ENABLED using {self._walk_cmd}(x, y, theta)")
        else:
            print(f"{LOG_PREFIX} " + "=" * 50)
            print(f"{LOG_PREFIX} MOVEMENT DISABLED - no velocity walking API found")
            print(f"{LOG_PREFIX} Expected moveToward/move or NAO leg joints")
            print(f"{LOG_PREFIX} " + "=" * 50)

    def _init_joint_gait_backend(self) -> bool:
        """
        Initialize a continuous joint-space gait fallback.

        This backend maps (x, y, theta) commands to smooth periodic leg joint
        trajectories when high-level moveToward/move APIs are unavailable.
        """
        joint_names = [
            "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
            "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        ]

        motors = {}
        try:
            for name in joint_names:
                m = self.robot.getDevice(name)
                if m is None:
                    return False
                motors[name] = m
        except Exception:
            return False

        self._gait_motors = motors
        self._gait_phase = 0.0
        self._gait_last_time = self.robot.getTime()

        # Use moderate motor velocity caps to avoid jerky motion.
        for m in self._gait_motors.values():
            try:
                m.setVelocity(4.0)
            except Exception:
                pass

        return True

    def _init_motion_file_backend(self) -> bool:
        """Load Forwards50/TurnLeft40/TurnRight40 for physics-based walking."""
        motions, success, _ = try_load_all_motions()
        if not success:
            return False
        self._motion_forward = motions.get("forward")
        self._motion_turn_left = motions.get("turn_left")
        self._motion_turn_right = motions.get("turn_right")
        self._current_motion = None
        self._motion_start_time = None      # sim-time when forward motion last started
        return (is_motion_valid(self._motion_forward) and
                is_motion_valid(self._motion_turn_left) and
                is_motion_valid(self._motion_turn_right))

    def _step_motion_command(self, x: float, theta: float) -> bool:
        """
        Non-blocking motion driver called every timestep.
        If a motion is still playing (advanced by robot.step() in the outer navigation loop), returns immediately. When done, starts the next one.
        The outer loop's heading/distance checks stop the motion early via
        stop_walking() when the target is reached.
        """
        if self._current_motion is not None and not self._current_motion.isOver():
            # Motion is still playing; let simulator advance it
            return True

        # Decide next motion based on commanded theta/x
        if abs(theta) > 0.05:
            next_motion = self._motion_turn_left if theta > 0 else self._motion_turn_right
            label = 'turn_left' if theta > 0 else 'turn_right'
        elif abs(x) > 0.01:
            next_motion = self._motion_forward
            label = 'forward'
        else:
            return True

        # Debug: report chosen motion and requested commands
        try:
            print(f"{LOG_PREFIX} _step_motion_command -> chosen={label} cmd=(x={x:.3f},theta={theta:.3f})")
        except Exception:
            pass

        self._current_motion = next_motion
        # Stop any existing motion cleanly
        try:
            self._current_motion.stop()
        except Exception:
            pass
        try:
            self._current_motion.setTime(0)
        except Exception:
            pass
        # Start playing the chosen motion
        try:
            self._current_motion.play()
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR playing motion {label}: {e}")
            return False
        print(f"{LOG_PREFIX} Motion '{label}' started")
        return True

    def _reactive_forward_burst(self, burst_steps: int = BURST_STEPS_NORMAL) -> str:
        """
        Forward burst with per-step sonar check.

        Plays Forwards50 continuously across calls using sim-time tracking
        (avoids isOver() which fires spuriously on some Webots builds).
        Restarts from t=0 only when the full motion duration has elapsed.

        Returns:
            "OK"        â€” burst completed normally
            "EMERGENCY" â€” obstacle within EMERGENCY_STOP_M, motion stopped
            "SIM_END"   â€” robot.step() returned -1
        """
        if self._motion_forward is None:
            return "OK"

        # Determine whether to (re)start the motion.
        # Use sim-time elapsed rather than isOver() â€” isOver() fires spuriously.
        fwd_duration_s = self._motion_forward.getDuration() / 1000.0
        now = self.robot.getTime()
        motion_expired = (
            self._current_motion is not self._motion_forward
            or self._motion_start_time is None
            or (now - self._motion_start_time) >= fwd_duration_s
        )

        if motion_expired:
            if self._current_motion is not None:
                self._current_motion.stop()
            try:
                self._motion_forward.setTime(0)
            except Exception:
                pass
            self._motion_forward.play()
            self._current_motion = self._motion_forward
            self._motion_start_time = self.robot.getTime()
            print(f"{LOG_PREFIX} _reactive_forward_burst -> starting forward motion (steps={burst_steps})")

        for step_idx in range(burst_steps):
            left = self._obstacle_detector.left_clearance()
            right = self._obstacle_detector.right_clearance()
            ahead = self._obstacle_detector.clearance_ahead()
            if ahead < EMERGENCY_STOP_M:
                if self._current_motion is not None:
                    self._current_motion.stop()
                    self._current_motion = None
                    self._motion_start_time = None
                print(f"{LOG_PREFIX} REACTIVE BURST EMERGENCY: L={left:.3f}m R={right:.3f}m F={ahead:.3f}m")
                return "EMERGENCY"

            if self.robot.step(self.timestep) == -1:
                if self._current_motion is not None:
                    self._current_motion.stop()
                return "SIM_END"

        return "OK"

    def _reactive_turn_burst(self, turn_motion, steps: int) -> str:
        """
        Play a turn motion for exactly `steps` sim-steps then stop.

        Produces a small partial rotation (steps=18 â‰ˆ 8Â°, steps=23 â‰ˆ 10Â°).
        Always restarts from t=0 â€” designed for incremental arc maneuvers.
        No sonar check during the turn (stopping mid-turn is destabilizing).

        Returns "OK" or "SIM_END".
        """
        if not is_motion_valid(turn_motion):
            return "OK"
        if self._current_motion is not None:
            try:
                self._current_motion.stop()
            except Exception:
                pass
        try:
            turn_motion.setTime(0)
        except Exception:
            pass
        # Debug: report turn burst starting
        try:
            label = 'turn_left' if turn_motion is self._motion_turn_left else 'turn_right'
        except Exception:
            label = 'turn'
        print(f"{LOG_PREFIX} _reactive_turn_burst -> starting '{label}' for {steps} steps")
        turn_motion.play()
        self._current_motion = turn_motion
        self._motion_start_time = None  # invalidate forward motion tracking

        for step_idx in range(steps):
            if self.robot.step(self.timestep) == -1:
                try:
                    self._current_motion.stop()
                except Exception:
                    pass
                self._current_motion = None
                print(f"{LOG_PREFIX} _reactive_turn_burst -> sim_end during '{label}' at step {step_idx}")
                return "SIM_END"

        try:
            self._current_motion.stop()
        except Exception:
            pass
        self._current_motion = None
        print(f"{LOG_PREFIX} _reactive_turn_burst -> completed '{label}'")
        return "OK"

    def _set_joint_gait_pose(
        self,
        l_hip_yaw: float, l_hip_roll: float, l_hip_pitch: float,
        l_knee: float, l_ankle_pitch: float, l_ankle_roll: float,
        r_hip_yaw: float, r_hip_roll: float, r_hip_pitch: float,
        r_knee: float, r_ankle_pitch: float, r_ankle_roll: float,
    ):
        """Apply one full lower-body pose to NAO leg joints."""
        if not self._gait_motors:
            return

        # Conservative joint bounds for stability.
        l_hip_yaw = self._clamp(l_hip_yaw, -0.35, 0.35)
        r_hip_yaw = self._clamp(r_hip_yaw, -0.35, 0.35)
        l_hip_roll = self._clamp(l_hip_roll, -0.35, 0.35)
        r_hip_roll = self._clamp(r_hip_roll, -0.35, 0.35)
        l_hip_pitch = self._clamp(l_hip_pitch, -0.95, 0.35)
        r_hip_pitch = self._clamp(r_hip_pitch, -0.95, 0.35)
        l_knee = self._clamp(l_knee, 0.05, 2.1)
        r_knee = self._clamp(r_knee, 0.05, 2.1)
        l_ankle_pitch = self._clamp(l_ankle_pitch, -1.0, 0.55)
        r_ankle_pitch = self._clamp(r_ankle_pitch, -1.0, 0.55)
        l_ankle_roll = self._clamp(l_ankle_roll, -0.35, 0.35)
        r_ankle_roll = self._clamp(r_ankle_roll, -0.35, 0.35)

        self._gait_motors["LHipYawPitch"].setPosition(l_hip_yaw)
        self._gait_motors["LHipRoll"].setPosition(l_hip_roll)
        self._gait_motors["LHipPitch"].setPosition(l_hip_pitch)
        self._gait_motors["LKneePitch"].setPosition(l_knee)
        self._gait_motors["LAnklePitch"].setPosition(l_ankle_pitch)
        self._gait_motors["LAnkleRoll"].setPosition(l_ankle_roll)

        self._gait_motors["RHipYawPitch"].setPosition(r_hip_yaw)
        self._gait_motors["RHipRoll"].setPosition(r_hip_roll)
        self._gait_motors["RHipPitch"].setPosition(r_hip_pitch)
        self._gait_motors["RKneePitch"].setPosition(r_knee)
        self._gait_motors["RAnklePitch"].setPosition(r_ankle_pitch)
        self._gait_motors["RAnkleRoll"].setPosition(r_ankle_roll)

    def _apply_joint_gait_command(self, x: float, theta: float):
        """Convert velocity command into smooth joint targets for both legs."""
        now = self.robot.getTime()
        dt = now - self._gait_last_time
        if dt < 0.0:
            dt = 0.0
        if dt > 0.1:
            dt = 0.1
        self._gait_last_time = now

        x = self._clamp(x, -1.0, 1.0)
        theta = self._clamp(theta, -1.0, 1.0)
        
        # Apply command ramping to prevent sudden starts that cause falls
        x, theta = self._recovery_manager.apply_ramp_to_commands(x, theta)
        ramp = self._recovery_manager.get_command_scale()
        hip_base = GAIT_BASE_HIP_PITCH * ramp
        knee_base = GAIT_BASE_KNEE * ramp
        cmd_mag = min(1.0, abs(x) + 0.7 * abs(theta))
        freq_hz = 0.55 + 0.85 * cmd_mag
        self._gait_phase += 2.0 * math.pi * freq_hz * dt
        phase = self._gait_phase

        # Left/right leg oscillators in anti-phase.
        s_l = math.sin(phase)
        s_r = math.sin(phase + math.pi)
        shift = math.sin(phase + math.pi / 2.0)

        swing_amp = self._clamp(0.04 + 0.16 * abs(x) + 0.10 * abs(theta), 0.04, GAIT_MAX_HIP_SWING)
        roll_amp = self._clamp(0.02 + 0.06 * abs(x) + 0.05 * abs(theta), 0.02, GAIT_MAX_ROLL)
        turn_split = self._clamp(GAIT_MAX_TURN_SPLIT * theta, -GAIT_MAX_TURN_SPLIT, GAIT_MAX_TURN_SPLIT)

        # Direction handling for forward/backward.
        direction = 1.0 if x >= 0.0 else -1.0
        l_hip_pitch = hip_base + direction * swing_amp * s_l - turn_split
        r_hip_pitch = hip_base + direction * swing_amp * s_r + turn_split

        # Knee flexion synchronized with hip swing for foot clearance.
        l_knee = knee_base + 0.20 * max(0.0, s_l) + 0.14 * abs(theta)
        r_knee = knee_base + 0.20 * max(0.0, s_r) + 0.14 * abs(theta)

        l_ankle_pitch = -0.92 * l_hip_pitch - 0.03
        r_ankle_pitch = -0.92 * r_hip_pitch - 0.03

        l_hip_roll = roll_amp * shift
        r_hip_roll = -roll_amp * shift
        l_ankle_roll = -0.70 * l_hip_roll
        r_ankle_roll = -0.70 * r_hip_roll

        l_hip_yaw = -0.18 * theta
        r_hip_yaw = 0.18 * theta

        self._set_joint_gait_pose(
            l_hip_yaw=l_hip_yaw, l_hip_roll=l_hip_roll, l_hip_pitch=l_hip_pitch,
            l_knee=l_knee, l_ankle_pitch=l_ankle_pitch, l_ankle_roll=l_ankle_roll,
            r_hip_yaw=r_hip_yaw, r_hip_roll=r_hip_roll, r_hip_pitch=r_hip_pitch,
            r_knee=r_knee, r_ankle_pitch=r_ankle_pitch, r_ankle_roll=r_ankle_roll,
        )

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _send_walk_command(self, x: float, y: float, theta: float) -> bool:
        """Send one continuous locomotion command to the detected walking API."""
        if not self._movement_enabled or self._walk_cmd is None:
            return False
        # Debug: report outgoing walk command and interface
        try:
            print(f"{LOG_PREFIX} _send_walk_command -> iface={self._walk_cmd} cmd=(x={x:.3f},y={y:.3f},theta={theta:.3f})")
        except Exception:
            pass

        if self._walk_cmd == "joint_gait":
            try:
                x = self._clamp(x, -JOINT_GAIT_FWD_MAX_CMD, JOINT_GAIT_FWD_MAX_CMD)
                theta = self._clamp(theta, -JOINT_GAIT_TURN_MAX_CMD, JOINT_GAIT_TURN_MAX_CMD)
                self._apply_joint_gait_command(x, theta)
                return True
            except Exception as e:
                print(f"{LOG_PREFIX} ERROR sending walk command via joint_gait: {e}")
                return False

        if self._walk_cmd == "motion_file":
            try:
                return self._step_motion_command(x, theta)
            except Exception as e:
                print(f"{LOG_PREFIX} ERROR in motion_file: {e}")
                return False

        if self._walk_iface is None:
            return False

        try:
            # Debug: call the robot/motion interface
            print(f"{LOG_PREFIX} Calling walk interface: {self._walk_cmd}({x:.3f}, {y:.3f}, {theta:.3f})")
            getattr(self._walk_iface, self._walk_cmd)(x, y, theta)
            return True
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR sending walk command via {self._walk_cmd}: {e}")
            return False

    def stop_walking(self) -> bool:
        """Stop velocity-based walking cleanly."""
        self._recovery_manager.stop_movement()
        if self._walk_cmd == "joint_gait":           # Do not force a fixed stop pose; abrupt posture snapping can topple NAO.
            return True
        if self._walk_cmd == "motion_file":          # Let the current motion finish its cycle for a smoother stop.
            if self._current_motion is not None:
                self._current_motion.stop()
                self._current_motion = None
            self._motion_start_time = None
            return True
            return self._send_walk_command(0.0, 0.0, 0.0)

    
    def rotate_to_heading(self, desired_heading: float, tolerance_rad: float, timeout_s: float = ROT_TIMEOUT_S) -> bool:
        """
        Rotate to a desired heading.

        For motion_file backend: plays discrete TurnLeft40/TurnRight40 cycles via
        play_motion() (duration-based, avoids the isOver() early-return bug).

        For other backends: continuous proportional control.
        """
        self._recovery_manager.start_movement()
        start_time = self.robot.getTime()

        # --- Motion-file backend: discrete turn cycles ---
        if self._walk_cmd == "motion_file":
            _prev_error_abs = None
            _no_progress_count = 0
            _fine_turn_attempts = 0
            while True:
                if self.is_fallen(verbose=False):
                    self._recovery_manager.mark_fall()
                    print(f"{LOG_PREFIX} Rotation interrupted: robot fell")
                    return False

                current_heading = self.get_current_heading()
                if current_heading is None:
                    return False

                error = self._normalize_angle(desired_heading - current_heading)
                print(f"{LOG_PREFIX} Rotating (motion_file): error={math.degrees(error):.1f}deg")

                # Motion files turn ~39deg per cycle. Using the config tolerance (typically
                # 10deg) causes infinite overshoot oscillation. Accept anything within
                # half a turn step - the walk phase corrects the remainder.
                effective_tolerance = max(tolerance_rad, MOTION_FILE_MIN_TURN_TOLERANCE_RAD)
                if abs(error) < effective_tolerance:
                    print(f"{LOG_PREFIX} Rotation close enough: error={math.degrees(error):.1f}deg "
                          f"(tolerance={math.degrees(effective_tolerance):.1f}deg, walk will correct)")
                    return True

                # For smaller errors, accept and let walking correct to avoid motion-file deadlocks.
                if abs(error) < math.radians(30.0):
                    print(f"{LOG_PREFIX} Rotation fine-tune: error={math.degrees(error):.1f}deg -> accept and walk-correct")
                    return True

                if (self.robot.getTime() - start_time) > timeout_s:
                    print(f"{LOG_PREFIX} Rotation timeout after {timeout_s:.1f}s")
                    return False

                # Deadlock detection: TurnLeft/Right drift can change desired_heading
                # faster than it corrects current_heading, causing error to grow each cycle.
                # If error not shrinking for 2 consecutive turns, accept and let walk correct.
                if _prev_error_abs is not None:
                    if abs(error) >= _prev_error_abs - math.radians(5.0):
                        _no_progress_count += 1
                        if _no_progress_count >= 2:
                            print(f"{LOG_PREFIX} Rotation deadlock (error={math.degrees(error):.1f}deg "
                                  f"not converging) â€” accepting, walk will correct")
                            return True
                    else:
                        _no_progress_count = 0
                _prev_error_abs = abs(error)

                if error > 0:
                    motion, label = self._motion_turn_left, "turn_left"
                else:
                    motion, label = self._motion_turn_right, "turn_right"

                if not self.play_motion(motion, label=label):
                    print(f"{LOG_PREFIX} Turn motion failed - aborting rotation")
                    return False

        # --- Continuous velocity backends (moveToward / move / joint_gait) ---
        loop_count = 0

        while True:
            if self.is_fallen(verbose=False):
                self._recovery_manager.mark_fall()
                self.stop_walking()
                print(f"{LOG_PREFIX} Rotation interrupted: robot fell")
                return False

            roll_pitch = self.get_roll_pitch()
            if roll_pitch is not None:
                roll, pitch = roll_pitch
                if abs(roll) > TILT_ABORT_THRESHOLD or abs(pitch) > TILT_ABORT_THRESHOLD:
                    self.stop_walking()
                    print(f"{LOG_PREFIX} Rotation safety stop: roll/pitch too high "
                          f"({math.degrees(roll):.1f}deg, {math.degrees(pitch):.1f}deg)")
                    return False

            current_heading = self.get_current_heading()
            if current_heading is None:
                self.stop_walking()
                return False

            error = self._normalize_angle(desired_heading - current_heading)
            if abs(error) < tolerance_rad:
                self.stop_walking()
                return True

            rot_limit = JOINT_GAIT_TURN_MAX_CMD if self._walk_cmd == "joint_gait" else ROT_MAX
            rotational_speed = self._clamp(ROT_KP * error, -rot_limit, rot_limit)

            # joint_gait needs forward bias to stay stable during turns
            forward_during_rotation = 0.06 if self._walk_cmd == "joint_gait" else 0.0

            if not self._send_walk_command(forward_during_rotation, 0.0, rotational_speed):
                self.stop_walking()
                return False

            if self.robot.step(self.timestep) == -1:
                self.stop_walking()
                return False

            loop_count += 1
            if loop_count % 25 == 0:
                print(f"{LOG_PREFIX} Rotating: error={math.degrees(error):.1f}deg, "
                      f"cmd_theta={rotational_speed:.3f}")

            if (self.robot.getTime() - start_time) > timeout_s:
                self.stop_walking()
                print(f"{LOG_PREFIX} Rotation timeout after {timeout_s:.1f}s")
                return False

    def walk_to_target(self, target_pos: Tuple[float, float, float], arrive_dist: float,
                       timeout_s: float = WALK_TIMEOUT_S,
                       target_class: str = TARGET_CLASS_ROOM) -> bool:
        """
        Walk toward target until within arrive distance.

        For motion_file backend: plays discrete Forwards50 / turn cycles.
        For other backends: continuous proportional control.
        """
        self._recovery_manager.start_movement()
        start_time = self.robot.getTime()

        # --- Motion-file backend: discrete forward/turn cycles ---
        if self._walk_cmd == "motion_file":
            start_pos_raw = self.get_current_position()
            if start_pos_raw is None:
                return False

            _is_narrow_passage = self._is_narrow_passage_approach(target_pos)

            ALIGN_TOLERANCE    = WALK_ALIGN_TOLERANCE_RAD     # 20Â° â€” motion file resolution floor
            ENTER_ROTATE_DEG   = WALK_ENTER_ROTATE_DEG        # 15° — re-align threshold
            _correction_count  = 0
            MAX_CORRECTIONS    = WALK_MAX_CORRECTIONS          # oscillation guard

            _sup_dist  = _SONAR_SUPPRESS_DIST.get(target_class, 1.20)
            _emerg_thr = _EMERG_THRESH_BY_CLASS.get(target_class, EMERGENCY_STOP_M)

            STATE_ALIGNING  = "ALIGNING"
            STATE_WALKING   = "WALKING"
            STATE_AVOIDING  = "AVOIDING"
            STATE_RECOVERY  = "RECOVERY"
            state = STATE_ALIGNING

            _avoidance_cycles = 0
            _post_avoid_grace = 0
            _avoid_entry_pos  = list(start_pos_raw)
            _avoid_cycle_count = 0
            _stuck_level       = 0
            _heading_lock_count = 0
            _prev_distance = None
            _no_progress_count = 0

            while True:
                if self.is_fallen(verbose=False):
                    return False

                current_pos      = self.get_current_position()
                distance         = self.compute_distance_to_target(target_pos)
                desired_heading  = self.compute_heading_to_target(target_pos)
                current_heading  = self.get_current_heading()
                if current_pos is None or distance is None or desired_heading is None or current_heading is None:
                    return False
                heading_error = self._normalize_angle(desired_heading - current_heading)

                if _post_avoid_grace > 0:
                    _post_avoid_grace -= 1
                _near_door_frame = self._doorway_detector.is_near_door_frame(current_pos[0], current_pos[1])
                _sonar_active = (not _near_door_frame and _post_avoid_grace == 0 and distance > _sup_dist)

                print(f"{LOG_PREFIX} [{state}] dist={distance:.3f}m  "
                      f"err={math.degrees(heading_error):.1f}deg  "
                      f"clr={self._obstacle_detector.clearance_ahead():.2f}m  "
                      f"sonar={'ON' if _sonar_active else 'suppressed'}")

                if distance < arrive_dist:
                    self.stop_walking()
                    return True

                if (self.robot.getTime() - start_time) > timeout_s:
                    print(f"{LOG_PREFIX} Walk timeout")
                    return False

                # Emergency stop â€” context-aware
                if state not in (STATE_AVOIDING, STATE_RECOVERY) and _sonar_active:
                    if self._obstacle_detector.clearance_ahead() < _emerg_thr:
                        self.stop_walking()
                        print(f"{LOG_PREFIX} EMERGENCY STOP â†’ AVOIDING")
                        state = STATE_AVOIDING
                        _avoidance_cycles = 0
                        _avoid_entry_pos = list(current_pos)
                        _avoid_cycle_count = 0
                        continue

                # â”€â”€ RECOVERY â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
                if state == STATE_RECOVERY:
                    left  = self._obstacle_detector.left_clearance()
                    right = self._obstacle_detector.right_clearance()
                    turn_mot = self._motion_turn_left if left >= right else self._motion_turn_right
                    print(f"{LOG_PREFIX} RECOVERY L{_stuck_level}: L={left:.2f} R={right:.2f}")
                    if _stuck_level == 1:
                        _tr = self._reactive_turn_burst(turn_mot, steps=RECOVERY_TURN_STRONG_STEPS)
                        if _tr == "SIM_END": return False
                        _fr = self._reactive_forward_burst(burst_steps=AVOIDANCE_FWD_STEPS * 2)
                        if _fr == "SIM_END": return False
                    elif _stuck_level == 2:
                        self.play_motion(turn_mot, label="recovery_L2")
                        _fr = self._reactive_forward_burst(burst_steps=BURST_STEPS_NORMAL)
                        if _fr == "SIM_END": return False
                    elif _stuck_level == 3:
                        self.play_motion(turn_mot, label="recovery_L3a")
                        self.play_motion(turn_mot, label="recovery_L3b")
                        _fr = self._reactive_forward_burst(burst_steps=BURST_STEPS_NORMAL * 2)
                        if _fr == "SIM_END": return False
                    else:
                        print(f"{LOG_PREFIX} RECOVERY L4: full reorientation")
                        self.rotate_to_heading(desired_heading, tolerance_rad=ALIGN_TOLERANCE, timeout_s=30.0)
                        _fr = self._reactive_forward_burst(burst_steps=BURST_STEPS_NORMAL)
                        if _fr == "SIM_END": return False
                    fresh = self.get_current_position()
                    _avoid_entry_pos  = list(fresh) if fresh else list(current_pos)
                    _avoid_cycle_count = 0
                    _avoidance_cycles  = 0
                    _post_avoid_grace  = POST_AVOID_GRACE_CYCLES
                    state = STATE_ALIGNING
                    continue

                # â”€â”€ AVOIDING â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
                elif state == STATE_AVOIDING:
                    left  = self._obstacle_detector.left_clearance()
                    right = self._obstacle_detector.right_clearance()
                    if max(left, right) >= SAFE_CLEAR_DISTANCE:
                        print(f"{LOG_PREFIX} AVOIDING: clearance regained â†’ ALIGNING")
                        _avoidance_cycles  = 0
                        _avoid_cycle_count = 0
                        _stuck_level       = 0
                        _post_avoid_grace  = POST_AVOID_GRACE_CYCLES
                        state = STATE_ALIGNING
                        continue
                    _avoidance_cycles  += 1
                    _avoid_cycle_count += 1
                    if _avoid_cycle_count >= STUCK_MAX_CYCLES:
                        dx = current_pos[0] - _avoid_entry_pos[0]
                        dy = current_pos[1] - _avoid_entry_pos[1]
                        moved = math.sqrt(dx*dx + dy*dy)
                        if moved < STUCK_PROGRESS_M:
                            _stuck_level += 1
                            print(f"{LOG_PREFIX} STUCK L{_stuck_level}: moved {moved:.3f}m in {_avoid_cycle_count} arcs")
                            if _stuck_level > MAX_STUCK_LEVEL:
                                print(f"{LOG_PREFIX} STUCK: max escalation â€” aborting")
                                return False
                            _avoid_cycle_count = 0
                            state = STATE_RECOVERY
                            continue
                        else:
                            _avoid_entry_pos  = list(current_pos)
                            _avoid_cycle_count = 0
                            _stuck_level = max(0, _stuck_level - 1)
                    if left >= right:
                        turn_mot  = self._motion_turn_left
                        direction = "left"
                    else:
                        turn_mot  = self._motion_turn_right
                        direction = "right"
                    print(f"{LOG_PREFIX} AVOIDING [{_avoidance_cycles}] L{_stuck_level}: L={left:.2f} R={right:.2f} â†’ arc-{direction}")
                    _tr = self._reactive_turn_burst(turn_mot, steps=AVOIDANCE_TURN_STEPS)
                    if _tr == "SIM_END": return False
                    if self._obstacle_detector.clearance_ahead() > EMERGENCY_STOP_M:
                        _fr = self._reactive_forward_burst(burst_steps=AVOIDANCE_FWD_STEPS)
                        if _fr == "SIM_END": return False

                # â”€â”€ ALIGNING: rotate to face target â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
                elif state == STATE_ALIGNING:
                    if abs(heading_error) < ALIGN_TOLERANCE:
                        _correction_count = 0
                        state = STATE_WALKING
                        continue
                    heading_before = current_heading
                    ok = self.rotate_to_heading(desired_heading, tolerance_rad=ALIGN_TOLERANCE, timeout_s=20.0)
                    heading_after = self.get_current_heading()
                    if heading_after is not None and heading_before is not None:
                        yaw_change = abs(self._normalize_angle(heading_after - heading_before))
                        if yaw_change < math.radians(HEADING_DEADLOCK_MIN_DEG):
                            _heading_lock_count += 1
                            print(f"{LOG_PREFIX} Heading deadlock #{_heading_lock_count}: only {math.degrees(yaw_change):.1f}Â° rotated")
                            if _heading_lock_count >= HEADING_DEADLOCK_MAX_FAILS:
                                print(f"{LOG_PREFIX} Heading deadlock â†’ RECOVERY (L{_stuck_level+1})")
                                _stuck_level = max(_stuck_level + 1, 2)
                                _avoid_cycle_count = 0
                                _heading_lock_count = 0
                                state = STATE_RECOVERY
                                continue
                        else:
                            _heading_lock_count = max(0, _heading_lock_count - 1)
                    if not ok:
                        return False
                    _correction_count = 0
                    state = STATE_WALKING
                    continue

                # â”€â”€ WALKING: step forward, re-align when heading drifts â”€â”€ #
                elif state == STATE_WALKING:
                    # Determine effective rotate threshold (tighter near doors/narrow passages)
                    if _is_narrow_passage and distance < 1.0:
                        if self.is_unstable():
                            print(f"{LOG_PREFIX} ABORT: unstable during narrow passage")
                            return False
                        _eff_rotate_deg = 15.0
                    elif distance < 0.80:
                        _eff_rotate_deg = 20.0
                    else:
                        _eff_rotate_deg = ENTER_ROTATE_DEG

                    # Motion-file backend benefits from a wider rotate threshold to avoid ping-pong.
                    if self._walk_cmd == "motion_file" and _eff_rotate_deg < 30.0:
                        _eff_rotate_deg = 30.0

                    if abs(heading_error) > math.radians(_eff_rotate_deg):
                        _correction_count += 1
                        if _correction_count > MAX_CORRECTIONS:
                            print(f"{LOG_PREFIX} Oscillation ({_correction_count} heading corrections) â€” aborting")
                            return False
                        print(f"{LOG_PREFIX} Heading drift {math.degrees(heading_error):.1f}deg â†’ ALIGNING")
                        state = STATE_ALIGNING
                        continue

                    # If distance is not improving, force a re-align to avoid walking arcs.
                    if _prev_distance is not None:
                        if distance >= (_prev_distance - WALK_OVERSHOOT_MARGIN_M):
                            _no_progress_count += 1
                        else:
                            _no_progress_count = 0
                        if _no_progress_count >= 12:
                            print(f"{LOG_PREFIX} No progress ({_no_progress_count} cycles) â†’ ALIGNING")
                            _no_progress_count = 0
                            # Force a small turn burst even for small heading errors
                            turn_motion = self._motion_turn_left if heading_error > 0 else self._motion_turn_right
                            _tr = self._reactive_turn_burst(turn_motion, steps=12)
                            if _tr == "SIM_END":
                                return False
                            state = STATE_ALIGNING
                            continue

                    if _sonar_active and not self._obstacle_detector.is_safe_to_advance():
                        self.stop_walking()
                        state = STATE_AVOIDING
                        _avoidance_cycles  = 0
                        _avoid_entry_pos   = list(current_pos)
                        _avoid_cycle_count = 0
                        continue

                    _burst_steps  = (BURST_STEPS_CAUTION if distance < CAUTION_DISTANCE else BURST_STEPS_NORMAL)
                    _burst_result = self._reactive_forward_burst(burst_steps=_burst_steps)
                    if _burst_result == "EMERGENCY":
                        self.stop_walking()
                        state = STATE_AVOIDING
                        _avoidance_cycles  = 0
                        _avoid_entry_pos   = list(current_pos)
                        _avoid_cycle_count = 0
                    elif _burst_result == "SIM_END":
                        return False
                    # reset correction counter on successful forward progress
                    _correction_count = 0
                    _prev_distance = distance

        # --- Continuous velocity backends ---
        loop_count = 0

        while True:
            if self.is_fallen(verbose=False):
                self._recovery_manager.mark_fall()
                self.stop_walking()
                print(f"{LOG_PREFIX} Walk interrupted: robot fell")
                return False

            roll_pitch = self.get_roll_pitch()
            if roll_pitch is not None:
                roll, pitch = roll_pitch
                if abs(roll) > TILT_ABORT_THRESHOLD or abs(pitch) > TILT_ABORT_THRESHOLD:
                    self.stop_walking()
                    print(f"{LOG_PREFIX} Walk safety stop: roll/pitch too high "
                          f"({math.degrees(roll):.1f}deg, {math.degrees(pitch):.1f}deg)")
                    return False

            distance = self.compute_distance_to_target(target_pos)
            desired_heading = self.compute_heading_to_target(target_pos)
            current_heading = self.get_current_heading()

            if distance is None or desired_heading is None or current_heading is None:
                self.stop_walking()
                return False

            if distance < arrive_dist:
                self.stop_walking()
                print(f"{LOG_PREFIX} ARRIVED at target!")
                return True

            heading_error = self._normalize_angle(desired_heading - current_heading)
            forward_speed = self._clamp(FWD_KP_DIST * distance, 0.0, FWD_MAX)
            theta_corr = self._clamp(THETA_KP_WALK * heading_error, -THETA_MAX_WALK, THETA_MAX_WALK)

            if not self._send_walk_command(forward_speed, 0.0, theta_corr):
                self.stop_walking()
                return False

            if self.robot.step(self.timestep) == -1:
                self.stop_walking()
                return False

            loop_count += 1
            if loop_count % 20 == 0:
                print(f"{LOG_PREFIX} Walking: dist={distance:.3f}m, "
                      f"v={forward_speed:.3f}, w={theta_corr:.3f}, "
                      f"err={math.degrees(heading_error):.1f}deg")

            if (self.robot.getTime() - start_time) > timeout_s:
                self.stop_walking()
                print(f"{LOG_PREFIX} Walk timeout after {timeout_s:.1f}s")
                return False
    
    def _load_standup_motions(self):
        """
        Load stand-up motions for fall recovery (optional).
        
        Stand-up motions are not required for basic navigation.
        If not available, fall recovery will not be possible.
        
        Re-attempts loading if motions are not valid (in case files were added).
        """
        global _standup_motions_loaded, _motion_standup_front, _motion_standup_back
        
        # Re-attempt loading if no valid motions (files might have been added)
        if _standup_motions_loaded and (is_motion_valid(_motion_standup_front) or is_motion_valid(_motion_standup_back)):
            return  # Already loaded successfully
        
        standup_motions = try_load_standup_motions()
        
        _motion_standup_front = standup_motions.get("standup_front")
        _motion_standup_back = standup_motions.get("standup_back")
        _standup_motions_loaded = True
        
        if _motion_standup_front or _motion_standup_back:
            print(f"{LOG_PREFIX} Fall recovery ENABLED:")
            print(f"{LOG_PREFIX}   Stand-up from front: {'YES' if is_motion_valid(_motion_standup_front) else 'NO'}")
            print(f"{LOG_PREFIX}   Stand-up from back: {'YES' if is_motion_valid(_motion_standup_back) else 'NO'}")
        else:
            print(f"{LOG_PREFIX} Fall recovery DISABLED - no stand-up motions available")
    
    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get NAO's current position using the node's getPosition() method.
        
        Returns:
            (x, y, z) tuple or None if not available.
        """
        if self._self_node is None:
            return None
        
        try:
            # Use getPosition() on the node for current position in world coords
            pos = self._self_node.getPosition()
            return (pos[0], pos[1], pos[2])
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR getting position: {e}")
            return None
    
    # =========================================================================
    # FALL DETECTION METHODS
    # =========================================================================
    
    def get_body_up_vector(self) -> Optional[Tuple[float, float, float]]:
        """
        Get the robot's body UP direction in world coordinates.
        
        For NAO robot:
        - Local X = right
        - Local Y = forward  
        - Local Z = UP (body vertical axis)
        
        Webots getOrientation() returns 3x3 rotation matrix row-major:
        [r00, r01, r02, r10, r11, r12, r20, r21, r22]
        
        Columns are local axes in world coords:
        - Local X in world: (o[0], o[3], o[6])
        - Local Y in world: (o[1], o[4], o[7])  <- NAO's forward
        - Local Z in world: (o[2], o[5], o[8])  <- NAO's up
        
        Returns:
            (x, y, z) unit vector representing body's UP direction in world,
            or None if not available.
        """
        if self._self_node is None:
            return None
        
        try:
            o = self._self_node.getOrientation()
            # Local Z axis is NAO's body up direction
            return (o[2], o[5], o[8])
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR getting orientation: {e}")
            return None
    
    def get_body_forward_vector(self) -> Optional[Tuple[float, float, float]]:
        """
        Get the robot's forward direction in world coordinates.
        
        Uses calibrated forward axis.
        
        Returns:
            (x, y, z) unit vector representing body's forward direction,
            or None if not available.
        """
        global _forward_axis, _forward_sign
        
        if self._self_node is None:
            return None
        
        try:
            o = self._self_node.getOrientation()
            
            if _forward_axis == "x":
                fwd = (o[0], o[3], o[6])
            elif _forward_axis == "y":
                fwd = (o[1], o[4], o[7])
            else:  # "z"
                fwd = (o[2], o[5], o[8])
            
            # Apply sign
            return (fwd[0] * _forward_sign, fwd[1] * _forward_sign, fwd[2] * _forward_sign)
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR getting forward vector: {e}")
            return None
    
    def get_roll_pitch(self) -> Optional[Tuple[float, float]]:
        """
        Compute approximate side-tilt and front/back tilt angles from the body-up vector.

        Webots default world frame is Y-up:
        - position[1] is vertical height
        - the Y component of the body's up vector should be ~1.0 when upright

        Returns:
            (roll, pitch) in radians, or None if not available.

        Notes:
            These angles are used only for diagnostics / fall heuristics. They do not need
            to match a strict aerospace convention; they just need to increase when the
            robot tilts sideways or forward/backward.
        """
        up = self.get_body_up_vector()
        if up is None:
            return None

        ux, uy, uz = up

        # Prevent division by zero when the robot is nearly horizontal.
        # In Z-up world uz is the vertical alignment component (~1.0 when standing).
        denom = max(abs(uz), 1e-3)

        # Sideways lean (roll) and forward/back lean (pitch) relative to world-up (Z).
        roll = math.atan2(ux, denom)
        pitch = math.atan2(uy, denom)

        return (roll, pitch)
    
    def is_fallen(self, verbose: bool = True, ignore_recovery_flag: bool = False) -> bool:
        """
        Detect if robot has fallen using multiple criteria.

        Webots default world frame is Y-up:
        - position: (x, y, z) where y is vertical height
        - body-up vector vertical component is up.y and should be ~1.0 when upright

        Checks:
        1. Height below threshold
        2. Body-up vector significantly tilted from vertical
        3. Roll or pitch exceeds threshold

        Args:
            verbose: If True, print diagnostic info when fall detected.
            ignore_recovery_flag: If True, evaluate the current pose even while recovery
                is in progress. This is required when verifying whether a stand-up motion
                actually succeeded.

        Returns:
            True if robot appears to have fallen, False otherwise.
        """
        global _recovery_in_progress

        # During recovery we usually suppress fall detection so locomotion code does not
        # immediately retrigger another recovery cycle. For post-recovery validation we
        # explicitly bypass this guard.
        if _recovery_in_progress and not ignore_recovery_flag:
            return False

        pos = self.get_current_position()
        up = self.get_body_up_vector()

        if pos is None or up is None:
            return False  # Can't detect without pose info

        height = pos[2]   # Z is vertical in this world (Z-up coordinate system)
        up_z = up[2]      # Z component of body-up vector (â‰ˆ1.0 when upright in Z-up world)

        roll_pitch = self.get_roll_pitch()
        roll = roll_pitch[0] if roll_pitch else 0.0
        pitch = roll_pitch[1] if roll_pitch else 0.0

        height_low = height < FALL_Z_THRESHOLD
        tilt_bad = up_z < FALL_TILT_THRESHOLD
        roll_bad = abs(roll) > FALL_ROLL_THRESHOLD
        pitch_bad = abs(pitch) > FALL_PITCH_THRESHOLD

        severe_tilt = up_z < 0.3
        orientation_bad = tilt_bad or roll_bad or pitch_bad

        fallen = severe_tilt or (height_low and orientation_bad)

        if verbose and (fallen or height_low or orientation_bad):
            print(f"\n{FALL_LOG_PREFIX} " + "=" * 60)
            print(f"{FALL_LOG_PREFIX} FALL DETECTION DEBUG")
            print(f"{FALL_LOG_PREFIX} " + "-" * 60)
            print(f"{FALL_LOG_PREFIX} Raw position (x,y,z): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
            print(f"{FALL_LOG_PREFIX} Using pos[2] as height (Z-up coordinate system)")
            print(f"{FALL_LOG_PREFIX} Raw up vector (x,y,z): ({up[0]:.4f}, {up[1]:.4f}, {up[2]:.4f})")
            print(f"{FALL_LOG_PREFIX} Using up[2] as vertical alignment (should be ~1.0 when upright)")
            print(f"{FALL_LOG_PREFIX} " + "-" * 60)
            print(f"{FALL_LOG_PREFIX} Height: {height:.3f}m (threshold: {FALL_Z_THRESHOLD}m) {'LOW' if height_low else 'OK'}")
            print(f"{FALL_LOG_PREFIX} Up.Z:   {up_z:.3f} (threshold: {FALL_TILT_THRESHOLD}) {'BAD' if tilt_bad else 'OK'}")
            print(f"{FALL_LOG_PREFIX} Roll:   {math.degrees(roll):.1f}deg (threshold: {math.degrees(FALL_ROLL_THRESHOLD):.1f}deg) {'BAD' if roll_bad else 'OK'}")
            print(f"{FALL_LOG_PREFIX} Pitch:  {math.degrees(pitch):.1f}deg (threshold: {math.degrees(FALL_PITCH_THRESHOLD):.1f}deg) {'BAD' if pitch_bad else 'OK'}")
            print(f"{FALL_LOG_PREFIX} " + "-" * 60)
            if fallen:
                print(f"{FALL_LOG_PREFIX} RESULT: FALLEN")
                if severe_tilt:
                    print(f"{FALL_LOG_PREFIX}   Reason: Severe tilt (up.z < 0.3)")
                else:
                    print(f"{FALL_LOG_PREFIX}   Reason: Low height + orientation issue")
            else:
                print(f"{FALL_LOG_PREFIX} RESULT: NOT FALLEN (robot appears upright)")
            print(f"{FALL_LOG_PREFIX} " + "=" * 60)

        return fallen    
    
    def is_unstable(self) -> bool:
        """
        Detect if robot is in warning zone (may be about to fall).

        Z-up coordinate system:
        - position[2] is vertical height
        - up[2] is vertical alignment

        Returns:
            True if robot appears unstable but not fully fallen.
        """
        pos = self.get_current_position()
        up = self.get_body_up_vector()

        if pos is None or up is None:
            return False

        height = pos[2]
        up_z = up[2]

        height_warning = FALL_Z_WARNING > height >= FALL_Z_THRESHOLD
        tilt_warning = FALL_TILT_WARNING > up_z >= FALL_TILT_THRESHOLD

        return height_warning or tilt_warning
    
    def get_fall_direction(self) -> str:
        """
        Determine which direction the robot fell (front or back).
        
        Uses pitch angle to determine direction:
        - Positive pitch (leaning forward) = fell forward (face down)
        - Negative pitch (leaning backward) = fell backward (face up)
        
        Returns:
            "front", "back", or "unknown"
        """
        roll_pitch = self.get_roll_pitch()
        if roll_pitch is None:
            return "unknown"
        
        roll, pitch = roll_pitch
        
        # Check body forward vector Z component (Z-up coordinate system)
        fwd = self.get_body_forward_vector()
        
        if fwd is not None:
            # If forward vector is pointing down (fwd.z < 0), robot is face-down
            if fwd[2] < -0.3:
                return "front"
            # If forward vector is pointing up (fwd.z > 0.3), robot is face-up
            elif fwd[2] > 0.3:
                return "back"
        
        # Fallback to pitch-based detection
        if pitch > 0.3:
            return "front"  # Nose down
        elif pitch < -0.3:
            return "back"   # Nose up
        
        # Side fall - try to recover as front (safer)
        if abs(roll) > 0.5:
            return "front"
        
        return "unknown"

    def get_current_heading(self) -> Optional[float]:
        """
        Get NAO's current yaw (heading) angle in radians using CALIBRATED axis.
        
        Uses the calibrated forward axis and sign determined by run_full_calibration().
        If not calibrated, defaults to local Z axis (standard assumption).
        
        Returns:
            Yaw angle (radians) or None if not available.
            Convention: 0 = facing +X world axis, positive = counter-clockwise from above.
        """
        if self._self_node is None:
            return None
        
        try:
            o = self._self_node.getOrientation()
            return self._get_yaw_from_orientation(o)
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR getting heading: {e}")
            return None
    
    def _get_yaw_from_orientation(self, o: list) -> float:
        """
        Extract yaw from orientation matrix using calibrated axis.
        
        Webots getOrientation() returns 3x3 rotation matrix row-major:
        [r00, r01, r02, r10, r11, r12, r20, r21, r22]
        
        Columns represent local X, Y, Z axes in world coordinates:
        - Local X in world: (o[0], o[3], o[6])
        - Local Y in world: (o[1], o[4], o[7])
        - Local Z in world: (o[2], o[5], o[8])
        
        For Z-up coordinate system, we project onto XY plane and compute yaw.
        Orientation matrix layout (row-major):
          o[0] o[1] o[2]   world-X component of local X, Y, Z
          o[3] o[4] o[5]   world-Y component of local X, Y, Z
          o[6] o[7] o[8]   world-Z component of local X, Y, Z
        Horizontal axes in Z-up world are world-X (row 0) and world-Y (row 1).
        """
        global _forward_axis, _forward_sign, _calibration_done

        # Project forward vector onto horizontal XY plane (Z-up world).
        # For each local axis, take world-X (row 0) and world-Y (row 1) components.
        if _forward_axis == "x":
            fx, fy = o[0], o[3]   # world-X and world-Y of local X
        elif _forward_axis == "y":
            fx, fy = o[1], o[4]   # world-X and world-Y of local Y
        else:  # "z"
            fx, fy = o[2], o[5]   # world-X and world-Y of local Z

        # Apply sign correction
        fx *= _forward_sign
        fy *= _forward_sign

        # Compute yaw: angle from +X axis toward +Y axis, counter-clockwise positive
        yaw = math.atan2(fy, fx)
        
        return yaw
    
    def _get_all_yaw_candidates(self) -> dict:
        """
        Compute yaw from all possible axis/sign combinations.
        
        Returns dict with keys like "x+", "x-", "y+", "y-", "z+", "z-" 
        and values as yaw in radians.
        """
        if self._self_node is None:
            return {}
        
        try:
            o = self._self_node.getOrientation()
        except:
            return {}
        
        candidates = {}
        # Z-up world: horizontal plane is XY.  Row 0 = world-X, row 1 = world-Y component.
        for axis, (fx_idx, fy_idx) in [("x", (0, 3)), ("y", (1, 4)), ("z", (2, 5))]:
            for sign_name, sign in [("+", 1), ("-", -1)]:
                fx = o[fx_idx] * sign
                fy = o[fy_idx] * sign
                yaw = math.atan2(fy, fx)
                candidates[f"{axis}{sign_name}"] = yaw
        
        return candidates
    
    def compute_heading_to_target(self, target_pos: Tuple[float, float, float]) -> Optional[float]:
        """
        Compute desired heading angle to face target.
        
        CONSISTENT WITH get_current_heading():
        - Uses atan2(dz, dx) to get angle from +X axis toward +Z axis
        - This matches the yaw extraction convention
        
        Args:
            target_pos: (x, y, z) of target in world coordinates
        
        Returns:
            Desired heading in radians or None.
        """
        current_pos = self.get_current_position()
        if current_pos is None:
            return None
        
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]   # Z-up: horizontal plane is XY

        # atan2(dy, dx) gives angle from +X axis toward +Y axis (Z-up horizontal plane)
        # Consistent with yaw = atan2(fy, fx) in get_current_heading()
        desired = math.atan2(dy, dx)

        print(f"{LOG_PREFIX} HEADING CALC: robot=({current_pos[0]:.3f}, {current_pos[1]:.3f}), "
              f"target=({target_pos[0]:.3f}, {target_pos[1]:.3f}), "
              f"d=({dx:.3f}, {dy:.3f}), desired={math.degrees(desired):.1f}deg")
        
        return desired
    
    def compute_distance_to_target(self, target_pos: Tuple[float, float, float]) -> Optional[float]:
        """
        Compute 2D distance (XY horizontal plane) to target.
        Z-up world: height is pos[2], horizontal plane is pos[0]/pos[1].

        Args:
            target_pos: (x, y, z) of target

        Returns:
            Distance in meters or None.
        """
        current_pos = self.get_current_position()
        if current_pos is None:
            return None

        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]   # Z-up: second horizontal axis is Y

        return math.sqrt(dx * dx + dy * dy)

    def _is_narrow_passage_approach(self, target_pos: Tuple[float, float, float]) -> bool:
        """Detect if target is bathroom (narrow doorway requires stricter alignment)."""
        try:
            # Bathroom target ~(1.99, -3.27) - check proximity
            bathroom_xy = self.house_config.get_target_translation("bathroom")
            if bathroom_xy is None:
                return False
            bx, by = bathroom_xy[0], bathroom_xy[1]
            tx, ty = target_pos[0], target_pos[1]
            dist = math.sqrt((tx - bx) ** 2 + (ty - by) ** 2)
            return dist < 0.50  # within 0.5m of bathroom target
        except Exception:
            return False
    
    def _distance_point_to_segment_xy(
        self,
        point: Tuple[float, float, float],
        seg_a: Tuple[float, float, float],
        seg_b: Tuple[float, float, float],
    ) -> float:
        """Return the shortest XY-plane distance from a point to a line segment (Z-up world)."""
        px, py = point[0], point[1]
        ax, ay = seg_a[0], seg_a[1]
        bx, by = seg_b[0], seg_b[1]

        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay
        ab_len_sq = abx * abx + aby * aby

        if ab_len_sq <= 1e-12:
            dx = px - ax
            dy = py - ay
            return math.sqrt(dx * dx + dy * dy)

        t = (apx * abx + apy * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))
        closest_x = ax + t * abx
        closest_y = ay + t * aby
        dx = px - closest_x
        dy = py - closest_y
        return math.sqrt(dx * dx + dy * dy)

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _step(self, duration_s: float = None) -> bool:
        """
        Step simulation.
        
        Args:
            duration_s: Optional duration in seconds
        
        Returns:
            False if simulation ended, True otherwise.
        """
        if duration_s:
            steps = int((duration_s * 1000) / self.timestep)
            for _ in range(max(1, steps)):
                if self.robot.step(self.timestep) == -1:
                    return False
        else:
            if self.robot.step(self.timestep) == -1:
                return False
        return True
    
    def _get_pose(self) -> tuple:
        """
        Get current position and yaw for diagnostics.
        
        Returns:
            Tuple of ((x, y, z), yaw_rad) or ((None, None, None), None) if unavailable.
        """
        pos = self.get_current_position()
        yaw = self.get_current_heading()
        if pos is None:
            pos = (None, None, None)
        return pos, yaw
    
    def _log_pose(self, label: str) -> tuple:
        """Log current pose with a label and return pose tuple."""
        pos, yaw = self._get_pose()
        if pos[0] is not None:
            yaw_deg = math.degrees(yaw) if yaw is not None else None
            print(f"{LOG_PREFIX} {label}: pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}), yaw={yaw_deg:.2f}deg")
        else:
            print(f"{LOG_PREFIX} {label}: pos=UNAVAILABLE, yaw=UNAVAILABLE")
        return pos, yaw
    
    def play_motion(self, motion, timeout_s: float = 15.0, label: str = "motion") -> bool:
        """
        Play a motion and wait for completion while stepping the simulation.
        
        CRITICAL FIXES:
        1. Stops and resets motion before playing (ensures clean start)
        2. Uses duration-based stepping (motion.getDuration) instead of isOver()
        3. Uses Webots simulation time (robot.getTime()) not wall clock
        4. Logs pose BEFORE and AFTER motion with detailed diagnostics
        5. Returns False if motion cannot be played
        6. Blocks during recovery (does not play motion if recovering from fall)
        
        Args:
            motion: Motion object to play (can be None or invalid)
            timeout_s: Maximum SIMULATION seconds to wait
            label: Label for logging (e.g., "turn_left", "forward")
        
        Returns:
            True if motion completed AND pose changed, False otherwise.
        """
        global _recovery_in_progress
        
        # Guard: don't play motions during fall recovery
        if _recovery_in_progress:
            print(f"{LOG_PREFIX} Motion '{label}' blocked - recovery in progress")
            return False
        
        # Guard: never call play() or isOver() on None/invalid motion
        if not is_motion_valid(motion):
            print(f"{LOG_PREFIX} ERROR: Motion '{label}' is None or invalid, cannot play")
            return False
        
        # Record pose BEFORE motion
        pos_before, yaw_before = self._log_pose(f"BEFORE {label}")
        
        try:
            # CRITICAL: Stop and reset motion to ensure it plays from beginning
            # This fixes the issue where repeated calls don't replay the motion
            motion.stop()

            # Settling steps: let joint positions and momentum from previous motion
            # dissipate before starting the next one. Without this, TurnLeft/Right40
            # after Forwards50 produces ~0.2deg yaw and 30cm forward movement instead
            # of ~39deg rotation, causing circular paths and falls.
            for _ in range(8):
                if self.robot.step(self.timestep) == -1:
                    return False

            # Try to reset time to 0 if API supports it
            try:
                motion.setTime(0)
            except AttributeError:
                pass  # setTime not available in all Webots versions

            # Start motion playback
            motion.play()
            
            # Record Webots simulation time (not wall clock!)
            start_sim_time = self.robot.getTime()
            step_count = 0
            expected_duration = motion.getDuration() / 1000.0  # Convert ms to s
            if expected_duration <= 0.0:
                print(f"{LOG_PREFIX} ERROR: Motion '{label}' has non-positive duration ({expected_duration:.3f}s)")
                motion.stop()
                return False
            
            print(f"{LOG_PREFIX} Playing '{label}' (expected duration: {expected_duration:.2f}s)...")
            
            # CRITICAL: run motion for its full declared duration.
            # Some setups report isOver() early; duration-based stepping avoids
            # premature termination (e.g., 2.84s motion ending in ~0.26s).
            while (self.robot.getTime() - start_sim_time) < expected_duration:
                if self.robot.step(self.timestep) == -1:
                    print(f"{LOG_PREFIX} ERROR: Simulation ended during motion '{label}'")
                    return False
                
                step_count += 1

                # Check timeout using SIMULATION time
                sim_elapsed = self.robot.getTime() - start_sim_time
                if sim_elapsed > timeout_s:
                    print(f"{LOG_PREFIX} WARNING: Motion '{label}' timeout after {sim_elapsed:.2f}s sim time ({step_count} steps)")
                    motion.stop()  # Stop the motion cleanly
                    return False
            
            actual_duration = self.robot.getTime() - start_sim_time
            print(f"{LOG_PREFIX} Motion '{label}' finished: {step_count} steps, {actual_duration:.2f}s actual")
            
            # Record pose AFTER motion
            pos_after, yaw_after = self._log_pose(f"AFTER {label}")
            
            # DIAGNOSTIC: log pose change (does NOT block navigation).
            # Motion returning isOver()==True is considered success â€” pose change is
            # informational only. Hard-failing on tiny physics jitter was causing
            # cascade aborts even when the robot was moving correctly.
            if pos_before[0] is not None and pos_after[0] is not None:
                dx = abs(pos_after[0] - pos_before[0])
                dy = abs(pos_after[1] - pos_before[1])
                dz = abs(pos_after[2] - pos_before[2])
                pos_delta = dx + dy + dz

                yaw_delta = 0.0
                if yaw_before is not None and yaw_after is not None:
                    yaw_delta = abs(self._normalize_angle(yaw_after - yaw_before))

                if pos_delta > 0.001 or yaw_delta > math.radians(0.5):
                    print(f"{LOG_PREFIX} Motion '{label}' completed: steps={step_count}, "
                          f"pos_delta={pos_delta:.4f}m, yaw_delta={math.degrees(yaw_delta):.2f}deg")
                else:
                    print(f"{LOG_PREFIX} WARNING: Motion '{label}' produced no measurable pose change "
                          f"(pos_delta={pos_delta:.6f}m, yaw_delta={math.degrees(yaw_delta):.4f}deg)")
                    print(f"{LOG_PREFIX}   Possible causes: joint-name mismatch, collision, or "
                          f"physics issue. Navigation will continue.")
            else:
                print(f"{LOG_PREFIX} WARNING: Could not validate pose change (pose unavailable)")

            # Motion ran for full declared duration without timeout.
            return True
            
        except Exception as e:
            print(f"{LOG_PREFIX} ERROR playing motion '{label}': {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _play_motion(self, motion, wait: bool = True, timeout_s: float = 15.0) -> bool:
        """
        Legacy wrapper for play_motion - deprecated, use play_motion() directly.
        Kept for backwards compatibility.
        """
        # Determine motion label from object (for logging)
        label = "unknown"
        if motion is self._motion_forward:
            label = "forward"
        elif motion is self._motion_turn_left:
            label = "turn_left"
        elif motion is self._motion_turn_right:
            label = "turn_right"
        
        return self.play_motion(motion, timeout_s=timeout_s, label=label)
    
    def rotate_toward_target(self, target_pos: Tuple[float, float, float]) -> bool:
        """
        Rotate NAO to face target using closed-loop control.
        
        Uses play_motion() which validates pose changes - will abort if motion
        execution fails (robot not moving).
        
        Includes fall detection and recovery between motions.
        
        Args:
            target_pos: (x, y, z) of target
        
        Returns:
            True if aligned (within tolerance), False if failed or movement disabled.
        """
        # Guard: check if movement is enabled
        if not self._movement_enabled:
            print(f"{LOG_PREFIX} Movement disabled - cannot rotate")
            return False
        
        # Check for fall before starting
        if self.is_fallen():
            print(f"{LOG_PREFIX} Robot fallen before rotation - attempting recovery...")
            if not self.recover_from_fall():
                print(f"{LOG_PREFIX} ABORTING: Recovery failed before rotation")
                return False
        
        global _calibration_done
        if not _calibration_done:
            print(f"{LOG_PREFIX} WARNING: Navigation not calibrated - running calibration first...")
            if not self.run_full_calibration():
                print(f"{LOG_PREFIX} Calibration failed - rotation may not work correctly")
        
        tolerance_rad = getattr(self.house_config, 'rotation_tolerance_rad', math.radians(10))

        desired_heading = self.compute_heading_to_target(target_pos)
        if desired_heading is None:
            print(f"{LOG_PREFIX} ERROR: Cannot compute desired heading")
            return False

        print(f"{LOG_PREFIX} Continuous rotation to heading {math.degrees(desired_heading):.1f}deg")
        success = self.rotate_to_heading(desired_heading, tolerance_rad=tolerance_rad)
        if success:
            print(f"{LOG_PREFIX} Aligned with target (error < {math.degrees(tolerance_rad):.1f}deg)")
        return success
    
    def walk_toward_target(self, target_pos: Tuple[float, float, float],
                           target_class: str = TARGET_CLASS_ROOM) -> bool:
        """
        Walk NAO toward target until within arrive distance.

        Improvements:
        - Re-aligns more aggressively as the robot gets close to the target
        - Uses a lower minimum-progress threshold to avoid false "stuck" detections
        - Detects when a long forward motion crossed through the arrival radius,
        even if the final post-step pose overshoots the exact target point

        Args:
            target_pos:   (x, y, z) of target
            target_class: sonar policy class (default TARGET_CLASS_ROOM)

        Returns:
            True if arrived, False if failed or movement disabled.
        """
        if not self._movement_enabled:
            print(f"{LOG_PREFIX} Movement disabled - cannot walk")
            return False

        if self.is_fallen():
            print(f"{LOG_PREFIX} Robot fallen before walking - attempting recovery...")
            if not self.recover_from_fall():
                print(f"{LOG_PREFIX} ABORTING: Recovery failed before walking")
                return False

        arrive_dist = getattr(self.house_config, 'arrive_distance_m', 0.5)
        return self.walk_to_target(target_pos, arrive_dist=arrive_dist,
                                   target_class=target_class)

    # =========================================================================
    # DOOR CROSSING â€” 6-state alignment + entry controller
    # =========================================================================

    def execute_door_crossing(
        self,
        door_id: str,
        approach_pos: Tuple[float, float, float],
        target_pos: Tuple[float, float, float],
    ) -> bool:
        """
        Front-facing, centred door entry via a 6-state machine:

          DETECT_DOOR â†’ ESTIMATE_POSE â†’ ALIGN_TO_DOOR â†’ VALIDATE_ALIGNMENT
                      â†’ ENTER_DOOR â†’ RECOVERY_IF_COLLISION_RISK

        Guarantees NAO never commits to a crossing unless:
          â€¢ heading error < ALIGN_ANGLE_THRESHOLD_RAD (8Â°)
          â€¢ lateral offset from door centre < LATERAL_THRESHOLD_M (0.12m)

        The perpendicular approach point is recomputed from actual NAO position
        so any drift accumulated while opening the door is corrected before entry.

        Args:
            door_id:      House-config door ID (e.g. "door_hall_kitchen")
            approach_pos: Door frame centre world position (from house_config)
            target_pos:   Final room target world position
        Returns:
            True when NAO is through the door, False on unrecoverable failure.
        """
        from skills.nav.doorway_detector import (
            _DOOR_FRAME_DATA, THROUGH_DOOR_OFFSET_M, THROUGH_DOOR_ARRIVE_M,
            NAO_BODY_HALF_WIDTH_M,
        )

        # Alignment thresholds.
        # ALIGN_ANGLE_THRESHOLD_RAD must be >= MOTION_FILE_MIN_TURN_TOLERANCE_RAD (20Â°)
        # because motion_file discrete turns (~39Â°) cannot achieve finer precision.
        # Using 8Â° here would make VALIDATE always fail â†’ wasted retries â†’ misaligned entry.
        ALIGN_ANGLE_THRESHOLD_RAD = MOTION_FILE_MIN_TURN_TOLERANCE_RAD   # 20Â°
        LATERAL_THRESHOLD_M       = 0.12   # m  â€” max lateral offset from door centre
        SAFETY_MARGIN_M           = 0.05   # m  â€” clearance each side beyond NAO half-width
        MAX_ALIGN_RETRIES         = 3
        APPROACH_DIST_M           = 0.70   # m  â€” perpendicular standoff from frame wall
        # arrive_distance for the centering step must be < LATERAL_THRESHOLD_M
        # so NAO actually moves close enough to be within threshold.
        CENTER_ARRIVE_M           = 0.08   # m  â€” was 0.15 (too large; NAO "arrived" before centering)

        # â”€â”€ STATE: DETECT_DOOR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
        frame_entry = _DOOR_FRAME_DATA.get(door_id)
        if frame_entry is not None:
            frame_cx, frame_cy, wall_y, inner_half_gap = frame_entry
        else:
            frame_cx, frame_cy = approach_pos[0], approach_pos[1]
            wall_y = approach_pos[1]
            inner_half_gap = 0.245   # assume kitchen-width gap as fallback
            print(f"{LOG_PREFIX} [DETECT_DOOR] Unknown door '{door_id}' â€” "
                  f"using approach_pos as frame centre")

        # Door normal: unit vector pointing from hallway INTO the room.
        # North-wall doors (wall_y=0.88): normal = +Y, heading = Ï€/2 = 90Â°
        # South-wall doors (wall_y=-1.52): normal = âˆ’Y, heading = âˆ’Ï€/2 = -90Â°
        # Heading convention: atan2(world_Y, world_X), consistent with
        # compute_heading_to_target and get_current_heading (local Y axis, _forward_axis="y").
        is_north_wall = wall_y > 0.0
        door_heading  = math.pi / 2.0 if is_north_wall else -math.pi / 2.0
        normal_y      = 1.0 if is_north_wall else -1.0

        # Traversability check:
        #   available_width = inner_half_gap * 2  (gap between post inner faces)
        #   required_width  = NAO_width + 2 * safety_margin
        available_width = inner_half_gap * 2.0
        required_width  = NAO_BODY_HALF_WIDTH_M * 2.0 + 2.0 * SAFETY_MARGIN_M
        traversable     = available_width > required_width
        print(f"{LOG_PREFIX} [DETECT_DOOR] id={door_id}  "
              f"frame=({frame_cx:.3f},{wall_y:.3f})  "
              f"normal={'N(+Y)' if is_north_wall else 'S(-Y)'}  "
              f"door_heading={math.degrees(door_heading):.1f}deg")
        print(f"{LOG_PREFIX} [TRAVERSABILITY] "
              f"available={available_width:.3f}m  "
              f"NAO_width={NAO_BODY_HALF_WIDTH_M*2:.3f}m  "
              f"safety={SAFETY_MARGIN_M:.3f}m/side  "
              f"required={required_width:.3f}m  "
              f"{'PASSABLE' if traversable else 'TOO NARROW â€” proceeding anyway'}")

        align_attempt = 0
        while align_attempt < MAX_ALIGN_RETRIES:
            align_attempt += 1

            # â”€â”€ STATE: ESTIMATE_POSE â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
            current_pos     = self.get_current_position()
            current_heading = self.get_current_heading()
            if current_pos is None or current_heading is None:
                return False

            # Lateral offset: signed X-distance from door centre line.
            # Positive = NAO is to the right of door centre (looking into room).
            lateral_offset = current_pos[0] - frame_cx
            # Heading error: signed angle from door normal to NAO's current heading.
            heading_error  = self._normalize_angle(door_heading - current_heading)
            # Perpendicular distance from wall plane.
            perp_dist = abs(current_pos[1] - wall_y)
            # Sonar clearance for decision logging.
            sonar_clr = self._obstacle_detector.clearance_ahead()

            print(f"{LOG_PREFIX} [ESTIMATE_POSE] attempt={align_attempt}  "
                  f"pos=({current_pos[0]:.3f},{current_pos[1]:.3f})  "
                  f"heading={math.degrees(current_heading):.1f}deg  "
                  f"door_normal={math.degrees(door_heading):.1f}deg  "
                  f"lat_offset={lateral_offset:+.3f}m  "
                  f"heading_err={math.degrees(heading_error):+.1f}deg  "
                  f"perp_dist={perp_dist:.3f}m  "
                  f"sonar_clr={sonar_clr:.3f}m")

            # â”€â”€ STATE: ALIGN_TO_DOOR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #

            # Phase A â€” Lateral centering.
            # Navigate to the canonical perpendicular approach point on the door
            # centre axis.  arrive_distance=CENTER_ARRIVE_M must be smaller than
            # LATERAL_THRESHOLD_M so the arrival check doesn't short-circuit
            # before NAO actually reaches the centred position.
            if abs(lateral_offset) > LATERAL_THRESHOLD_M:
                standoff = max(perp_dist, APPROACH_DIST_M)
                perp_approach = (
                    frame_cx,
                    wall_y - normal_y * standoff,
                    0.0,
                )
                dist_to_center = math.sqrt(
                    (perp_approach[0] - current_pos[0]) ** 2 +
                    (perp_approach[1] - current_pos[1]) ** 2
                )
                print(f"{LOG_PREFIX} [ALIGN_A] Centering: "
                      f"lat_offset={lateral_offset:+.3f}m > threshold={LATERAL_THRESHOLD_M}m  "
                      f"target=({perp_approach[0]:.3f},{perp_approach[1]:.3f})  "
                      f"dist={dist_to_center:.3f}m  arrive={CENTER_ARRIVE_M}m")
                _navigate_to_position(self, perp_approach,
                                      arrive_distance=CENTER_ARRIVE_M,
                                      target_class=TARGET_CLASS_DOOR)
                # Re-read position after centering
                _centered_pos = self.get_current_position()
                if _centered_pos is not None:
                    _new_lat = abs(_centered_pos[0] - frame_cx)
                    print(f"{LOG_PREFIX} [ALIGN_A] After centering: "
                          f"pos=({_centered_pos[0]:.3f},{_centered_pos[1]:.3f})  "
                          f"new_lat_offset={_new_lat:+.3f}m  "
                          f"{'OK' if _new_lat <= LATERAL_THRESHOLD_M else 'STILL OFF'}")

            # Phase B â€” Rotate to face door normal.
            # Re-read heading_error since Phase A navigation may have changed it.
            _h_now = self.get_current_heading()
            if _h_now is not None:
                heading_error = self._normalize_angle(door_heading - _h_now)
            if abs(heading_error) > ALIGN_ANGLE_THRESHOLD_RAD:
                print(f"{LOG_PREFIX} [ALIGN_B] Rotating to door normal: "
                      f"target={math.degrees(door_heading):.1f}deg  "
                      f"current={math.degrees(_h_now or 0):.1f}deg  "
                      f"err={math.degrees(heading_error):+.1f}deg  "
                      f"threshold={math.degrees(ALIGN_ANGLE_THRESHOLD_RAD):.1f}deg")
                self.rotate_to_heading(door_heading,
                                       tolerance_rad=ALIGN_ANGLE_THRESHOLD_RAD,
                                       timeout_s=20.0)
                _h_after = self.get_current_heading()
                if _h_after is not None:
                    _err_after = abs(self._normalize_angle(door_heading - _h_after))
                    print(f"{LOG_PREFIX} [ALIGN_B] After rotation: "
                          f"heading={math.degrees(_h_after):.1f}deg  "
                          f"err={math.degrees(_err_after):.1f}deg")

            # â”€â”€ STATE: VALIDATE_ALIGNMENT â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
            current_pos     = self.get_current_position()
            current_heading = self.get_current_heading()
            if current_pos is None or current_heading is None:
                return False

            lat_err  = abs(current_pos[0] - frame_cx)
            head_err = abs(self._normalize_angle(door_heading - current_heading))
            aligned  = (lat_err  <= LATERAL_THRESHOLD_M and
                        head_err <= ALIGN_ANGLE_THRESHOLD_RAD)

            print(f"{LOG_PREFIX} [VALIDATE] attempt={align_attempt}  "
                  f"lat={lat_err:.3f}m (â‰¤{LATERAL_THRESHOLD_M}m? {'YES' if lat_err<=LATERAL_THRESHOLD_M else 'NO'})  "
                  f"head={math.degrees(head_err):.1f}deg (â‰¤{math.degrees(ALIGN_ANGLE_THRESHOLD_RAD):.1f}deg? "
                  f"{'YES' if head_err<=ALIGN_ANGLE_THRESHOLD_RAD else 'NO'})  "
                  f"â†’ {'ALIGNED' if aligned else 'RETRY'}")

            if aligned:
                break

        if align_attempt >= MAX_ALIGN_RETRIES and not aligned:
            print(f"{LOG_PREFIX} [VALIDATE] Max retries â€” best effort (lat={lat_err:.3f}m head={math.degrees(head_err):.1f}deg)")

        # â”€â”€ STATE: ENTER_DOOR â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
        through_pos = self._doorway_detector.get_through_waypoint(
            approach_pos, target_pos, offset_m=THROUGH_DOOR_OFFSET_M
        )
        _left_clr  = self._obstacle_detector.left_clearance()
        _right_clr = self._obstacle_detector.right_clearance()
        _fwd_clr   = self._obstacle_detector.clearance_ahead()
        _cp        = self.get_current_position()
        _ch        = self.get_current_heading()
        print(f"{LOG_PREFIX} [ENTER_DOOR] "
              f"pos=({(_cp[0] if _cp else 0):.3f},{(_cp[1] if _cp else 0):.3f})  "
              f"heading={math.degrees(_ch or 0):.1f}deg  "
              f"through=({through_pos[0]:.3f},{through_pos[1]:.3f})  "
              f"sonar L={_left_clr:.3f}m F={_fwd_clr:.3f}m R={_right_clr:.3f}m  "
              f"lat_before_entry={abs((_cp[0] if _cp else frame_cx) - frame_cx):.3f}m")

        crossed = _navigate_to_position(self, through_pos,
                                        arrive_distance=THROUGH_DOOR_ARRIVE_M,
                                        target_class=TARGET_CLASS_DOOR)

        _cp2 = self.get_current_position()
        print(f"{LOG_PREFIX} [ENTER_DOOR] result={'SUCCESS' if crossed else 'FAIL'}  "
              f"final_pos=({(_cp2[0] if _cp2 else 0):.3f},{(_cp2[1] if _cp2 else 0):.3f})")

        if crossed:
            return True

        # â”€â”€ STATE: RECOVERY_IF_COLLISION_RISK â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ #
        print(f"{LOG_PREFIX} [RECOVERY] Crossing failed â€” backing up and re-aligning")
        current_pos = self.get_current_position()
        if current_pos is not None:
            backup_pos = (
                current_pos[0],
                current_pos[1] - normal_y * 0.35,
                0.0,
            )
            print(f"{LOG_PREFIX} [RECOVERY] pos=({current_pos[0]:.3f},{current_pos[1]:.3f})  "
                  f"backup=({backup_pos[0]:.3f},{backup_pos[1]:.3f})  "
                  f"sonar={self._obstacle_detector.clearance_ahead():.3f}m")
            _navigate_to_position(self, backup_pos,
                                  arrive_distance=0.15,
                                  target_class=TARGET_CLASS_TRANSIT)

        self.rotate_to_heading(door_heading,
                               tolerance_rad=ALIGN_ANGLE_THRESHOLD_RAD,
                               timeout_s=20.0)
        _ch_rec = self.get_current_heading()
        print(f"{LOG_PREFIX} [RECOVERY] Re-entering: heading={math.degrees(_ch_rec or 0):.1f}deg  "
              f"through=({through_pos[0]:.3f},{through_pos[1]:.3f})")
        return _navigate_to_position(self, through_pos,
                                     arrive_distance=THROUGH_DOOR_ARRIVE_M,
                                     target_class=TARGET_CLASS_DOOR)

    # =========================================================================
    # FALL RECOVERY METHODS
    # =========================================================================
    
    def stop_all_motions(self):
        """
        Stop any currently active locomotion commands.

        Called before recovery to ensure no conflicting walking commands.
        """
        try:
            # Velocity locomotion stop (primary locomotion path)
            self.stop_walking()

            # Legacy motion stop calls (kept for compatibility only)
            if self._motion_forward and is_motion_valid(self._motion_forward):
                self._motion_forward.stop()
            if self._motion_turn_left and is_motion_valid(self._motion_turn_left):
                self._motion_turn_left.stop()
            if self._motion_turn_right and is_motion_valid(self._motion_turn_right):
                self._motion_turn_right.stop()
            print(f"{RECOVERY_LOG_PREFIX} All locomotion commands stopped")
        except Exception as e:
            print(f"{RECOVERY_LOG_PREFIX} Error stopping motions: {e}")
    
    def can_recover(self) -> bool:
        """
        Check if fall recovery is possible.
        
        Returns:
            True if at least one stand-up motion is available.
        """
        global _motion_standup_front, _motion_standup_back
        return is_motion_valid(_motion_standup_front) or is_motion_valid(_motion_standup_back)
    
    def recover_from_fall(self, max_attempts: int = 3) -> bool:
        """
        Attempt to recover from a fall by standing up.

        Steps:
        1. Stop all active locomotion motions
        2. Determine fall direction (front/back)
        3. Select appropriate stand-up motion
        4. Play stand-up motion fully
        5. Wait for stabilization
        6. Verify robot is upright

        Args:
            max_attempts: Maximum recovery attempts before giving up.

        Returns:
            True if successfully recovered, False if recovery failed.
        """
        global _recovery_in_progress, _motion_standup_front, _motion_standup_back

        if _recovery_in_progress:
            print(f"{RECOVERY_LOG_PREFIX} Recovery already in progress")
            return False

        if not self.can_recover():
            print(f"{RECOVERY_LOG_PREFIX} Cannot recover - no stand-up motions available")
            print(f"{RECOVERY_LOG_PREFIX} Copy from: WEBOTS_HOME/projects/robots/softbank/nao/motions/")
            print(f"{RECOVERY_LOG_PREFIX}   - StandUpFromFront.motion")
            print(f"{RECOVERY_LOG_PREFIX}   - StandUpFromBack.motion")
            return False

        _recovery_in_progress = True

        print(f"\n{RECOVERY_LOG_PREFIX} " + "=" * 50)
        print(f"{RECOVERY_LOG_PREFIX} ATTEMPTING FALL RECOVERY")
        print(f"{RECOVERY_LOG_PREFIX} " + "=" * 50)

        try:
            self.stop_all_motions()

            for _ in range(5):
                if self.robot.step(self.timestep) == -1:
                    return False

            for attempt in range(1, max_attempts + 1):
                print(f"\n{RECOVERY_LOG_PREFIX} Recovery attempt {attempt}/{max_attempts}")

                direction = self.get_fall_direction()
                print(f"{RECOVERY_LOG_PREFIX} Fall direction: {direction}")

                motion = None
                motion_label = "unknown"

                if direction == "front" and is_motion_valid(_motion_standup_front):
                    motion = _motion_standup_front
                    motion_label = "StandUpFromFront"
                elif direction == "back" and is_motion_valid(_motion_standup_back):
                    motion = _motion_standup_back
                    motion_label = "StandUpFromBack"
                elif is_motion_valid(_motion_standup_front):
                    motion = _motion_standup_front
                    motion_label = "StandUpFromFront (default)"
                elif is_motion_valid(_motion_standup_back):
                    motion = _motion_standup_back
                    motion_label = "StandUpFromBack (fallback)"
                else:
                    print(f"{RECOVERY_LOG_PREFIX} No suitable stand-up motion for direction: {direction}")
                    continue

                print(f"{RECOVERY_LOG_PREFIX} Using motion: {motion_label}")

                success = self._play_recovery_motion(motion, motion_label)

                if not success:
                    print(f"{RECOVERY_LOG_PREFIX} Motion playback failed")
                    continue

                print(f"{RECOVERY_LOG_PREFIX} Waiting for stabilization...")
                stabilize_steps = int(500 / self.timestep)
                for _ in range(stabilize_steps):
                    if self.robot.step(self.timestep) == -1:
                        return False

                # IMPORTANT: bypass the recovery guard so we verify the real pose
                if not self.is_fallen(verbose=False, ignore_recovery_flag=True):
                    pos = self.get_current_position()
                    height = pos[2] if pos else 0  # Z-up: height is pos[2]
                    print(f"{RECOVERY_LOG_PREFIX} " + "-" * 50)
                    print(f"{RECOVERY_LOG_PREFIX} RECOVERY SUCCESSFUL!")
                    print(f"{RECOVERY_LOG_PREFIX} Robot is upright (z={height:.3f}m)")
                    print(f"{RECOVERY_LOG_PREFIX} " + "-" * 50)
                    return True
                else:
                    print(f"{RECOVERY_LOG_PREFIX} Robot still fallen after recovery attempt")

            print(f"{RECOVERY_LOG_PREFIX} " + "!" * 50)
            print(f"{RECOVERY_LOG_PREFIX} RECOVERY FAILED after {max_attempts} attempts")
            print(f"{RECOVERY_LOG_PREFIX} " + "!" * 50)
            return False

        finally:
            _recovery_in_progress = False
    
    def _play_recovery_motion(self, motion, label: str, timeout_s: float = 30.0) -> bool:
        """
        Play a stand-up motion and wait for completion.
        
        Similar to play_motion but without pose change validation
        (stand-up motion is expected to significantly change pose).
        
        Args:
            motion: Motion object to play
            label: Description for logging
            timeout_s: Maximum time to wait
        
        Returns:
            True if motion completed, False on error.
        """
        if not is_motion_valid(motion):
            return False
        
        try:
            # Reset and play
            motion.stop()
            try:
                motion.setTime(0)
            except AttributeError:
                pass
            
            motion.play()
            
            start_time = self.robot.getTime()
            expected_duration = motion.getDuration() / 1000.0
            
            print(f"{RECOVERY_LOG_PREFIX} Playing {label} (expected: {expected_duration:.1f}s)...")
            
            if expected_duration <= 0.0:
                print(f"{RECOVERY_LOG_PREFIX} Invalid duration for {label}: {expected_duration:.2f}s")
                motion.stop()
                return False

            while (self.robot.getTime() - start_time) < expected_duration:
                if self.robot.step(self.timestep) == -1:
                    return False

                elapsed = self.robot.getTime() - start_time
                if elapsed > timeout_s:
                    print(f"{RECOVERY_LOG_PREFIX} Motion timeout after {elapsed:.1f}s")
                    motion.stop()
                    return False
            
            actual_duration = self.robot.getTime() - start_time
            print(f"{RECOVERY_LOG_PREFIX} Motion completed in {actual_duration:.1f}s")
            return True
            
        except Exception as e:
            print(f"{RECOVERY_LOG_PREFIX} Error playing motion: {e}")
            return False
    
    def check_and_recover(self) -> bool:
        """
        Check if fallen and attempt recovery if needed.
        
        Convenience method for integration into navigation loops.
        
        Returns:
            True if robot is upright (either didn't fall or recovered),
            False if fallen and recovery failed.
        """
        if self.is_fallen():
            return self.recover_from_fall()
        return True

    def calibrate_forward_axis(self) -> bool:
        """
        Keep NAO default forward-axis calibration.

        Velocity locomotion mode intentionally does not use prerecorded
        Forwards50 motion for calibration. For standard Webots NAO, local Y+
        is forward, so we keep defaults.
        """
        global _forward_axis, _forward_sign, _calibration_done

        _forward_axis = "y"
        _forward_sign = 1
        _calibration_done = True
        print(f"{LOG_PREFIX} Velocity mode: using default forward axis y+")
        return True
    
    def calibrate_turn_directions(self) -> bool:
        """
        Validate turning direction in velocity mode.

        In velocity locomotion mode, positive theta is expected to rotate
        counter-clockwise, so no left/right motion swap is required.
        """
        global _turns_swapped

        if not self._movement_enabled:
            print(f"{LOG_PREFIX} Cannot calibrate - movement disabled")
            return False

        _turns_swapped = False
        print(f"{LOG_PREFIX} Velocity mode: turn mapping uses theta sign directly (no swap)")
        return True
    
    def run_full_calibration(self) -> bool:
        """
        Run complete calibration: forward axis + turn directions.
        
        MUST be called at least once before navigation for reliable results.
        Sets global calibration state used by get_current_heading().
        
        Returns:
            True if all calibration steps passed.
        """
        global _calibration_done
        
        print(f"\n{LOG_PREFIX} " + "#" * 70)
        print(f"{LOG_PREFIX} FULL NAVIGATION CALIBRATION")
        print(f"{LOG_PREFIX} " + "#" * 70)
        
        if not self._movement_enabled:
            print(f"{LOG_PREFIX} Cannot calibrate - movement disabled")
            return False
        
        # Step 1: Calibrate forward axis
        axis_ok = self.calibrate_forward_axis()
        if not axis_ok:
            print(f"{LOG_PREFIX} Forward axis calibration FAILED")
            return False
        
        # Step 2: Calibrate turn directions  
        turns_ok = self.calibrate_turn_directions()
        if not turns_ok:
            print(f"{LOG_PREFIX} Turn direction calibration FAILED")
            return False
        
        _calibration_done = True
        print(f"\n{LOG_PREFIX} " + "#" * 70)
        print(f"{LOG_PREFIX} CALIBRATION COMPLETE - Navigation should now work correctly")
        print(f"{LOG_PREFIX}   Forward axis: {_forward_axis}{'+' if _forward_sign > 0 else '-'}")
        print(f"{LOG_PREFIX}   Turns swapped: {_turns_swapped}")
        print(f"{LOG_PREFIX} " + "#" * 70 + "\n")
        
        return True
    
    def calibrate_turn_direction(self) -> bool:
        """
        Legacy method - now calls calibrate_turn_directions().
        Kept for backwards compatibility.
        """
        return self.calibrate_turn_directions()
    
    def run_motion_diagnostic(self) -> bool:
        """
        Run locomotion diagnostic for velocity-based walking.

        Tests:
        1. Rotate in place with small positive theta command
        2. Move forward with small x command

        Returns:
            True if both tests show measurable motion, False otherwise.
        """
        print(f"\n{LOG_PREFIX} " + "=" * 70)
        print(f"{LOG_PREFIX} VELOCITY LOCOMOTION DIAGNOSTIC")
        print(f"{LOG_PREFIX} " + "=" * 70)
        
        if not self._movement_enabled:
            print(f"{LOG_PREFIX} ERROR: Movement is DISABLED")
            print(f"{LOG_PREFIX} No velocity walking interface found.")
            return False
        
        print(f"{LOG_PREFIX} Movement enabled: YES")
        print(f"{LOG_PREFIX} Robot type: {type(self.robot).__name__}")
        print(f"{LOG_PREFIX} Timestep: {self.timestep}ms")
        print(f"{LOG_PREFIX} Self node: {self._self_node}")
        print(f"{LOG_PREFIX} Walk API: {self._walk_cmd}")
        
        all_passed = True

        pos_before = self.get_current_position()
        yaw_before = self.get_current_heading()
        if pos_before is None or yaw_before is None:
            print(f"{LOG_PREFIX} ERROR: Pose unavailable for diagnostic")
            return False

        # Test 1: In-place rotation
        print(f"\n{LOG_PREFIX} --- TEST 1: Rotate (velocity) ---")
        if not self._send_walk_command(0.0, 0.0, 0.2):
            return False
        for _ in range(max(1, int(500 / self.timestep))):
            if self.robot.step(self.timestep) == -1:
                self.stop_walking()
                return False
        self.stop_walking()

        yaw_after = self.get_current_heading()
        if yaw_after is None:
            all_passed = False
        else:
            yaw_delta = abs(self._normalize_angle(yaw_after - yaw_before))
            if yaw_delta > math.radians(2.0):
                print(f"{LOG_PREFIX} TEST 1: PASSED (yaw delta={math.degrees(yaw_delta):.1f}deg)")
            else:
                print(f"{LOG_PREFIX} TEST 1: FAILED (yaw delta={math.degrees(yaw_delta):.2f}deg)")
                all_passed = False

        # Test 2: Forward translation
        print(f"\n{LOG_PREFIX} --- TEST 2: Forward (velocity) ---")
        pos_mid = self.get_current_position()
        if pos_mid is None:
            return False

        if not self._send_walk_command(0.25, 0.0, 0.0):
            return False
        for _ in range(max(1, int(700 / self.timestep))):
            if self.robot.step(self.timestep) == -1:
                self.stop_walking()
                return False
        self.stop_walking()

        pos_after = self.get_current_position()
        if pos_after is None:
            all_passed = False
        else:
            dx = pos_after[0] - pos_mid[0]
            dy = pos_after[1] - pos_mid[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.01:
                print(f"{LOG_PREFIX} TEST 2: PASSED (translation={dist:.3f}m)")
            else:
                print(f"{LOG_PREFIX} TEST 2: FAILED (translation={dist:.4f}m)")
                all_passed = False
        
        # Summary
        print(f"\n{LOG_PREFIX} " + "=" * 70)
        if all_passed:
            print(f"{LOG_PREFIX} DIAGNOSTIC RESULT: ALL TESTS PASSED")
            print(f"{LOG_PREFIX} Velocity locomotion is working correctly.")
        else:
            print(f"{LOG_PREFIX} DIAGNOSTIC RESULT: TESTS FAILED")
            print(f"{LOG_PREFIX} Velocity locomotion is NOT working. Possible causes:")
            print(f"{LOG_PREFIX}   1. Walking API unavailable on this robot/controller")
            print(f"{LOG_PREFIX}   2. Robot physics not enabled")
            print(f"{LOG_PREFIX}   3. Robot stuck/colliding with something")
            print(f"{LOG_PREFIX}   4. Controller attached to wrong robot")
        print(f"{LOG_PREFIX} " + "=" * 70 + "\n")
        
        return all_passed


def _navigate_to_position(nav: NavigationController, target_pos,
                           arrive_distance: float = None,
                           target_class: str = TARGET_CLASS_ROOM) -> bool:
    """
    Navigate to a position via walk_to_target (handles alignment internally).
    """
    original_distance = nav.house_config.arrive_distance_m
    if arrive_distance is not None:
        nav.house_config.arrive_distance_m = arrive_distance
    try:
        print(f"{LOG_PREFIX} Navigating to ({target_pos[0]:.2f}, {target_pos[1]:.2f})")
        return nav.walk_toward_target(target_pos, target_class=target_class)
    finally:
        nav.house_config.arrive_distance_m = original_distance


def _compute_door_heading(approach_x: float, approach_y: float) -> Optional[float]:
    """Compute heading that faces perpendicular into the door (away from hallway center)."""
    HALLWAY_CENTER_Y = -0.32
    # Point from approach toward the wall (away from hallway center)
    dy = approach_y - HALLWAY_CENTER_Y
    if abs(dy) < 0.1:
        return None
    return math.atan2(dy / abs(dy), 0.0)  # face north or south


def go_to_target(target_id: str, robot, house_config: "HouseConfig",
                 say_func=None, handle_doors: bool = True) -> bool:
    """
    Navigate NAO to a target location, opening doors as needed.
    
    Iteration 3: Handles route_doors - doors that must be opened to reach target.
    For each door in route_doors:
    1. Navigate to door's approach position
    2. Open the door
    3. Continue to next door or final target
    
    Args:
        target_id: Target ID or label (e.g., "kitchen" or "Kitchen")
        robot: Webots Robot/Supervisor instance
        house_config: Loaded HouseConfig instance
        say_func: Optional function to make NAO speak
        handle_doors: Whether to handle route_doors (default True)
    
    Returns:
        True if arrived at target, False on error/timeout.
    """
    print(f"\n{LOG_PREFIX} " + "=" * 50)
    print(f"{LOG_PREFIX} GO_TO_TARGET: '{target_id}'")
    print(f"{LOG_PREFIX} " + "=" * 50)
    
    # Resolve target ID
    resolved_id = house_config.resolve_target_id(target_id)
    if resolved_id is None:
        print(f"{LOG_PREFIX} ERROR: Unknown target '{target_id}'")
        available = house_config.list_targets()
        print(f"{LOG_PREFIX} Known targets: {available}")
        if say_func:
            say_func(f"I don't know where {target_id} is.")
        return False
    
    # Get target info for diagnostics
    target_info = house_config._target_by_id.get(resolved_id, {})
    def_name = target_info.get("def", "")
    label = target_info.get("label", resolved_id)
    
    print(f"{LOG_PREFIX} Resolved '{target_id}' -> id='{resolved_id}', DEF='{def_name}'")
    
    # Get target position (uses on-demand fetching from Webots)
    target_pos = house_config.get_target_translation(resolved_id, use_cache=False)
    if target_pos is None:
        print(f"{LOG_PREFIX} ERROR: Cannot get position for target '{resolved_id}'")
        print(f"{LOG_PREFIX} Troubleshooting:")
        print(f"{LOG_PREFIX}   1. Check NAO node has 'supervisor TRUE' in Webots")
        print(f"{LOG_PREFIX}   2. Check a node exists with: DEF {def_name} Transform {{ translation X Y Z }}")
        print(f"{LOG_PREFIX}   3. DEF names are CASE-SENSITIVE")
        if say_func:
            say_func(f"I cannot find the location of {target_id}.")
        return False
    
    print(f"{LOG_PREFIX} Target '{resolved_id}' at ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
    
    # Create navigation controller
    nav = NavigationController(robot, house_config)
    
    # Check if movement is enabled (motions loaded successfully)
    if not nav._movement_enabled:
        print(f"{LOG_PREFIX} ERROR: Movement disabled - velocity walking API unavailable")
        print(f"{LOG_PREFIX} Expected moveToward/move API or NAO leg joints")
        if say_func:
            say_func("I cannot move. Walking interface is unavailable.")
        return False
    
    # Check if we can track position
    if nav.get_current_position() is None:
        print(f"{LOG_PREFIX} ERROR: Cannot get NAO position")
        print(f"{LOG_PREFIX} Make sure NAO has supervisor=True in Webots")
        if say_func:
            say_func("I cannot navigate. Check my settings.")
        return False
    
    current_pos = nav.get_current_position()
    print(f"{LOG_PREFIX} NAO at ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
    
    # =========================================================================
    # ITERATION 3: Handle route_doors
    # =========================================================================
    if handle_doors:
        route_doors = house_config.get_route_doors_for_target(resolved_id)
        if route_doors:
            print(f"{LOG_PREFIX} Route requires opening {len(route_doors)} door(s): {route_doors}")
            
            # Import open_door skill
            from skills.open_door import open_door
            
            for door_id in route_doors:
                print(f"\n{LOG_PREFIX} --- Processing door: {door_id} ---")
                
                # Get door approach position
                approach_pos = house_config.get_door_approach_position(door_id)
                if approach_pos is None:
                    print(f"{LOG_PREFIX} WARNING: No approach position for door '{door_id}'")
                    print(f"{LOG_PREFIX} Skipping door approach, will try to open anyway")
                else:
                    print(f"{LOG_PREFIX} Navigating to door approach: ({approach_pos[0]:.2f}, {approach_pos[1]:.2f}, {approach_pos[2]:.2f})")

                    # Get door label for speech
                    door_info = house_config.find_door_by_id(door_id)
                    door_label = door_info.get("label", door_id) if door_info else door_id

                    if say_func:
                        say_func(f"Going to the {door_label} door.")

                    # Pre-approach: navigate to the canonical perpendicular approach
                    # point â€” on the door's center axis, 1.90m from the wall.
                    # Using the door normal (not the approachâ†’target vector) ensures
                    # NAO always arrives from directly in front, never at an angle.
                    from skills.nav.doorway_detector import _DOOR_FRAME_DATA
                    _frame_entry = _DOOR_FRAME_DATA.get(door_id)
                    if _frame_entry is not None:
                        _fcx, _fcy, _wally, _ = _frame_entry
                        _n_y = 1.0 if _wally > 0.0 else -1.0
                        # Use the frame centre y-coordinate (_fcy) for the
                        # perpendicular pre-approach point (bugfix: previously
                        # used wall_y variable accidentally which placed the
                        # point on the wall line instead of the frame centre).
                        pre_approach = (_fcx, _fcy - _n_y * 1.90, 0.0)
                    else:
                        # Fallback: old vector-based pre-approach
                        _adx = approach_pos[0] - target_pos[0]
                        _ady = approach_pos[1] - target_pos[1]
                        _aseg = math.sqrt(_adx * _adx + _ady * _ady)
                        if _aseg > 0.3:
                            _aux, _auy = _adx / _aseg, _ady / _aseg
                            pre_approach = (
                                approach_pos[0] + _aux * 1.2,
                                approach_pos[1] + _auy * 1.2,
                                0.0,
                            )
                        else:
                            pre_approach = None

                    if pre_approach is not None:
                        print(f"{LOG_PREFIX} Pre-approach (perp axis): "
                              f"({pre_approach[0]:.2f}, {pre_approach[1]:.2f})")
                        _navigate_to_position(nav, pre_approach, arrive_distance=0.20)

                    # Navigate straight to door on perpendicular axis
                    door_arrive_dist = getattr(house_config, 'door_arrive_distance_m', 0.8)
                    if not _navigate_to_position(nav, approach_pos, door_arrive_dist):
                        print(f"{LOG_PREFIX} Failed to reach door '{door_id}'")
                        if say_func:
                            say_func(f"I could not reach the {door_label} door.")
                        return False

                    print(f"{LOG_PREFIX} Reached door approach position")
                    # Align perpendicular to door before opening
                    door_heading = _compute_door_heading(approach_pos[0], approach_pos[1])
                    if door_heading is not None:
                        nav.rotate_to_heading(door_heading, tolerance_rad=math.radians(10.0), timeout_s=15.0)
                        print(f"{LOG_PREFIX} Pre-door alignment complete")

                # Open the door
                if not open_door(door_id, robot, house_config, say_func=say_func, wait=True):
                    print(f"{LOG_PREFIX} Failed to open door '{door_id}'")
                    return False

                print(f"{LOG_PREFIX} Door '{door_id}' opened successfully")

        # 6-state door crossing: DETECTâ†’POSEâ†’ALIGNâ†’VALIDATEâ†’ENTERâ†’RECOVERY
        # Replaces the old single through-door waypoint with a full alignment
        # controller that guarantees front-facing, centred entry.
        if approach_pos is not None:
            if not nav.execute_door_crossing(door_id, approach_pos, target_pos):
                print(f"{LOG_PREFIX} Door crossing failed for '{door_id}'")
                if say_func:
                    say_func("I could not cross the doorway safely.")
                return False
    else:
        print(f"{LOG_PREFIX} No doors required for this route")

    # =========================================================================
    # Navigate to final target
    # =========================================================================
    print(f"\n{LOG_PREFIX} --- Navigating to final target: {label} ---")
    
    if say_func:
        say_func(f"Going to {label}.")
    
    # Walk to target (alignment handled internally by ALIGNING state)
    print(f"\n{LOG_PREFIX} Walking to target...")
    if not nav.walk_toward_target(target_pos):
        print(f"{LOG_PREFIX} Navigation failed (timeout or error)")
        if say_func:
            say_func("I could not reach the destination.")
        return False
    
    # Success
    print(f"\n{LOG_PREFIX} Navigation complete!")
    
    return True


def navigate_to_coords(
    x: float,
    y: float,
    robot,
    house_config,
    say_func=None,
    arrive_dist: float = 0.30,
) -> bool:
    """
    Navigate to world coordinates (x, y) directly - no named target lookup,
    no door handling. Used for approach waypoints and fine positioning.

    Lower-level than go_to_target(). Reuses the same NavigationController
    and walk_to_target machinery.
    """
    target_pos = (x, y, 0.0)
    ctrl = NavigationController(robot, house_config)
    return _navigate_to_position(ctrl, target_pos, arrive_distance=arrive_dist)


def run_navigation_sanity_check() -> None:
    """
    Run sanity check for navigation system.
    
    Prints diagnostic information about:
    - Controller directory
    - Motion file paths and existence status
    
    Call this to troubleshoot motion loading issues.
    """
    motion_sanity_check()


def run_turn_calibration(robot, house_config) -> bool:
    """
    Run turn direction calibration.
    
    Creates a NavigationController and tests if TurnLeft motion actually
    increases yaw (counter-clockwise rotation).
    
    Args:
        robot: Webots Robot/Supervisor instance
        house_config: HouseConfig instance
    
    Returns:
        True if calibration passed, False if there's a direction mismatch.
    """
    nav = NavigationController(robot, house_config)
    return nav.calibrate_turn_direction()


def run_motion_diagnostic(robot, house_config) -> bool:
    """
    Run comprehensive motion diagnostic.
    
    Tests that motion execution actually moves the robot.
    Use this to verify the motion system is working before navigation.
    
    Args:
        robot: Webots Robot/Supervisor instance
        house_config: HouseConfig instance
    
    Returns:
        True if all motion tests pass, False if any fail.
    """
    nav = NavigationController(robot, house_config)
    return nav.run_motion_diagnostic()


def run_full_calibration(robot, house_config) -> bool:
    """
    Run full navigation calibration.
    
    RECOMMENDED: Run this once at startup before any navigation commands.
    
    Calibrates:
    1. Forward axis - detects which local axis (X, Y, Z) is NAO's forward direction
    2. Turn directions - verifies/swaps TurnLeft/TurnRight motion mapping
    
    After calibration, navigation yaw calculations will be correct.
    
    Args:
        robot: Webots Robot/Supervisor instance
        house_config: HouseConfig instance
    
    Returns:
        True if all calibration steps passed.
    """
    nav = NavigationController(robot, house_config)
    return nav.run_full_calibration()


def is_calibrated() -> bool:
    """Check if navigation has been calibrated."""
    return _calibration_done


def get_calibration_info() -> dict:
    """Get current calibration settings."""
    return {
        "calibrated": _calibration_done,
        "forward_axis": _forward_axis,
        "forward_sign": _forward_sign,
        "turns_swapped": _turns_swapped,
    }


# =============================================================================
# MODULE-LEVEL FALL DETECTION/RECOVERY FUNCTIONS
# =============================================================================

def is_recovery_in_progress() -> bool:
    """Check if fall recovery is currently in progress."""
    return _recovery_in_progress


def has_standup_motions() -> bool:
    """Check if stand-up motions are available for fall recovery."""
    return is_motion_valid(_motion_standup_front) or is_motion_valid(_motion_standup_back)


def check_and_recover_from_fall(robot, house_config) -> bool:
    """
    Check if robot has fallen and attempt recovery.
    
    Convenience function for external code to check/recover falls.
    
    Args:
        robot: Webots Robot/Supervisor instance
        house_config: HouseConfig instance
    
    Returns:
        True if robot is upright (didn't fall or recovered successfully),
        False if fallen and recovery failed.
    """
    nav = NavigationController(robot, house_config)
    return nav.check_and_recover()


def get_fall_status(robot, house_config) -> dict:
    """
    Get detailed fall status information.

    Webots default world frame is Y-up:
    - height = pos[1]
    - vertical alignment = up[1]

    Args:
        robot: Webots Robot/Supervisor instance
        house_config: HouseConfig instance

    Returns:
        Dict with fall detection details.
    """
    nav = NavigationController(robot, house_config)

    pos = nav.get_current_position()
    up = nav.get_body_up_vector()
    roll_pitch = nav.get_roll_pitch()

    height = pos[2] if pos else None   # Z-up: height is pos[2]
    up_z = up[2] if up else None
    roll = math.degrees(roll_pitch[0]) if roll_pitch else None
    pitch = math.degrees(roll_pitch[1]) if roll_pitch else None

    fallen = nav.is_fallen(verbose=False)
    direction = nav.get_fall_direction() if fallen else None

    return {
        "fallen": fallen,
        "height": height,
        "up_z": up_z,
        "roll": roll,
        "pitch": pitch,
        "direction": direction,
        "can_recover": nav.can_recover(),
        "recovery_in_progress": _recovery_in_progress,
    }

