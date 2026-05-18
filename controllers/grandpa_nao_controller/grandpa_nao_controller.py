"""
GRANDPA_NAO Controller — Passive Elderly Presence Simulation
Webots NAO robot representing an elderly person.

ARCHITECTURE:
    Grandpa is PASSIVE. He never initiates voice requests or autonomous movement.
    All state changes are driven by external signals from SERVICE_NAO via Receiver.

    Flow:
        User speaks → SERVICE_NAO interprets → SERVICE_NAO signals Grandpa
        Grandpa reacts by entering the appropriate state.

State machine:
    RESTING (default, indefinite) ──[signal: start_walk]──► WALKING_SLOW
    WALKING_SLOW ──(energy depleted)──► TIRED
    TIRED / any state ──[signal: assist]──► WAITING_FOR_ASSISTANCE
    WAITING_FOR_ASSISTANCE ──(SERVICE_NAO arrives)──► BEING_ASSISTED
    BEING_ASSISTED ──(destination reached)──► RESTING

    Any state ──[signal: stop]──► RESTING
    EMERGENCY_FALL: placeholder (not fully implemented)

Signal protocol (SERVICE_NAO → Grandpa via Emitter, channel 2):
    "start_walk"   — begin slow walking episode
    "stop"         — return to RESTING immediately
    "assist"       — enter WAITING_FOR_ASSISTANCE (SERVICE_NAO will approach)
    "arrived"      — SERVICE_NAO confirms destination reached → RESTING

Communication:
    Receiver channel 2: incoming signals from SERVICE_NAO
    Emitter channel 1:  reserved for future use (Grandpa does NOT send voice requests)
"""

import math
import os
import random
from enum import Enum, auto
from controller import Motion, Supervisor

# ==============================================================================
# CONSTANTS
# ==============================================================================
TIMESTEP_MS = 32
LOG_PREFIX  = "[GRANDPA]"

# Walking speed — 75% of normal; fast enough for gait stability
GRANDPA_FWD_MAX = 0.45

# Gait shape — mirrors go_to_target for consistency
GAIT_BASE_HIP_PITCH = -0.15
GAIT_BASE_KNEE      =  0.30
GAIT_MAX_HIP_SWING  =  0.22
GAIT_MAX_TURN_SPLIT =  0.12

# Energy model — time-based, only active during WALKING_SLOW
ENERGY_MAX              = 100.0
ENERGY_TIRED_THRESH     =  20.0   # walking stops when energy drops here
ENERGY_DRAIN_PER_S      =   4.0   # units/second drained while walking (~25s full walk)
ENERGY_RESTORE_PER_S    =   1.0   # units/second restored while resting (informational)

# Walking pauses (keep subtle realism)
PAUSE_MIN_S             =   1.5
PAUSE_MAX_S             =   4.0
PAUSE_PROBABILITY       =   0.003  # per 32ms step

# Proximity thresholds
ASSISTANCE_PROXIMITY_M  =   1.2   # SERVICE_NAO within this → BEING_ASSISTED
ARRIVED_DISTANCE_M      =   0.8   # SERVICE_NAO stopped here → destination reached

# Channels
GRANDPA_CHANNEL = 1   # reserved / future
SERVICE_CHANNEL = 2   # incoming signals from SERVICE_NAO

# Startup
INIT_DELAY_S      =  2.0   # hold standing pose before any state runs
STAND_HIP_PITCH   = -0.45
STAND_KNEE_PITCH  =  0.70
STAND_ANKLE_PITCH = -0.35
WALK_RAMP_S       =  1.5   # blend standing→gait base when walk starts

LEG_JOINT_NAMES = [
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
]


# ==============================================================================
# UTILITIES
# ==============================================================================
def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ==============================================================================
# STATE MACHINE
# ==============================================================================
class GrandpaState(Enum):
    RESTING                = auto()   # default — stands still indefinitely
    WALKING_SLOW           = auto()   # triggered by "start_walk" signal
    TIRED                  = auto()   # energy depleted; waits for external action
    WAITING_FOR_ASSISTANCE = auto()   # triggered by "assist" signal
    BEING_ASSISTED         = auto()   # SERVICE_NAO arrived; Grandpa follows
    EMERGENCY_FALL         = auto()   # placeholder — not fully implemented


# ==============================================================================
# GRANDPA CONTROLLER
# ==============================================================================
class GrandpaController:
    """
    Passive elderly NAO. Stands still until SERVICE_NAO sends a signal.
    No autonomous voice generation. No random state changes.
    """

    def __init__(self):
        self.robot      = Supervisor()
        self.timestep   = TIMESTEP_MS
        self._self_node = self.robot.getSelf()

        # State
        self.state            = GrandpaState.RESTING
        self.state_start_time = self.robot.getTime()

        # Energy
        self.energy = ENERGY_MAX

        # Walk bookkeeping
        self._in_pause        = False
        self._pause_until     = 0.0
        self._walk_ramp_start = None   # set on first gait call; reset each pause

        # Gait
        self._gait_motors = {}
        self._gait_phase  = 0.0
        self._gait_last_t = self.robot.getTime()
        self._movement_ok = self._init_gait()

        # Motion file (preferred; skipped if path non-ASCII)
        self._fwd_motion     = self._load_motion("Forwards50.motion")
        self._motion_playing = False

        # Startup stabilization
        self._initialized  = False
        self._init_start_t = None
        self._apply_stand_pose()   # stabilize before first step()

        # Devices
        self._emitter = self.robot.getDevice("emitter")
        if self._emitter:
            self._emitter.setChannel(GRANDPA_CHANNEL)

        self._receiver = self.robot.getDevice("receiver")
        if self._receiver:
            self._receiver.setChannel(SERVICE_CHANNEL)
            self._receiver.enable(self.timestep)

        print(f"{LOG_PREFIX} Initialized. state=RESTING  energy={self.energy:.0f}")
        print(f"{LOG_PREFIX} Movement: {'ENABLED' if self._movement_ok else 'DISABLED'}")
        print(f"{LOG_PREFIX} Passive mode — waiting for SERVICE_NAO signals")

    # --------------------------------------------------------------------------
    # MOTION / GAIT
    # --------------------------------------------------------------------------
    def _load_motion(self, filename: str):
        """Returns None if path has non-ASCII chars (Webots C library limitation on Windows)."""
        path = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', 'nao_assist_controller', 'motions', filename
        ))
        if not os.path.isfile(path):
            print(f"{LOG_PREFIX} Motion file not found: {filename}")
            return None
        if not path.isascii():
            print(f"{LOG_PREFIX} Motion path has non-ASCII chars — skipping (C library limitation)")
            return None
        try:
            m = Motion(path)
            print(f"{LOG_PREFIX} Motion loaded: {filename}")
            return m
        except Exception as e:
            print(f"{LOG_PREFIX} Motion unavailable ({filename}): {e}")
            return None

    def _init_gait(self) -> bool:
        motors = {}
        try:
            for name in LEG_JOINT_NAMES:
                m = self.robot.getDevice(name)
                if m is None:
                    return False
                motors[name] = m
        except Exception:
            return False
        for m in motors.values():
            try:
                m.setVelocity(4.0)
            except Exception:
                pass
        self._gait_motors = motors
        return True

    def _stop_motion(self):
        if self._fwd_motion and self._motion_playing:
            self._fwd_motion.stop()
            self._motion_playing = False

    def _apply_stand_pose(self):
        """Hold stable NAO upright stance. Called during init and RESTING."""
        self._set_pose(
            lhy=0.0, lhr=0.0, lhp=STAND_HIP_PITCH,
            lk=STAND_KNEE_PITCH, lap=STAND_ANKLE_PITCH, lar=0.0,
            rhy=0.0, rhr=0.0, rhp=STAND_HIP_PITCH,
            rk=STAND_KNEE_PITCH, rap=STAND_ANKLE_PITCH, rar=0.0,
        )

    def _apply_gait(self, x: float, theta: float):
        """
        Joint-space gait using SERVICE_NAO's proven formula (zero hip roll).
        Blends from standing pose → gait base over WALK_RAMP_S on each fresh start.
        _walk_ramp_start is None at walk start and after every pause so ramp
        always begins at 0 when gait actually fires.
        """
        now = self.robot.getTime()
        dt  = _clamp(now - self._gait_last_t, 0.0, 0.1)
        self._gait_last_t = now

        x     = _clamp(x,     -1.0, 1.0)
        theta = _clamp(theta, -1.0, 1.0)

        if self._walk_ramp_start is None:
            self._walk_ramp_start = now
        ramp = _clamp((now - self._walk_ramp_start) / WALK_RAMP_S, 0.0, 1.0)

        hip_base  = STAND_HIP_PITCH  + (GAIT_BASE_HIP_PITCH - STAND_HIP_PITCH)  * ramp
        knee_base = STAND_KNEE_PITCH + (GAIT_BASE_KNEE      - STAND_KNEE_PITCH) * ramp

        cmd_mag = min(1.0, abs(x) + 0.7 * abs(theta))
        freq_hz = 0.55 + 0.85 * cmd_mag
        self._gait_phase += 2.0 * math.pi * freq_hz * dt
        phase = self._gait_phase

        s_l   = math.sin(phase)
        s_r   = math.sin(phase + math.pi)
        shift = math.sin(phase + math.pi / 2.0)

        swing_amp  = _clamp(0.04 + 0.16 * abs(x) + 0.10 * abs(theta),
                            0.04, GAIT_MAX_HIP_SWING) * ramp
        roll_amp   = 0.0
        turn_split = _clamp(GAIT_MAX_TURN_SPLIT * theta,
                            -GAIT_MAX_TURN_SPLIT, GAIT_MAX_TURN_SPLIT)

        direction = 1.0 if x >= 0.0 else -1.0
        lhp = hip_base  + direction * swing_amp * s_l - turn_split
        rhp = hip_base  + direction * swing_amp * s_r + turn_split
        lk  = knee_base + 0.20 * max(0.0, s_l) * ramp + 0.14 * abs(theta)
        rk  = knee_base + 0.20 * max(0.0, s_r) * ramp + 0.14 * abs(theta)
        lap = -0.92 * lhp - 0.03
        rap = -0.92 * rhp - 0.03
        lhr = roll_amp * shift;  rhr = -roll_amp * shift
        lar = -0.70 * lhr;       rar = -0.70 * rhr
        lhy = -0.18 * theta;     rhy =  0.18 * theta

        self._set_pose(lhy, lhr, lhp, lk, lap, lar,
                       rhy, rhr, rhp, rk, rap, rar)

    def _set_pose(self, lhy, lhr, lhp, lk, lap, lar,
                        rhy, rhr, rhp, rk, rap, rar):
        if not self._gait_motors:
            return
        lhp = _clamp(lhp, -0.95, 0.35); rhp = _clamp(rhp, -0.95, 0.35)
        lk  = _clamp(lk,   0.05, 2.10); rk  = _clamp(rk,   0.05, 2.10)
        lap = _clamp(lap,  -1.00, 0.55); rap = _clamp(rap,  -1.00, 0.55)
        lhy = _clamp(lhy,  -0.35, 0.35); rhy = _clamp(rhy,  -0.35, 0.35)
        lhr = _clamp(lhr,  -0.35, 0.35); rhr = _clamp(rhr,  -0.35, 0.35)
        lar = _clamp(lar,  -0.35, 0.35); rar = _clamp(rar,  -0.35, 0.35)
        m = self._gait_motors
        m["LHipYawPitch"].setPosition(lhy); m["LHipRoll"].setPosition(lhr)
        m["LHipPitch"].setPosition(lhp);    m["LKneePitch"].setPosition(lk)
        m["LAnklePitch"].setPosition(lap);  m["LAnkleRoll"].setPosition(lar)
        m["RHipYawPitch"].setPosition(rhy); m["RHipRoll"].setPosition(rhr)
        m["RHipPitch"].setPosition(rhp);    m["RKneePitch"].setPosition(rk)
        m["RAnklePitch"].setPosition(rap);  m["RAnkleRoll"].setPosition(rar)

    # --------------------------------------------------------------------------
    # POSITION / ORIENTATION
    # --------------------------------------------------------------------------
    def _get_position(self):
        if self._self_node:
            return self._self_node.getPosition()
        return [0.0, 0.0, 0.0]

    def _get_heading(self) -> float:
        if not self._self_node:
            return 0.0
        o = self._self_node.getOrientation()
        return math.atan2(o[3], o[0])

    def _service_nao_pos(self):
        try:
            node = self.robot.getFromDef("SERVICE_NAO")
            if node:
                return node.getPosition()
        except Exception:
            pass
        return None

    def _dist_to_service_nao(self) -> float:
        spos = self._service_nao_pos()
        if spos is None:
            return float("inf")
        gpos = self._get_position()
        dx = spos[0] - gpos[0]
        dz = spos[2] - gpos[2]
        return math.sqrt(dx * dx + dz * dz)

    # --------------------------------------------------------------------------
    # COMMUNICATION
    # --------------------------------------------------------------------------
    def _poll_incoming(self) -> list:
        """Drain receiver queue; return decoded signal strings."""
        if not self._receiver:
            return []
        messages = []
        while self._receiver.getQueueLength() > 0:
            try:
                data = self._receiver.getData().decode("utf-8")
                messages.append(data.strip())
                self._receiver.nextPacket()
            except Exception:
                break
        return messages

    def _dispatch_signal(self, msg: str):
        """
        Handle incoming signal from SERVICE_NAO.

        Protocol:
            "start_walk"  → WALKING_SLOW (begin slow walk episode)
            "stop"        → RESTING      (halt any movement)
            "assist"      → WAITING_FOR_ASSISTANCE
            "arrived"     → RESTING      (destination confirmed by SERVICE_NAO)
        """
        print(f"{LOG_PREFIX} Signal received: \"{msg}\"")
        token = msg.lower().strip()

        if token == "start_walk":
            if self.state not in (GrandpaState.WALKING_SLOW, GrandpaState.BEING_ASSISTED):
                self._start_walk_episode()

        elif token == "stop":
            self._stop_motion()
            self._enter(GrandpaState.RESTING)

        elif token == "assist":
            self._stop_motion()
            self._enter(GrandpaState.WAITING_FOR_ASSISTANCE)

        elif token == "arrived":
            self._stop_motion()
            self.energy = min(ENERGY_MAX, self.energy + 20.0)
            self._enter(GrandpaState.RESTING)

        else:
            print(f"{LOG_PREFIX} Unknown signal ignored: \"{msg}\"")

    def _start_walk_episode(self):
        self._in_pause        = False
        self._pause_until     = 0.0
        self._walk_ramp_start = None
        self._enter(GrandpaState.WALKING_SLOW)

    # --------------------------------------------------------------------------
    # STATE MACHINE CORE
    # --------------------------------------------------------------------------
    def _enter(self, new_state: GrandpaState):
        if self.state != new_state:
            print(f"{LOG_PREFIX} {self.state.name} → {new_state.name}"
                  f"  [energy={self.energy:.1f}]")
        self.state            = new_state
        self.state_start_time = self.robot.getTime()

    def _time_in_state(self) -> float:
        return self.robot.getTime() - self.state_start_time

    # --------------------------------------------------------------------------
    # STATE HANDLERS
    # --------------------------------------------------------------------------
    def _handle_resting(self, dt: float):
        """
        Default stable state. Stands still indefinitely.
        Energy restores slowly (informational; no auto-walk triggers).
        No joint velocity commands issued here.
        """
        self.energy = min(ENERGY_MAX, self.energy + ENERGY_RESTORE_PER_S * dt)

    def _handle_walking_slow(self, dt: float):
        """
        Slow walk episode. Energy drains continuously (time-based).
        Random pauses simulate elderly shuffling.
        Transitions to TIRED when energy depleted.
        Walk stops only via energy depletion or "stop" signal.
        """
        # Time-based energy drain
        self.energy = max(0.0, self.energy - ENERGY_DRAIN_PER_S * dt)

        if self.energy <= ENERGY_TIRED_THRESH:
            self._stop_motion()
            print(f"{LOG_PREFIX} Fatigued — entering TIRED")
            self._enter(GrandpaState.TIRED)
            return

        now = self.robot.getTime()

        # Pause logic
        if self._in_pause:
            if self._fwd_motion and self._motion_playing:
                self._fwd_motion.stop()
                self._motion_playing = False
            self._walk_ramp_start = None   # re-ramp when resuming
            if now >= self._pause_until:
                self._in_pause = False
                print(f"{LOG_PREFIX} Resuming walk after pause")
            return

        if random.random() < PAUSE_PROBABILITY:
            dur = random.uniform(PAUSE_MIN_S, PAUSE_MAX_S)
            self._in_pause    = True
            self._pause_until = now + dur
            print(f"{LOG_PREFIX} Pause ({dur:.1f}s)")
            return

        # Walk: prefer motion file; fall back to joint gait
        if self._fwd_motion is not None:
            if self._fwd_motion.isOver():
                self._fwd_motion.setTime(0)
                self._fwd_motion.play()
                self._motion_playing = True
        elif self._movement_ok:
            theta = random.uniform(-0.04, 0.04)
            self._apply_gait(x=GRANDPA_FWD_MAX, theta=theta)

    def _handle_tired(self, dt: float):
        """
        Energy depleted. Stands still. No automatic escalation.
        Waits for "assist" signal from SERVICE_NAO to enter WAITING_FOR_ASSISTANCE.
        Energy restores slowly while standing.
        """
        self.energy = min(ENERGY_MAX, self.energy + ENERGY_RESTORE_PER_S * dt)

    def _handle_waiting_for_assistance(self, dt: float):
        """
        Waiting for SERVICE_NAO to physically arrive.
        Transitions automatically when SERVICE_NAO enters ASSISTANCE_PROXIMITY_M.
        """
        if self._dist_to_service_nao() <= ASSISTANCE_PROXIMITY_M:
            print(f"{LOG_PREFIX} SERVICE_NAO arrived — beginning assisted walk")
            self._enter(GrandpaState.BEING_ASSISTED)

    def _handle_being_assisted(self, dt: float):
        """
        Follow SERVICE_NAO at slow pace, trailing slightly.
        Transitions to RESTING when SERVICE_NAO stops within ARRIVED_DISTANCE_M
        or when "arrived" signal received.
        """
        spos = self._service_nao_pos()
        if spos is None:
            return

        gpos = self._get_position()
        dx   = spos[0] - gpos[0]
        dz   = spos[2] - gpos[2]
        dist = math.sqrt(dx * dx + dz * dz)

        if dist <= ARRIVED_DISTANCE_M:
            print(f"{LOG_PREFIX} Destination reached. Resting.")
            self._stop_motion()
            self.energy = min(ENERGY_MAX, self.energy + 20.0)
            self._enter(GrandpaState.RESTING)
            return

        if self._movement_ok and dist > ASSISTANCE_PROXIMITY_M * 0.4:
            heading      = self._get_heading()
            target_angle = math.atan2(dx, dz)
            err          = (target_angle - heading + math.pi) % (2 * math.pi) - math.pi
            theta        = _clamp(err * 0.8, -0.30, 0.30)
            self._apply_gait(x=GRANDPA_FWD_MAX * 0.5, theta=theta)

    def _handle_emergency_fall(self, dt: float):
        """
        EMERGENCY_FALL — PLACEHOLDER.
        Future: read InertialUnit for pitch/roll spike, detect Z-drop,
        play StandUpFromBack/Front motion, signal SERVICE_NAO.
        Currently: log only, no action.
        """
        print(f"{LOG_PREFIX} EMERGENCY_FALL (placeholder — no action taken)")

    # --------------------------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------------------------
    def run(self):
        print(f"{LOG_PREFIX} Main loop started")
        prev_time = self.robot.getTime()

        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            dt  = _clamp(now - prev_time, 0.0, 0.1)
            prev_time = now

            # Initialization delay: hold standing pose, no signals processed
            if not self._initialized:
                if self._init_start_t is None:
                    self._init_start_t = now
                    print(f"{LOG_PREFIX} Stabilizing... ({INIT_DELAY_S}s)")
                self._apply_stand_pose()
                if now - self._init_start_t < INIT_DELAY_S:
                    continue
                self._initialized = True
                self.state_start_time = now
                print(f"{LOG_PREFIX} Stabilization complete — standing by")

            # Process incoming signals FIRST (highest priority)
            for msg in self._poll_incoming():
                self._dispatch_signal(msg)

            # Dispatch state handler
            match self.state:
                case GrandpaState.RESTING:
                    self._handle_resting(dt)
                case GrandpaState.WALKING_SLOW:
                    self._handle_walking_slow(dt)
                case GrandpaState.TIRED:
                    self._handle_tired(dt)
                case GrandpaState.WAITING_FOR_ASSISTANCE:
                    self._handle_waiting_for_assistance(dt)
                case GrandpaState.BEING_ASSISTED:
                    self._handle_being_assisted(dt)
                case GrandpaState.EMERGENCY_FALL:
                    self._handle_emergency_fall(dt)


# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    GrandpaController().run()
