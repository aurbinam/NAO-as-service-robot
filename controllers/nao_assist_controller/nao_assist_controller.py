"""
NAO Assist Controller - Main Entry Point

This controller:
1. Checks if user profile exists and is confirmed
2. If not, captures user name via TCP (from voice_listener.py) or typed fallback
3. Persists profile to data/user_profile.json
4. NAO says "Nice to meet you, <name>. I am ready."
5. After identity confirmed, accepts commands:
   - "list places" - shows available navigation targets
   - "go to <target>" - navigates to static target location (opens doors on route)
   - "open the <label> door" - opens a door
   - "close the <label> door" - closes a door

Demo features:
- Press 'R' key to reset profile and restart name acquisition flow

House knowledge is STATIC (from DEF nodes in world), NOT learned/mapped.
Door control uses the Webots Supervisor API to manipulate hinge joint fields
directly. Navigation and door handling are routed through RobotBrain, which
orchestrates GenAIInterpreter -> TaskPlanner -> Executor internally.
"""

import sys
import os
import socket
import select
import threading
import time
import re
from pathlib import Path



# Add controller directory to path for imports
controller_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, controller_dir)

from ai_command_parser import parse_command
from controller import Supervisor, Keyboard

from config import (
    TIMESTEP_MS, LOG_PREFIX, TCP_HOST, TCP_PORT,
    NAME_CAPTURE_TIMEOUT_SECONDS, SPEAK_VOLUME,
    SPEECH_SECONDS_PER_CHAR, MIN_SPEECH_SECONDS,
    EXTRA_PAUSE_AFTER_LINE, PUNCTUATION_BONUS_PER_MARK,
    PUNCTUATION_MARKS
)
from user_profile import (
    load_profile, save_profile, profile_exists_and_confirmed,
    get_or_create_profile, UserProfile
)
from house.house_loader import load_house, get_house_config
from intelligence.robot_brain import RobotBrain
from intelligence.executor import Executor
from assistive_tasks import AssistiveStateMachine, AssistiveState
from companion_brain import CompanionBrain

# =============================================================================
# PROJECT PATHS
# =============================================================================
PROJECT_ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = PROJECT_ROOT / "data" / "user_profile.json"


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================
# Reset command words (case-insensitive) - for TCP/voice reset
RESET_COMMANDS = {"reset", "restart", "start over"}


def reset_profile_file() -> bool:
    """
    Delete the user profile file from disk.
    
    Returns:
        True if file was deleted or didn't exist, False on error.
    """
    print(f"{LOG_PREFIX} [RESET] Profile path: {PROFILE_PATH}")
    print(f"{LOG_PREFIX} [RESET] Exists before: {PROFILE_PATH.exists()}")
    
    try:
        if PROFILE_PATH.exists():
            PROFILE_PATH.unlink()
            print(f"{LOG_PREFIX} [RESET] Profile file DELETED: {PROFILE_PATH}")
            print(f"{LOG_PREFIX} [RESET] Exists after: {PROFILE_PATH.exists()}")
            return True
        else:
            print(f"{LOG_PREFIX} [RESET] Profile file not found (already clean)")
            return True
    except Exception as e:
        print(f"{LOG_PREFIX} [RESET] ERROR deleting profile: {e}")
        import traceback
        traceback.print_exc()
        return False


def is_reset_command(msg: str) -> bool:
    """
    Check if message is a reset command.
    
    Args:
        msg: Raw message string.
    
    Returns:
        True if message matches a reset command.
    """
    return msg.strip().lower() in RESET_COMMANDS


def clean_name(raw: str) -> str:
    """
    Clean a raw name string by removing common prefixes and punctuation.
    
    Handles inputs like:
      - "name is ABBA"      -> "ABBA"
      - "my name is ABBA"   -> "ABBA"
      - "I am ABBA"         -> "ABBA"
      - "I'm ABBA"          -> "ABBA"
      - "ABBA!"             -> "ABBA"
    
    Args:
        raw: Raw name string possibly with prefixes.
    
    Returns:
        Cleaned name (1-3 tokens max).
    """
    if not raw:
        return ""
    
    # Remove leading/trailing whitespace
    text = raw.strip()
    
    # Case-insensitive removal of common prefixes
    # Order matters: longer patterns first
    prefixes = [
        r"^my\s+name\s+is\s+",      # "my name is "
        r"^name\s+is\s+",           # "name is "
        r"^i\s+am\s+",              # "i am "
        r"^i'm\s+",                 # "i'm "
        r"^it's\s+",                # "it's "
        r"^call\s+me\s+",           # "call me "
    ]
    
    for pattern in prefixes:
        text = re.sub(pattern, "", text, flags=re.IGNORECASE)
    
    # Remove punctuation (keep apostrophes in names like O'Brien)
    text = re.sub(r"[^\w\s\-']", "", text)
    
    # Split and take first 1-3 tokens
    tokens = text.split()[:3]
    cleaned = " ".join(tokens).strip()
    
    return cleaned if cleaned else raw.strip()


class NAOAssistController:
    """
    Main controller for NAO assist mode.
    
    """
    
    def __init__(self):
        """Initialize the controller."""
        # Use Supervisor for position tracking and DEF node access
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        if self.timestep == 0:
            self.timestep = TIMESTEP_MS
        
        # Devices
        self._speaker = None
        self._keyboard = None
        self._init_devices()
        
        # House knowledge
        self._house_config = None
        self._brain = None
        
        # Assistive tasks
        self._executor = None
        self._assistive_state_machine = None
        self._grandpa_position = None
        
        # Grandpa signal emitter (super_emitter, channel 2)
        self._grandpa_emitter = None

        # TCP state
        self._tcp_server = None
        self._tcp_client = None
        self._tcp_buffer = ""
        
        # Typed input state
        self._typed_name = None
        self._input_thread = None
        self._input_active = False
        self._typed_commands = []  # queue for READY-state typed commands
        
        # State
        self._identity_confirmed = False
        self._user_profile = None
        self._needs_reset = False
        self._companion = None
    
    def _init_devices(self):
        """Initialize NAO devices."""
        # Speaker
        try:
            self._speaker = self.robot.getDevice("speaker")
            if self._speaker:
                print(f"{LOG_PREFIX} Speaker device found")
            else:
                print(f"{LOG_PREFIX} Speaker not found - will only print text")
        except Exception as e:
            print(f"{LOG_PREFIX} Speaker error: {e}")
            self._speaker = None
        
        # Keyboard (for demo reset with 'R' key)
        try:
            self._keyboard = self.robot.getKeyboard()
            if self._keyboard:
                self._keyboard.enable(self.timestep)
                print(f"{LOG_PREFIX} [INIT] Keyboard enabled (press 'R' to reset demo)")
            else:
                print(f"{LOG_PREFIX} [INIT] Keyboard not available")
        except Exception as e:
            print(f"{LOG_PREFIX} [INIT] Keyboard error: {e}")
            self._keyboard = None

        # Grandpa signal emitter (super_emitter on channel 2)
        try:
            self._grandpa_emitter = self.robot.getDevice("super_emitter")
            if self._grandpa_emitter:
                print(f"{LOG_PREFIX} [INIT] Grandpa emitter ready (channel 2)")
            else:
                print(f"{LOG_PREFIX} [INIT] super_emitter not found — Grandpa signals disabled")
        except Exception as e:
            print(f"{LOG_PREFIX} [INIT] Grandpa emitter error: {e}")
            self._grandpa_emitter = None

    def _get_grandpa_position(self) -> tuple:
        """
        Read Grandpa's real-time position from DEF GRANDPA Supervisor node.

        Returns None if the node is unreachable — callers must handle None
        rather than falling back to a stale default coordinate.
        """
        try:
            grandpa_node = self.robot.getFromDef("GRANDPA")
            if grandpa_node:
                pos = grandpa_node.getPosition()
                return (pos[0], pos[1], pos[2])
        except Exception as e:
            print(f"{LOG_PREFIX} Could not read GRANDPA position: {e}")
        return None

    def _refresh_grandpa_position(self) -> None:
        """
        Re-read Grandpa's position from Supervisor and push to the assistive
        state machine. Called every simulation step in the READY loop.
        """
        pos = self._get_grandpa_position()
        if pos is not None and self._assistive_state_machine is not None:
            self._assistive_state_machine.set_grandpa_position(pos)
    
    def _check_reset_key(self) -> bool:
        """
        Check if 'R' key was pressed for demo reset.
        Handles Webots modifier bits by masking with 0xFF.
        
        Returns:
            True if reset key was pressed, False otherwise.
        """
        if self._keyboard is None:
            return False
        
        key = self._keyboard.getKey()
        while key != -1:
            # Mask off modifier bits to get ASCII value
            k = key & 0xFF
            
            try:
                char_repr = chr(k) if 32 <= k < 127 else '?'
                print(f"{LOG_PREFIX} [KEY] raw={key} ascii={k} char='{char_repr}'")
            except:
                print(f"{LOG_PREFIX} [KEY] raw={key} ascii={k} char=?")
            
            # Check for 'R' or 'r' (82 = 'R', 114 = 'r')
            if k == ord('R') or k == ord('r'):
                print(f"{LOG_PREFIX} [KEY] Reset key detected!")
                return True
            
            key = self._keyboard.getKey()
        
        return False
    
    def _perform_reset(self):
        """
        Reset profile and dialogue state for demo.
        """
        print(f"\n{LOG_PREFIX} " + "=" * 50)
        print(f"{LOG_PREFIX} DEMO RESET TRIGGERED")
        print(f"{LOG_PREFIX} " + "=" * 50)
        
        # Delete profile file
        reset_profile_file()
        
        # Reset state variables
        self._identity_confirmed = False
        self._user_profile = None
        self._typed_name = None
        self._needs_reset = True
        self._companion = None
        
        # Cleanup any existing TCP connection
        self._cleanup_tcp()
    
    # =========================================================================
    # SPEECH
    # =========================================================================
    def _estimate_speech_time(self, text: str) -> float:
        """Estimate speech duration to prevent overlap."""
        base = max(MIN_SPEECH_SECONDS, len(text) * SPEECH_SECONDS_PER_CHAR)
        punct_count = sum(text.count(ch) for ch in PUNCTUATION_MARKS)
        punctuation_bonus = punct_count * PUNCTUATION_BONUS_PER_MARK
        return base + punctuation_bonus + EXTRA_PAUSE_AFTER_LINE
    
    def _step_seconds(self, seconds: float) -> bool:
        """Step simulation for specified seconds."""
        steps = int((seconds * 1000.0) / self.timestep)
        for _ in range(max(1, steps)):
            if self.robot.step(self.timestep) == -1:
                return False
        return True
    
    def say(self, text: str) -> bool:
        """Make NAO speak and wait appropriately."""
        print(f'{LOG_PREFIX} NAO: "{text}"')
        
        if self._speaker:
            try:
                try:
                    self._speaker.speak(text, SPEAK_VOLUME)
                except TypeError:
                    self._speaker.speak(text)
            except Exception as e:
                print(f"{LOG_PREFIX} Speaker error: {e}")
        
        wait_time = self._estimate_speech_time(text)
        return self._step_seconds(wait_time)
    
    # =========================================================================
    # TCP SERVER
    # =========================================================================
    def _init_tcp_server(self):
        """Initialize TCP server for voice_listener.py communication."""
        try:
            self._tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._tcp_server.bind((TCP_HOST, TCP_PORT))
            self._tcp_server.listen(1)
            self._tcp_server.setblocking(False)
            print(f"{LOG_PREFIX} TCP Server listening on {TCP_HOST}:{TCP_PORT}")
            return True
        except Exception as e:
            print(f"{LOG_PREFIX} TCP Server setup error: {e}")
            self._tcp_server = None
            return False
    
    def _read_tcp_messages(self) -> list:
        """Non-blocking read from TCP socket."""
        messages = []
        
        if self._tcp_server is None:
            return messages
        
        # Accept client (non-blocking)
        if self._tcp_client is None:
            try:
                readable, _, _ = select.select([self._tcp_server], [], [], 0)
                if readable:
                    self._tcp_client, addr = self._tcp_server.accept()
                    self._tcp_client.setblocking(False)
                    print(f"{LOG_PREFIX} TCP Client connected from {addr}")
            except Exception:
                pass
        
        # Read from client (non-blocking)
        if self._tcp_client is not None:
            try:
                readable, _, _ = select.select([self._tcp_client], [], [], 0)
                if readable:
                    data = self._tcp_client.recv(1024)
                    if data:
                        self._tcp_buffer += data.decode("utf-8", errors="ignore")
                    else:
                        print(f"{LOG_PREFIX} TCP Client disconnected")
                        self._tcp_client.close()
                        self._tcp_client = None
                        self._tcp_buffer = ""
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"{LOG_PREFIX} TCP Read error: {e}")
                try:
                    self._tcp_client.close()
                except Exception:
                    pass
                self._tcp_client = None
                self._tcp_buffer = ""
        
        # Extract complete lines
        while "\n" in self._tcp_buffer:
            line, self._tcp_buffer = self._tcp_buffer.split("\n", 1)
            line = line.strip()
            if line:
                messages.append(line)
        
        return messages
    
    def _cleanup_tcp(self):
        """Clean up TCP resources."""
        try:
            if self._tcp_client:
                self._tcp_client.close()
                self._tcp_client = None
            if self._tcp_server:
                self._tcp_server.close()
                self._tcp_server = None
        except Exception:
            pass
        self._tcp_buffer = ""
    
    # =========================================================================
    # NAME PARSING
    # =========================================================================
    def _parse_name_from_message(self, msg: str) -> str:
        """
        Parse name from message formats:
          - "NAME:Aris"       -> "Aris"
          - "MY_NAME_IS Aris" -> "Aris"
          - "MY NAME IS Aris" -> "Aris"
        """
        msg_upper = msg.upper().strip()
        name = None
        
        if msg_upper.startswith("NAME:"):
            name = msg[5:].strip()
        elif msg_upper.startswith("MY_NAME_IS "):
            name = msg[11:].strip()
        elif msg_upper.startswith("MY NAME IS "):
            name = msg[11:].strip()
        
        if name:
            words = name.split()[:3]
            name = " ".join(words).strip()
            if len(name) >= 1:
                return name
        
        return None
    
    # =========================================================================
    # TYPED INPUT FALLBACK
    # =========================================================================
    def _input_thread_func(self):
        """Thread function for non-blocking console input."""
        while self._input_active:
            try:
                # This will block, but it's in a separate thread
                user_input = input()
                if user_input.strip():
                    self._typed_name = user_input.strip()
                    break
            except EOFError:
                break
            except Exception:
                break
    
    def _start_input_thread(self):
        """Start background thread for typed input."""
        self._input_active = True
        self._input_thread = threading.Thread(target=self._input_thread_func, daemon=True)
        self._input_thread.start()
    
    def _stop_input_thread(self):
        """Stop the input thread."""
        self._input_active = False
        if self._input_thread:
            self._input_thread = None

    def _command_input_thread_func(self):
        """Background thread: polls command file and queues commands."""
        cmd_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cmd.txt")
        print(f"[NAO_ASSIST] Type commands in: {cmd_file}")
        print("[NAO_ASSIST] Examples: 'go to kitchen', 'list places', 'open the bedroom door'")
        import time
        while self._input_active:
            try:
                with open(cmd_file, "r") as f:
                    content = f.read().strip()
                if content:
                    # Clear file then queue command
                    with open(cmd_file, "w") as f:
                        f.write("")
                    for line in content.splitlines():
                        line = line.strip()
                        if line:
                            self._typed_commands.append(line)
            except Exception:
                pass
            time.sleep(0.2)

    def _start_command_input_thread(self):
        """Start file-polling command input thread for READY state."""
        self._input_active = True
        self._typed_commands = []
        cmd_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cmd.txt")
        try:
            with open(cmd_file, "w") as f:
                f.write("")
            print(f"[NAO_ASSIST] Command file ready: {cmd_file}")
        except Exception as e:
            print(f"[NAO_ASSIST] WARNING: Could not create cmd.txt: {e}")
        self._input_thread = threading.Thread(target=self._command_input_thread_func, daemon=True)
        self._input_thread.start()

    def _read_typed_commands(self) -> list:
        """Drain and return all pending typed commands."""
        cmds, self._typed_commands = self._typed_commands, []
        return cmds

    # =========================================================================
    # IDENTITY CAPTURE
    # =========================================================================
    def _capture_identity(self) -> bool:
        """
        Capture user identity via TCP or typed fallback.
        
        Flow:
        1. Initialize TCP server
        2. Greet user and ask for name
        3. Wait for NAME message via TCP
        4. After timeout, allow typed fallback
        5. Confirm and save profile
        
        Returns:
            True if identity was captured and confirmed, False if simulation ends.
        """
        # Initialize TCP
        self._init_tcp_server()
        
        # Small pause before greeting
        self._step_seconds(0.6)
        
        # Greeting
        self.say("Hello. I am NAO, and I am here with you.")
        self.say("What is your name?")
        
        print(f"{LOG_PREFIX} Waiting for name via TCP ({TCP_HOST}:{TCP_PORT})...")
        print(f"{LOG_PREFIX} Or type your name in console after {NAME_CAPTURE_TIMEOUT_SECONDS} seconds.")
        
        user_name = None
        start_time = time.time()
        typed_fallback_started = False
        
        # Wait for name
        while self.robot.step(self.timestep) != -1:
            # Check for demo reset key
            if self._check_reset_key():
                self._perform_reset()
                return False  # Signal to re-enter capture flow
            
            # Check TCP messages
            messages = self._read_tcp_messages()
            for msg in messages:
                print(f"{LOG_PREFIX} Received: {msg}")
                
                # Check for TCP reset command ("reset", "restart", "start over")
                if is_reset_command(msg):
                    print(f"{LOG_PREFIX} [RESET] TCP reset command received: '{msg}'")
                    self._perform_reset()
                    return False  # Signal to re-enter capture flow
                
                name = self._parse_name_from_message(msg)
                if name:
                    # Apply name cleaning
                    user_name = clean_name(name)
                    print(f"{LOG_PREFIX} Name captured via TCP: {user_name}")
                    break
            
            if user_name:
                break
            
            # Check typed input
            if self._typed_name:
                # Check for typed reset command
                if is_reset_command(self._typed_name):
                    print(f"{LOG_PREFIX} [RESET] Typed reset command received")
                    self._typed_name = None
                    self._perform_reset()
                    return False
                
                # Apply name cleaning
                user_name = clean_name(self._typed_name)
                print(f"{LOG_PREFIX} Name captured via console: {user_name}")
                break
            
            # After timeout, enable typed fallback
            elapsed = time.time() - start_time
            if elapsed >= NAME_CAPTURE_TIMEOUT_SECONDS and not typed_fallback_started:
                typed_fallback_started = True
                print(f"\n{LOG_PREFIX} ============================================")
                print(f"{LOG_PREFIX} TYPED FALLBACK: Enter your name below:")
                print(f"{LOG_PREFIX} ============================================")
                self._start_input_thread()
        
        # Stop input thread if running
        self._stop_input_thread()
        
        # Cleanup TCP
        self._cleanup_tcp()
        
        if user_name is None:
            return False  # Simulation ended
        
        # Create/update profile
        self._user_profile = get_or_create_profile()
        self._user_profile.update_name(user_name, confirmed=True)
        
        # Save profile
        if save_profile(self._user_profile):
            print(f"{LOG_PREFIX} Profile saved successfully")
        else:
            print(f"{LOG_PREFIX} Warning: Failed to save profile")
        
        # Confirm to user
        self.say(f"Nice to meet you, {user_name}. I am here with you.")
        
        self._identity_confirmed = True
        self._companion = CompanionBrain(self.say, self._user_profile, save_profile)
        return True
    
    # =========================================================================
    # MAIN RUN LOOP
    # =========================================================================
    def run(self):
        """
        Main control loop.
        
        Iteration 3 flow:
        1. Load house configuration (static targets + doors)
        2. Check if profile exists and is confirmed -> skip to READY
        3. If not, capture identity
        4. Enter READY state (handles all commands)
        5. Check for 'R' key to reset demo at any time
        
        Commands in READY:
        - "list places" - show available targets
        - "go to <target>" - navigate (opens doors on route)
        - "open the <label> door" - open a door
        - "close the <label> door" - close a door
        """
        print("=" * 60)
        print(f"{LOG_PREFIX} NAO ASSIST CONTROLLER - Iteration 3")
        print(f"{LOG_PREFIX} Identity-first + Static House + DOORS")
        print(f"{LOG_PREFIX} Commands: 'list places', 'go to <target>'")
        print(f"{LOG_PREFIX}           'open the <label> door', 'close the <label> door'")
        print(f"{LOG_PREFIX} Press 'R' at any time to reset demo")
        print("=" * 60)
        
        # Step simulation once to initialize
        if self.robot.step(self.timestep) == -1:
            print(f"{LOG_PREFIX} Simulation ended during init")
            return
        
        # Load house configuration (static targets + doors from DEF nodes)
        from house import load_house
        self._house_config = load_house(self.robot)
        if self._house_config:
            targets = self._house_config.list_targets()
            doors = self._house_config.list_doors()
            print(f"{LOG_PREFIX} House loaded: {len(targets)} targets, {len(doors)} doors")
            self._brain = RobotBrain(self.robot, self._house_config, self.say)

            # Initialize assistive tasks system (Phase 1)
            self._executor = Executor(self.robot, self._house_config, self.say)
            initial_pos = self._get_grandpa_position()
            if initial_pos is None:
                print(f"{LOG_PREFIX} WARNING: GRANDPA DEF node not found — coordinate nav will fail until node is present")
                initial_pos = (0.0, 0.0, 0.0)  # sentinel; will be overwritten first cycle
            self._assistive_state_machine = AssistiveStateMachine(
                self.robot, self._house_config, self._executor,
                initial_pos,
                brain=self._brain,
                say_func=self.say,
            )
            print(f"{LOG_PREFIX} Assistive tasks initialized (Phase 1)")
        else:
            print(f"{LOG_PREFIX} WARNING: House config not loaded - navigation disabled")
        
        # Main state loop - allows re-entry after reset
        while True:
            # Check for existing confirmed profile (or after reset)
            if profile_exists_and_confirmed() and not self._needs_reset:
                self._user_profile = load_profile()
                print(f"{LOG_PREFIX} Existing profile found: {self._user_profile.name}")
                self._identity_confirmed = True
                self._companion = CompanionBrain(self.say, self._user_profile, save_profile)
                
                # Greet returning user
                self.say(f"Welcome back, {self._user_profile.name}. I am glad you are here.")
            else:
                if self._needs_reset:
                    # After reset, just ask for name
                    self._needs_reset = False
                    print(f"{LOG_PREFIX} Starting fresh identity capture...")
                else:
                    print(f"{LOG_PREFIX} No confirmed profile found. Starting identity capture...")
                
                if not self._capture_identity():
                    if self._needs_reset:
                        # Reset was triggered during capture, restart loop
                        self.say("Let us start fresh. What is your name?")
                        continue
                    else:
                        print(f"{LOG_PREFIX} Simulation ended during identity capture")
                        return
            
            # Enter READY state
            print("=" * 60)
            print(f"{LOG_PREFIX} READY STATE")
            print(f"{LOG_PREFIX} User: {self._user_profile.name}")
            print(f"{LOG_PREFIX} Profile: data/user_profile.json")
            if self._house_config:
                print(f"{LOG_PREFIX} House: {len(self._house_config.list_targets())} targets, {len(self._house_config.list_doors())} doors")
            print("=" * 60)
            print(f"{LOG_PREFIX} Available commands:")
            print(f"{LOG_PREFIX}   - 'list places' - show available targets")
            print(f"{LOG_PREFIX}   - 'go to <target>' - navigate (opens doors on route)")
            print(f"{LOG_PREFIX}   - 'open the <label> door' - open a door")
            print(f"{LOG_PREFIX}   - 'close the <label> door' - close a door")
            print(f"{LOG_PREFIX}   - 'reset' - restart demo")
            print(f"{LOG_PREFIX} Press 'R' or say 'reset' to restart demo")
            
            # Initialize TCP server for READY state (for reset commands)
            self._init_tcp_server()
            self._start_command_input_thread()

            # Main simulation loop - keep running until reset or end
            reset_triggered = False
            while self.robot.step(self.timestep) != -1:
                # Refresh Grandpa's real-time position every cycle
                self._refresh_grandpa_position()

                # Tick assistive state machine (handles dialogue timeout)
                if self._assistive_state_machine:
                    self._assistive_state_machine.tick(self.robot.getTime())

                # Proactive companionship and reminders (idle only)
                if self._companion:
                    assistive_busy = (
                        self._assistive_state_machine is not None and
                        self._assistive_state_machine.state != AssistiveState.IDLE
                    )
                    for msg in self._companion.tick(time.time(), assistive_busy=assistive_busy):
                        self.say(msg)

                # Check for demo reset key
                if self._check_reset_key():
                    self._perform_reset()
                    self.say("Let us start fresh. What is your name?")
                    reset_triggered = True
                    break  # Exit inner loop to restart identity capture
                
                # Check for TCP commands + typed commands in READY state
                messages = self._read_tcp_messages() + self._read_typed_commands()
                for msg in messages:
                    print(f"{LOG_PREFIX} [READY] Received: {msg}")
                    
                    # Check for reset command first
                    if is_reset_command(msg):
                        print(f"{LOG_PREFIX} [RESET] TCP reset command in READY state: '{msg}'")
                        self._perform_reset()
                        self.say("Let us start fresh. What is your name?")
                        reset_triggered = True
                        break
                    
                    # Try assistive tasks (Phase 1) first
                    if self._assistive_state_machine and self._assistive_state_machine.process_voice_input(msg):
                        continue  # Skip to next message if assistive task handled it
                    
                    # Parse and handle command (pass house_config for bare place name fallback)
                    cmd_type, cmd_arg = parse_command(msg, self._house_config)

                    # Normalize parser output to the new brain command families.
                    cmd_norm = {
                        "GO_TO": "go",
                        "GUIDE_TO": "guide",
                        "OPEN_DOOR": "open",
                        "CLOSE_DOOR": "close",
                        "LIST_PLACES": "list",
                    }.get(cmd_type)

                    if cmd_norm == "guide":
                        if self._brain:
                            from skills.assistive_guidance import execute_assistive_guidance
                            execute_assistive_guidance(
                                destination=cmd_arg,
                                robot=self.robot,
                                brain=self._brain,
                                say_func=self.say,
                                emitter=self._grandpa_emitter,
                            )
                        else:
                            self.say("House configuration not loaded. Cannot assist.")
                    elif cmd_norm in ("go", "open", "close"):
                        if self._brain:
                            self._brain.handle_command(msg)
                        else:
                            self.say("House configuration not loaded. Cannot navigate.")
                    elif cmd_norm == "list":
                        # keep existing list-places logic here unchanged
                        if self._house_config:
                            targets = self._house_config.list_targets()
                            places_str = ", ".join(targets)
                            self.say(f"I know these places: {places_str}")
                        else:
                            self.say("Sorry, I don't have any house information loaded.")
                    elif cmd_type in ("GO_TO_UNKNOWN", "GUIDE_TO_UNKNOWN"):
                        rooms = self._house_config.list_targets() if self._house_config else []
                        rooms_str = ", ".join(r.replace("_", " ") for r in rooms) or "kitchen, bedroom, bathroom"
                        self.say(
                            "I am not sure which room you mean. "
                            f"Please say one of these: {rooms_str}."
                        )
                    else:
                        if self._companion and self._companion.handle_message(msg):
                            continue
                        self.say("I am not sure I understood. You can say 'go to' or 'open the door'.")
                
                if reset_triggered:
                    break
                
                # Continue simulation loop
            
            if not reset_triggered:
                # Simulation ended (robot.step returned -1)
                self._stop_input_thread()
                self._cleanup_tcp()
                print(f"{LOG_PREFIX} Controller ended")
                return
            
            # If we get here, reset was triggered - loop back to identity capture


def main():
    """Entry point."""
    controller = NAOAssistController()
    controller.run()


if __name__ == "__main__":
    try:
        main()
    finally:
        # Close stdin to unblock any daemon thread waiting on input(),
        # then force exit so Webots doesn't hit the 1-second timeout.
        try:
            sys.stdin.close()
        except Exception:
            pass
        sys.exit(0)
