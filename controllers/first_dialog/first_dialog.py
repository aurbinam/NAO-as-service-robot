"""
NAO First Dialog Module - Voice Listener Integration
=====================================================
NAO greets the user and captures their name via voice recognition.

This module provides:
  - get_user_name(robot, timestep) : Run greeting flow and return captured name

CONNECTIONS (pick one by setting CONNECTION_MODE below):
  - "RECEIVER": Webots Receiver device (voice listener uses Emitter)
  - "TCP":      TCP socket on localhost:5005 (voice listener connects as client)

MESSAGE FORMATS (case-insensitive):
  - "NAME:Aris"        -> extracts "Aris"
  - "MY_NAME_IS Aris"  -> extracts "Aris"

Usage:
  As module: from first_dialog import get_user_name
             name = get_user_name(robot, timestep)
  
  Standalone: Set this as NAO's controller in Webots and press Play.
"""

import socket
import select
import os
os.environ['WEBOTS_WARNINGS'] = 'disable'

# CONFIGURATION - CHANGE THIS TO SELECT CONNECTION MODE
CONNECTION_MODE = "TCP"  # Options: "RECEIVER" or "TCP"
TCP_HOST = "127.0.0.1"
TCP_PORT = 5005

SPEAK_VOLUME = 1.0
SPEECH_SECONDS_PER_CHAR = 0.085  
MIN_SPEECH_SECONDS = 2.2         
EXTRA_PAUSE_AFTER_LINE = 0.8
PUNCTUATION_BONUS_PER_MARK = 0.25  
PUNCTUATION_MARKS = ".?!,;:"

# Module-level state (initialized when get_user_name is called or standalone)
_robot = None
_timestep = None
_speaker = None
_receiver = None
_tcp_server = None
_tcp_client = None
_tcp_buffer = ""
_initialized = False

# INITIALIZATION
def _init_module(robot, timestep):
    """
    Initialize module state with provided robot and timestep.
    Called automatically by get_user_name() or can be called manually.
    """
    global _robot, _timestep, _speaker, _receiver, _initialized
    global _tcp_server, _tcp_client, _tcp_buffer
    
    if _initialized:
        return  # Already initialized
    
    _robot = robot
    _timestep = timestep
    
    # SPEAKER DEVICE
    try:
        _speaker = robot.getDevice("speaker")
        if _speaker:
            print("[FirstDialog] Speaker device found")
        else:
            print("[FirstDialog] Speaker not found - will only print text")
    except Exception as e:
        print(f"[FirstDialog] Speaker error: {e}")
        _speaker = None
    
    # RECEIVER SETUP (if using receiver mode)
    if CONNECTION_MODE == "RECEIVER":
        try:
            _receiver = robot.getDevice("receiver")
            if _receiver:
                _receiver.enable(timestep)
                print("[FirstDialog] Receiver device enabled")
            else:
                print("[FirstDialog] Receiver not found")
        except Exception as e:
            print(f"[FirstDialog] Receiver error: {e}")
            _receiver = None
    
    # TCP SETUP (if using TCP mode)
    if CONNECTION_MODE == "TCP":
        try:
            _tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            _tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            _tcp_server.bind((TCP_HOST, TCP_PORT))
            _tcp_server.listen(1)
            _tcp_server.setblocking(False)
            print(f"[FirstDialog] TCP Server listening on {TCP_HOST}:{TCP_PORT}")
        except Exception as e:
            print(f"[FirstDialog] TCP Server setup error: {e}")
            _tcp_server = None
    
    _tcp_client = None
    _tcp_buffer = ""
    _initialized = True


def _step_seconds(seconds: float) -> bool:
    """
    Step the simulation for `seconds` without blocking the OS thread.
    Returns False if simulation ends.
    """
    global _robot, _timestep
    steps = int((seconds * 1000.0) / _timestep)
    for _ in range(max(1, steps)):
        if _robot.step(_timestep) == -1:
            return False
    return True


def _estimate_speech_time(text: str) -> float:
    """
    Conservative time estimate so speech does NOT overlap.
    This does not change the TTS engine rate; it prevents rushing/overlap.
    """
    # Base estimate from length
    base = max(MIN_SPEECH_SECONDS, len(text) * SPEECH_SECONDS_PER_CHAR)

    # Add natural pauses if punctuation exists
    punct_count = sum(text.count(ch) for ch in PUNCTUATION_MARKS)
    punctuation_bonus = punct_count * PUNCTUATION_BONUS_PER_MARK

    # Add a fixed extra pause after every line
    total = base + punctuation_bonus + EXTRA_PAUSE_AFTER_LINE
    return total


def say(text: str) -> bool:
    """
    Make NAO speak the text and print it to console.
    Waits long enough after speaking so phrases don't overlap or feel rushed.
    """
    global _speaker
    print(f'NAO: "{text}"')

    # Speak if speaker exists
    if _speaker:
        try:
            # Some Webots builds use speak(text, volume), others speak(text)
            try:
                _speaker.speak(text, SPEAK_VOLUME)
            except TypeError:
                _speaker.speak(text)
        except Exception as e:
            print(f"[FirstDialog] Speaker error: {e}")

    # Wait for an estimated duration so the next phrase doesn't cut this one off
    wait_time = _estimate_speech_time(text)
    return _step_seconds(wait_time)


# RECEIVER MESSAGES
def _read_receiver_messages():
    """
    Drain all pending messages from the Receiver.
    Returns a list of decoded strings.
    """
    global _receiver
    messages = []
    if _receiver is None:
        return messages

    while _receiver.getQueueLength() > 0:
        try:
            data = _receiver.getData()
            if data:
                msg = data.decode("utf-8", errors="ignore").strip()
                if msg:
                    messages.append(msg)
        except Exception as e:
            print(f"[FirstDialog] Receiver read error: {e}")
        _receiver.nextPacket()

    return messages


# TCP MESSAGES
def _read_tcp_messages():
    """
    Non-blocking read from TCP socket.
    Returns a list of complete lines (messages).
    """
    global _tcp_client, _tcp_buffer, _tcp_server
    messages = []

    if _tcp_server is None:
        return messages

    # Accept client (non-blocking)
    if _tcp_client is None:
        try:
            readable, _, _ = select.select([_tcp_server], [], [], 0)
            if readable:
                _tcp_client, addr = _tcp_server.accept()
                _tcp_client.setblocking(False)
                print(f"[FirstDialog] TCP Client connected from {addr}")
        except Exception:
            pass

    # Read from client (non-blocking)
    if _tcp_client is not None:
        try:
            readable, _, _ = select.select([_tcp_client], [], [], 0)
            if readable:
                data = _tcp_client.recv(1024)
                if data:
                    _tcp_buffer += data.decode("utf-8", errors="ignore")
                else:
                    print("[FirstDialog] TCP Client disconnected")
                    _tcp_client.close()
                    _tcp_client = None
                    _tcp_buffer = ""
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[FirstDialog] TCP Read error: {e}")
            try:
                _tcp_client.close()
            except Exception:
                pass
            _tcp_client = None
            _tcp_buffer = ""

    # Extract complete lines
    while "\n" in _tcp_buffer:
        line, _tcp_buffer = _tcp_buffer.split("\n", 1)
        line = line.strip()
        if line:
            messages.append(line)

    return messages


# NAME PARSING
def parse_name_from_message(msg):
    """
    Parse a name from message formats:
      - "NAME:Aris"       -> "Aris"
      - "MY_NAME_IS Aris" -> "Aris"
      - "MY NAME IS Aris" -> "Aris"

    Returns the name (1-3 words) or None if not found.
    Case-insensitive matching.
    """
    msg_upper = msg.upper().strip()
    name = None

    if msg_upper.startswith("NAME:"):
        name = msg[5:].strip()  # keep original case for the name
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


# UNIFIED MESSAGE READING
def _read_messages():
    if CONNECTION_MODE == "RECEIVER":
        return _read_receiver_messages()
    elif CONNECTION_MODE == "TCP":
        return _read_tcp_messages()
    return []


# MAIN API: get_user_name()
def get_user_name(robot, timestep):
    """
    Run the complete greeting flow and return the captured user name.
    
    This function:
      1. Initializes speaker, TCP server (or receiver)
      2. Greets the user with intro phrases
      3. Asks for user's name
      4. Waits FOREVER until a valid name is received
      5. Returns the captured name
    
    Args:
        robot: Webots Robot instance
        timestep: Simulation timestep (ms)
    
    Returns:
        str: The captured user name (never None, waits forever)
    """
    global _robot, _timestep, _initialized
    
    # Initialize module state
    _init_module(robot, timestep)
    
    print("=" * 50)
    print("NAO FIRST DIALOG - Greeting Flow")
    print(f"Connection mode: {CONNECTION_MODE}")
    print("=" * 50)
    
    # Small pause right when starting (feels less abrupt)
    _step_seconds(0.6)
    
    # Intro phrases
    say("Hello! I am NAO, your assistant.")
    say("Welcome. I am ready to help you.")
    say("What is your name?")
    
    print("[FirstDialog] Waiting for name...")
    
    user_name = None
    
    # Wait FOREVER until name is received
    while _robot.step(_timestep) != -1:
        messages = _read_messages()
        
        for msg in messages:
            print(f"[FirstDialog] Received: {msg}")
            name = parse_name_from_message(msg)
            if name:
                user_name = name
                print(f"[FirstDialog] Name captured: {user_name}")
                break
        
        if user_name:
            break
    
    # Confirm name
    if user_name:
        say(f"Nice to meet you, {user_name}.")
        say("Let me learn about your house!")
    
    # Cleanup TCP for handoff to main controller
    _cleanup_tcp()
    
    return user_name


def _cleanup_tcp():
    """Clean up TCP resources so main controller can use the port if needed."""
    global _tcp_server, _tcp_client, _initialized
    try:
        if _tcp_client:
            _tcp_client.close()
            _tcp_client = None
        if _tcp_server:
            _tcp_server.close()
            _tcp_server = None
    except Exception:
        pass
    _initialized = False  # Allow re-initialization if needed


# STANDALONE MODE (when run directly as controller)
# This block only runs when first_dialog.py is the actual Webots controller,
# NOT when imported as a module by nao_main_controller.
def _run_standalone():
    """Run as standalone Webots controller."""
    try:
        from controller import Robot as WebotsRobot
        _standalone_robot = WebotsRobot()
        _standalone_timestep = int(_standalone_robot.getBasicTimeStep())
        
        # Run greeting flow
        name = get_user_name(_standalone_robot, _standalone_timestep)
        
        if name:
            print(f"[Standalone] Greeting complete, user name: {name}")
        
        # Idle loop (standalone mode only)
        print("[Standalone] Entering idle loop...")
        while _standalone_robot.step(_standalone_timestep) != -1:
            pass
        
        print("[Standalone] Controller ended")
    except ImportError:
        # Not running in Webots, just module import
        pass


# Only run standalone if this is the main script
if __name__ == "__main__":
    _run_standalone()
