"""
Assistive Tasks Module — Dialogue-Based Assistive State Machine

HARD RULE: NAO never navigates immediately on escort/come intent.
Location is ALWAYS obtained via dialogue first.
Coordinate-based navigation is DISABLED for come/escort behavior.

State topology:
  escort_request / come_request:
    IDLE → AWAITING_LOCATION → NAVIGATING_TO_USER → ESCORTING → COMPLETED → IDLE
  bring_object (legacy):
    IDLE → NAVIGATING_TO_ROOM → SIMULATING_PICKUP → NAVIGATING_TO_USER_LEGACY → COMPLETED → IDLE

Correct console flow:
  [ASSISTIVE] Intent: escort_request
  [ASSISTIVE] Destination stored: kitchen
  [ASSISTIVE] State → AWAITING_LOCATION
  [NAO] Where are you?

  [ASSISTIVE] Location received: bedroom
  [ASSISTIVE] State → NAVIGATING_TO_USER
  [EXECUTOR] ('navigate', 'bedroom')

  [ASSISTIVE] Arrived at bedroom
  [ASSISTIVE] State → ESCORTING
  [EXECUTOR] ('navigate', 'kitchen')

  [ASSISTIVE] State → COMPLETED
  [NAO] We have arrived at the kitchen.
"""

import re
import time
from enum import Enum
from typing import Dict, Optional, Tuple

LOG_PREFIX = "[ASSISTIVE]"

LOCATION_TIMEOUT_S = 30.0
MAX_LOCATION_RETRIES = 2


# ==========================================================================
# STATES
# ==========================================================================

class AssistiveState(Enum):
    IDLE = "idle"
    AWAITING_LOCATION = "awaiting_location"
    NAVIGATING_TO_USER = "navigating_to_user"
    ESCORTING = "escorting"
    COMPLETED = "completed"
    # Legacy bring_object flow only
    NAVIGATING_TO_ROOM = "navigating_to_room"
    SIMULATING_PICKUP = "simulating_pickup"
    NAVIGATING_TO_USER_LEGACY = "navigating_to_user_legacy"


# ==========================================================================
# OBJECT-LOCATION MAPPING
# ==========================================================================

OBJECT_LOCATIONS: Dict[str, str] = {
    "water": "kitchen",
    "medicine": "kitchen",
    "pillow": "bedroom",
    "blanket": "bedroom",
    "remote": "living_room",
    "phone": "kitchen",
}


# ==========================================================================
# VOICE COMMAND PARSING
# ==========================================================================

# escort_request: combined come + destination extracted from speech.
# All of these → AWAITING_LOCATION (never immediate navigation).
_ESCORT_REQUEST_RE = [
    re.compile(r"come\s+and\s+(?:take|escort|guide|lead|bring|get)\s+me\s+to\s+(?:the\s+)?(.+)"),
    re.compile(r"come\s+(?:take|get|escort|lead)\s+me\s+to\s+(?:the\s+)?(.+)"),
    re.compile(r"(?:escort|lead)\s+me\s+to\s+(?:the\s+)?(.+)"),
    re.compile(r"take\s+me\s+to\s+(?:the\s+)?(.+)"),
    re.compile(r"help\s+me\s+(?:go\s+to|get\s+to|reach)\s+(?:the\s+)?(.+)"),
    re.compile(r"help\s+me\s+to\s+(?:the\s+)?(.+)"),
    re.compile(r"guide\s+me\s+to\s+(?:the\s+)?(.+)"),
]

# come_request: bare "come here/to me" — no destination.
# Also → AWAITING_LOCATION. After arrival, task is complete (no escort phase).
_COME_REQUEST_PHRASES = ["come here", "come to me"]


def parse_voice_command(text: str) -> Tuple[Optional[str], Optional[str]]:
    """
    Detect intent from Grandpa's voice input.

    Returns:
        ("escort_request", destination)    — come + escort with known destination
        ("escort_request_unknown", raw)    — come + escort but destination unresolved
        ("come_request", None)             — bare come here, no destination
        ("bring_object", object_name)      — fetch and deliver object
        (None, None)                       — unrecognized
    """
    text_lower = text.lower().strip()

    # 1. escort_request — specific destination extracted (highest priority)
    for pattern in _ESCORT_REQUEST_RE:
        m = pattern.search(text_lower)
        if m:
            raw_dest = m.group(1).strip()
            try:
                from ai_command_parser import resolve_navigation_target
                dest = resolve_navigation_target(raw_dest)
            except Exception:
                dest = None
            if dest:
                return ("escort_request", dest)
            return ("escort_request_unknown", raw_dest)

    # 2. come_request — bare "come here", no destination
    if any(phrase in text_lower for phrase in _COME_REQUEST_PHRASES):
        return ("come_request", None)

    # 3. bring_object — fetch and deliver
    bring_keywords = ["bring", "get", "fetch", "need", "bring me"]
    if any(kw in text_lower for kw in bring_keywords):
        for word in reversed(text_lower.split()):
            if word in OBJECT_LOCATIONS:
                return ("bring_object", word)

    return (None, None)


def get_object_location(object_name: str) -> Optional[str]:
    return OBJECT_LOCATIONS.get(object_name)


def _parse_room_from_text(text: str, house_config) -> Optional[str]:
    """
    Extract a known room ID from free-form location reply.
    "I am in the bedroom" → "bedroom"
    Never falls back to coordinates or default position.
    """
    text_lower = text.lower().strip()
    known_rooms = house_config.list_targets() if house_config else []

    for room_id in known_rooms:
        if room_id in text_lower or room_id.replace("_", " ") in text_lower:
            return room_id

    try:
        from ai_command_parser import fuzzy_match_target
        return fuzzy_match_target(text_lower, known_rooms, cutoff=0.65)
    except Exception:
        return None


# ==========================================================================
# ASSISTIVE STATE MACHINE
# ==========================================================================

class AssistiveStateMachine:
    """
    Dialogue-based assistive state machine.

    INVARIANT: No navigation occurs before a valid room name is confirmed
    via dialogue. Coordinate-based navigation is never used for
    escort_request or come_request intents.
    """

    def __init__(
        self,
        robot,
        house_config,
        executor,
        grandpa_position: Tuple[float, float, float],
        brain=None,
        say_func=None,
    ):
        self.robot = robot
        self.house_config = house_config
        self.executor = executor
        self.brain = brain
        self._say = say_func if say_func is not None else lambda t: print(f"[NAO] {t}")
        self.grandpa_position = grandpa_position

        self.state = AssistiveState.IDLE
        self._pending_destination: Optional[str] = None
        self._location_retry_count: int = 0
        self._location_timeout_start: Optional[float] = None
        # Legacy field kept for bring_object
        self.current_object: Optional[str] = None

        print(f"{LOG_PREFIX} State machine initialized")

    def set_grandpa_position(self, position: Tuple[float, float, float]) -> None:
        """Update Grandpa's real-time coordinates. Called every control cycle."""
        self.grandpa_position = position

    def tick(self, current_time: float) -> None:
        """
        Called every simulation step.
        Fires AWAITING_LOCATION → IDLE timeout if no reply arrives within 30s.
        """
        if self.state != AssistiveState.AWAITING_LOCATION:
            return
        if self._location_timeout_start is None:
            return
        elapsed = current_time - self._location_timeout_start
        if elapsed >= LOCATION_TIMEOUT_S:
            print(f"{LOG_PREFIX} Location reply timeout ({elapsed:.1f}s)")
            self._say("I did not receive a response. Please call me again when you need help.")
            self._reset()

    # ── Public entry point ─────────────────────────────────────────────── #

    def process_voice_input(self, text: str) -> bool:
        """
        Route incoming message.

        AWAITING_LOCATION: message interpreted as location reply only.
                           New navigation commands are IGNORED here.
        IDLE: message parsed for intent.

        Returns True if message was consumed (skip normal parse_command routing).
        """
        # ── Dialogue mode ──────────────────────────────────────────────
        if self.state == AssistiveState.AWAITING_LOCATION:
            return self._handle_location_reply(text)

        # ── Normal command mode ────────────────────────────────────────
        intent, param = parse_voice_command(text)

        if intent is None:
            return False

        print(f"{LOG_PREFIX} Intent: {intent}")

        if intent in ("escort_request", "come_request"):
            self._pending_destination = param  # None for come_request
            if param:
                print(f"{LOG_PREFIX} Destination stored: {param}")
            self._transition(AssistiveState.AWAITING_LOCATION)
            self._location_retry_count = 0
            self._location_timeout_start = self.robot.getTime()
            self._say("Where are you?")
            return True

        if intent == "escort_request_unknown":
            self._say(f"I would like to help you, but I do not know where '{param}' is.")
            return True

        if intent == "bring_object":
            self.current_object = param
            self._execute_bring_object()
            return True

        return False

    # ── AWAITING_LOCATION handler ──────────────────────────────────────── #

    def _handle_location_reply(self, text: str) -> bool:
        """
        Parse room from Grandpa's location reply.
        Never accepts coordinates or empty fallback.
        Retries up to MAX_LOCATION_RETRIES before aborting.
        """
        room = _parse_room_from_text(text, self.house_config)
        known_rooms = self.house_config.list_targets() if self.house_config else []

        if room and room in known_rooms:
            print(f"{LOG_PREFIX} Location received: {room}")
            self._location_timeout_start = None
            self._execute_escort_from_room(room, self._pending_destination)
            return True

        # Not recognized
        self._location_retry_count += 1
        rooms_str = self._rooms_str()
        print(f"{LOG_PREFIX} Unrecognized location reply '{text}' "
              f"(attempt {self._location_retry_count}/{MAX_LOCATION_RETRIES})")

        if self._location_retry_count >= MAX_LOCATION_RETRIES:
            self._say(
                f"I could not determine your location after {MAX_LOCATION_RETRIES} attempts. "
                f"Please try again later."
            )
            self._reset()
        else:
            self._say(
                f"I did not recognize that room. "
                f"Please say {rooms_str}."
            )
            # Reset per-retry timeout window
            self._location_timeout_start = self.robot.getTime()

        return True  # always consume while in dialogue state

    # ── Core escort execution ──────────────────────────────────────────── #

    def _execute_escort_from_room(self, user_room: str, destination: Optional[str]) -> None:
        """
        Blocked escort execution — only called after valid room confirmed.

        Step 1: navigate(user_room)        [symbolic, no coordinates]
        Step 2: say "I am here."
        Step 3: navigate(destination)      [symbolic, door-aware via brain]
                — skipped if destination is None (bare come_request)
        Step 4: say arrival message
        """
        user_room_readable = user_room.replace("_", " ")

        # ── Step 1: navigate to user's declared room ────────────────── #
        self._transition(AssistiveState.NAVIGATING_TO_USER)
        result = self.executor.execute(("navigate", user_room))

        if result.status != "success":
            self._say(f"I could not reach the {user_room_readable}. I am sorry.")
            self._reset()
            return

        print(f"{LOG_PREFIX} Arrived at {user_room}")
        self._say("I am here.")

        # ── Step 2: escort (skipped for bare come_request) ─────────── #
        if destination is None:
            self._transition(AssistiveState.COMPLETED)
            self._reset()
            return

        dest_readable = destination.replace("_", " ")
        self._transition(AssistiveState.ESCORTING)
        self._say(f"Please follow me to the {dest_readable}.")

        if self.brain is not None:
            success = self.brain.handle_command(f"go to {destination}")
        else:
            print(f"{LOG_PREFIX} WARNING: brain not set — executor fallback (no door handling)")
            fallback = self.executor.execute(("navigate", destination))
            success = fallback.status == "success"

        # ── Step 3: arrival ─────────────────────────────────────────── #
        self._transition(AssistiveState.COMPLETED)
        if success:
            self._say(f"We have arrived at the {dest_readable}.")
            print(f"{LOG_PREFIX} ✓ ESCORT COMPLETED — {destination}")
        else:
            self._say(f"I had trouble reaching the {dest_readable}. I am sorry.")
            print(f"{LOG_PREFIX} ✗ ESCORT FAILED — {destination}")

        self._reset()

    # ── Legacy: bring_object ───────────────────────────────────────────── #

    def _execute_bring_object(self) -> None:
        object_name = self.current_object
        room = get_object_location(object_name)

        if room is None:
            print(f"{LOG_PREFIX} ERROR: Unknown object '{object_name}'")
            return

        print(f"{LOG_PREFIX} Task: Bring {object_name} from {room} to Grandpa")

        self._transition(AssistiveState.NAVIGATING_TO_ROOM)
        result = self.executor.execute(("navigate", room))
        if result.status != "success":
            print(f"{LOG_PREFIX} ERROR: Failed to reach {room}")
            return

        print(f"{LOG_PREFIX} ✓ At {room}")
        self._transition(AssistiveState.SIMULATING_PICKUP)
        time.sleep(1.5)
        print(f"{LOG_PREFIX} ✓ Picked up {object_name}")

        # Coordinate nav is acceptable here — delivery to known position.
        gx, gy, gz = self.grandpa_position
        print(f"{LOG_PREFIX} Delivering to Grandpa at ({gx:.2f}, {gy:.2f})")
        self._transition(AssistiveState.NAVIGATING_TO_USER_LEGACY)

        result = self.executor.execute(("navigate_to_coords", gx, gy, gz, 1.0))
        if result.status != "success":
            print(f"{LOG_PREFIX} ERROR: Failed to reach Grandpa coordinates")
            return

        print(f"{LOG_PREFIX} ✓ Delivered {object_name}")
        self._transition(AssistiveState.COMPLETED)
        self._reset()

    # ── Helpers ────────────────────────────────────────────────────────── #

    def _transition(self, new_state: AssistiveState) -> None:
        print(f"{LOG_PREFIX} State → {new_state.value.upper()}")
        self.state = new_state

    def _reset(self) -> None:
        self.state = AssistiveState.IDLE
        self._pending_destination = None
        self._location_retry_count = 0
        self._location_timeout_start = None

    def _rooms_str(self) -> str:
        if self.house_config is None:
            return "kitchen, bedroom, bathroom, living room, sofa"
        rooms = self.house_config.list_targets()
        return ", ".join(r.replace("_", " ") for r in rooms)
