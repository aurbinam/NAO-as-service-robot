"""
Layer 1: GenAI Natural Language Interpretation.

Converts free-form user commands into structured intent dicts
using the Claude API. Falls back to keyword matching if unavailable.
"""

import json
import os
from typing import Any, Dict, List, Optional

LOG_PREFIX = "[INTERPRETER]"

_SYSTEM_PROMPT = """\
You are the command interpreter for a service robot (NAO) in a house.

Parse the user command into a structured JSON intent.

Available rooms: {rooms}
Available doors: {doors}

Output rules - return ONLY valid JSON, no markdown, no explanation:

Single navigation:
  {{"intent": "navigate", "targets": ["<room_id>"]}}

Multi-step navigation:
  {{"intent": "multi_step", "steps": [{{"action": "navigate", "target": "<room_id>"}}, ...]}}

Door operation:
  {{"intent": "door_operation", "action": "open"|"close", "door": "<door_label>"}}

Unrecognised:
  {{"intent": "unknown", "raw": "<original text>"}}

Map colloquial names to IDs (e.g. "the kitchen" -> "kitchen").
Use only room IDs from the available rooms list.
"""


class GenAIInterpreter:
    """
    Natural language -> structured intent via Claude API.
    Pure translation layer: no planning, no execution.
    """

    def __init__(self, rooms: List[str], doors: List[str]):
        self._rooms = rooms
        self._doors = doors
        self._client = None
        self._system_prompt = _SYSTEM_PROMPT.format(rooms=rooms, doors=doors)
        self._init_client()

    def _init_client(self):
        try:
            import anthropic
            api_key = os.environ.get("ANTHROPIC_API_KEY", "")
            if api_key:
                self._client = anthropic.Anthropic(api_key=api_key)
                print(f"{LOG_PREFIX} Claude API client initialised.")
            else:
                print(f"{LOG_PREFIX} WARNING: ANTHROPIC_API_KEY not set - using keyword fallback.")
        except ImportError:
            print(f"{LOG_PREFIX} WARNING: anthropic package not installed - using keyword fallback.")

    def interpret(self, user_input: str) -> Dict[str, Any]:
        """
        Parse natural language -> intent dict.

        Returns one of:
            {"intent": "navigate",       "targets": ["kitchen"]}
            {"intent": "multi_step",     "steps": [{"action": "navigate", "target": "..."}]}
            {"intent": "door_operation", "action": "open"|"close", "door": "<label>"}
            {"intent": "unknown",        "raw": "<original text>"}
        """
        if self._client is not None:
            result = self._call_api(user_input)
            if result is not None:
                print(f"{LOG_PREFIX} API intent: {result}")
                return result

        result = self._keyword_fallback(user_input)
        print(f"{LOG_PREFIX} Fallback intent: {result}")
        return result

    def _call_api(self, user_input: str) -> Optional[Dict[str, Any]]:
        try:
            response = self._client.messages.create(
                model="claude-haiku-4-5-20251001",
                max_tokens=256,
                system=self._system_prompt,
                messages=[{"role": "user", "content": user_input}],
            )
            raw = response.content[0].text.strip()
            # Strip accidental markdown fences
            if raw.startswith("```"):
                raw = raw.split("```")[1]
                if raw.startswith("json"):
                    raw = raw[4:]
            return json.loads(raw.strip())
        except Exception as exc:
            print(f"{LOG_PREFIX} API call failed: {exc}")
            return None

    def _keyword_fallback(self, user_input: str) -> Dict[str, Any]:
        """Rule-based fallback when API is unavailable."""
        lower = user_input.lower()
        matched = [r for r in self._rooms if r.lower().replace("_", " ") in lower]
        if len(matched) == 1:
            return {"intent": "navigate", "targets": [matched[0]]}
        if len(matched) > 1:
            return {
                "intent": "multi_step",
                "steps": [{"action": "navigate", "target": r} for r in matched],
            }
        for door in self._doors:
            if door.lower() in lower:
                action = "close" if "close" in lower else "open"
                return {"intent": "door_operation", "action": action, "door": door}
        return {"intent": "unknown", "raw": user_input}
