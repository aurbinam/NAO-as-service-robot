"""
Fast-path voice command parser for the NAO Assist Controller.

This module is the *synchronous fallback* interpreter — it uses regex
patterns and fuzzy string matching to classify voice commands without
any API call.  It runs before the AI brain so that high-confidence
commands (exact phrase matches) are dispatched immediately.

For ambiguous or complex commands the result is passed to
``intelligence.genai_interpreter.GenAIInterpreter`` which uses the
Claude API for intent resolution.

Public API
----------
parse_command(text, house_config) -> (cmd_type, cmd_arg)
    Classify a normalised voice command.  Returns one of:
      ("GO_TO",           "<room_id>")
      ("GUIDE_TO",        "<room_id>")
      ("OPEN_DOOR",       "<door_label>")
      ("CLOSE_DOOR",      "<door_label>")
      ("LIST_PLACES",     None)
      ("GO_TO_UNKNOWN",   "<raw text>")
      ("GUIDE_TO_UNKNOWN","<raw text>")
      (None,              None)          # unrecognised

resolve_navigation_target(candidate, house_config) -> Optional[str]
    Resolve a raw place name to a canonical room ID.

fuzzy_match_target(candidate, options, cutoff) -> Optional[str]
    Fuzzy string match against a list of target names.
"""

import re
import difflib
from typing import List, Optional, Tuple

# Filler words stripped from raw voice input before pattern matching
_FILLER_PATTERNS = [
    r"\bplease\b",
    r"\bcan you\b",
    r"\bcould you\b",
    r"\bwould you\b",
    r"\bi want to\b",
    r"\bi would like to\b",
    r"\bnao\b",
    r"\bhey nao\b",
]

# Pattern tables
LIST_PATTERNS = [
    "list places",
    "show places",
    "what places",
    "where can you go",
    "list locations",
    "show locations",
    "available places",
]

GUIDE_TO_PATTERNS = [
    r"help\s+me\s+go\s+to\s+(?:the\s+)?(.+)",
    r"help\s+me\s+to\s+(?:the\s+)?(.+)",
    r"guide\s+me\s+to\s+(?:the\s+)?(.+)",
    r"lead\s+me\s+to\s+(?:the\s+)?(.+)",
    r"escort\s+me\s+to\s+(?:the\s+)?(.+)",
    r"assist\s+me\s+(?:to\s+)?(?:go\s+to\s+)?(?:the\s+)?(.+)",
]

GO_PATTERNS = [
    r"go\s+to\s+(?:the\s+)?(.+)",
    r"navigate\s+to\s+(?:the\s+)?(.+)",
    r"take\s+me\s+to\s+(?:the\s+)?(.+)",
    r"walk\s+to\s+(?:the\s+)?(.+)",
    r"head\s+to\s+(?:the\s+)?(.+)",
    r"bring\s+me\s+to\s+(?:the\s+)?(.+)",
    r"move\s+to\s+(?:the\s+)?(.+)",
]

# Hints used to score navigation intent when no pattern matches
_NAVIGATION_HINTS = [
    "go", "navigate", "walk", "take me",
    "bring me", "move", "head to", "guide me",
]

# Fallback map for common place names when house_config is unavailable
_COMMON_PLACES = {
    "kitchen":    "kitchen",
    "bedroom":    "bedroom",
    "living room": "living_room",
    "bathroom":   "bathroom",
    "sofa":       "sofa",
    "home":       "home",
}


# Public helpers

def normalize_command_text(text: str) -> str:
    """Strip punctuation and filler words; lower-case and collapse whitespace."""
    text = text.strip().lower()
    text = re.sub(r"[^\w\s]", " ", text)
    for pattern in _FILLER_PATTERNS:
        text = re.sub(pattern, " ", text)
    return re.sub(r"\s+", " ", text).strip()


def fuzzy_match_target(
    candidate: str,
    options: List[str],
    cutoff: float = 0.72,
) -> Optional[str]:
    """
    Return the canonical room ID that best matches *candidate*, or None.

    Checks both underscore and space variants (e.g. "living_room" / "living room")
    so callers do not need to normalise the option list beforehand.
    """
    if not candidate or not options:
        return None

    candidate = candidate.strip().lower()
    lookup: dict = {}
    variants: List[str] = []

    for opt in options:
        canonical = opt.strip().lower()
        human = canonical.replace("_", " ")
        for variant in {canonical, human}:
            lookup[variant] = canonical
            variants.append(variant)

    match = difflib.get_close_matches(candidate, variants, n=1, cutoff=cutoff)
    return lookup[match[0]] if match else None


def resolve_navigation_target(
    candidate: str,
    house_config=None,
) -> Optional[str]:
    """
    Resolve a raw place-name string to a canonical room ID.

    Resolution order:
      1. Direct lookup via house_config.resolve_target_id
      2. Fuzzy match against house_config.list_targets()
      3. Common-place fallback table
      4. Fuzzy match against common-place table
    """
    if not candidate:
        return None

    candidate = candidate.strip().lower()
    # Strip trailing filler words that survive normalization
    candidate = re.sub(r"\bplease\b$", "", candidate).strip()
    candidate = re.sub(r"\bnow\b$",    "", candidate).strip()

    variants = [candidate, candidate.replace(" ", "_"), candidate.replace("_", " ")]

    if house_config is not None:
        for variant in variants:
            resolved = house_config.resolve_target_id(variant)
            if resolved:
                return resolved
        fuzzy = fuzzy_match_target(candidate, house_config.list_targets())
        if fuzzy:
            return fuzzy

    if candidate in _COMMON_PLACES:
        return _COMMON_PLACES[candidate]

    return fuzzy_match_target(candidate, list(_COMMON_PLACES.values()))


def _resolve_door_label(candidate: str, house_config=None) -> str:
    """Resolve a raw door-name fragment to a known door label, or return as-is."""
    if not candidate:
        return candidate

    candidate = candidate.strip().lower()
    candidate = re.sub(r"\bplease\b$", "", candidate).strip()
    candidate = re.sub(r"\bnow\b$",    "", candidate).strip()

    if house_config is not None:
        try:
            doors = [d.get("label", "").strip().lower() for d in house_config.get_all_doors()]
            doors = [d for d in doors if d]
            if candidate in doors:
                return candidate
            match = difflib.get_close_matches(candidate, doors, n=1, cutoff=0.72)
            if match:
                return match[0]
        except Exception:
            pass

    return candidate


def _navigation_intent_score(text: str) -> float:
    """Heuristic score [0, 1] for how likely text is a navigation command."""
    score = sum(0.2 for hint in _NAVIGATION_HINTS if hint in text)
    return min(score, 1.0)


# Main classifier

def parse_command(
    text: str,
    house_config=None,
) -> Tuple[Optional[str], Optional[str]]:
    """
    Classify a voice command into a (cmd_type, cmd_arg) pair.

    Returns
    -------
    ("GO_TO",            room_id)     — navigate to room
    ("GUIDE_TO",         room_id)     — escort user to room
    ("OPEN_DOOR",        door_label)  — open named door
    ("CLOSE_DOOR",       door_label)  — close named door
    ("LIST_PLACES",      None)        — list available rooms
    ("GO_TO_UNKNOWN",    raw_text)    — navigation intent, target unclear
    ("GUIDE_TO_UNKNOWN", raw_text)    — escort intent, target unclear
    (None,               None)        — command not recognised
    """
    text = normalize_command_text(text)
    if not text:
        return (None, None)

    # List-places shortcut
    for pattern in LIST_PATTERNS:
        if pattern in text:
            return ("LIST_PLACES", None)

    # Door — open
    for pattern in [r"open\s+(?:the\s+)?(.+?)\s+door$",
                    r"open\s+door\s+(?:to\s+)?(?:the\s+)?(.+)$"]:
        match = re.search(pattern, text)
        if match:
            return ("OPEN_DOOR", _resolve_door_label(match.group(1).strip(), house_config))

    # Door — close
    for pattern in [r"close\s+(?:the\s+)?(.+?)\s+door$",
                    r"shut\s+(?:the\s+)?(.+?)\s+door$"]:
        match = re.search(pattern, text)
        if match:
            return ("CLOSE_DOOR", _resolve_door_label(match.group(1).strip(), house_config))

    # Escort / guide
    for pattern in GUIDE_TO_PATTERNS:
        match = re.match(pattern, text)
        if match:
            resolved = resolve_navigation_target(match.group(1).strip(), house_config)
            return ("GUIDE_TO", resolved) if resolved else ("GUIDE_TO_UNKNOWN", match.group(1).strip())

    # Navigation
    for pattern in GO_PATTERNS:
        match = re.match(pattern, text)
        if match:
            resolved = resolve_navigation_target(match.group(1).strip(), house_config)
            return ("GO_TO", resolved) if resolved else ("GO_TO_UNKNOWN", match.group(1).strip())

    # Bare place name
    resolved = resolve_navigation_target(text, house_config)
    if resolved:
        return ("GO_TO", resolved)

    # Weak navigation signal (e.g. "kitchen" without a verb)
    if _navigation_intent_score(text) >= 0.4:
        return ("GO_TO_UNKNOWN", text)

    return (None, None)