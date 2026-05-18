"""
SAY Skill — text output via Webots Speaker device or console fallback.

The main controller uses ``NAOAssistController.say()`` for all speech,
which handles timing and device access.  This module provides the
standalone ``execute_say`` helper used by the legacy executor path.
"""

from config import LOG_PREFIX


def execute_say(text: str, robot=None) -> bool:
    """Print *text* to console (Speaker TTS not yet wired up)."""
    print(f'{LOG_PREFIX} SAY: "{text}"')
    return True