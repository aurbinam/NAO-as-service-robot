"""
GO_TO_USER Skill — coordinate-based user navigation (stub).

Currently unimplemented. The full assistive workflow uses
``AssistiveStateMachine`` (assistive_tasks.py) which navigates to a
room rather than a live person position.  This stub is retained so
the skill registry in skills/__init__.py can resolve the symbol.
"""

from config import LOG_PREFIX


def execute_go_to_user(user_id: str, robot=None) -> bool:
    """Log and return False (not yet implemented)."""
    print(f"{LOG_PREFIX} GO_TO_USER: '{user_id}' — not implemented")
    return False