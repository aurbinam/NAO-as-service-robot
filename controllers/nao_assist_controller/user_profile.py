"""
User Profile Management - Persist and load user identity.
Iteration 1: JSON-based profile storage.
"""

import json
import os
from datetime import datetime
from typing import Optional


# Profile path relative to project root
def _get_profile_path() -> str:
    """
    Get the absolute path to user_profile.json.
    
    Path is: <project_root>/data/user_profile.json
    Controller is at: <project_root>/controllers/nao_assist_controller/
    """
    controller_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(controller_dir))
    data_dir = os.path.join(project_root, "data")
    
    # Ensure data directory exists
    os.makedirs(data_dir, exist_ok=True)
    
    return os.path.join(data_dir, "user_profile.json")


class UserProfile:
    """
    User profile data structure.
    
    Fields:
        user_id: Unique identifier (e.g., "user1")
        name: User's display name
        confirmed: Whether the name has been confirmed
        created_at: ISO timestamp of profile creation
        updated_at: ISO timestamp of last update
    """
    
    def __init__(
        self,
        user_id: str = "user1",
        name: str = "",
        confirmed: bool = False,
        created_at: Optional[str] = None,
        updated_at: Optional[str] = None,
        memory: Optional[dict] = None
    ):
        self.user_id = user_id
        self.name = name
        self.confirmed = confirmed
        now = datetime.now().isoformat()
        self.created_at = created_at or now
        self.updated_at = updated_at or now
        self.memory = memory or {}
    
    def to_dict(self) -> dict:
        """Convert profile to dictionary for JSON serialization."""
        return {
            "user_id": self.user_id,
            "name": self.name,
            "confirmed": self.confirmed,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "memory": self.memory
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> "UserProfile":
        """Create profile from dictionary."""
        return cls(
            user_id=data.get("user_id", "user1"),
            name=data.get("name", ""),
            confirmed=data.get("confirmed", False),
            created_at=data.get("created_at"),
            updated_at=data.get("updated_at"),
            memory=data.get("memory", {})
        )
    
    def update_name(self, name: str, confirmed: bool = True) -> None:
        """Update name and set confirmed status."""
        self.name = name
        self.confirmed = confirmed
        self.updated_at = datetime.now().isoformat()


def load_profile() -> Optional[UserProfile]:
    """
    Load user profile from disk.
    
    Returns:
        UserProfile if file exists and is valid, None otherwise.
    """
    profile_path = _get_profile_path()
    
    if not os.path.exists(profile_path):
        return None
    
    try:
        with open(profile_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return UserProfile.from_dict(data)
    except (json.JSONDecodeError, IOError) as e:
        print(f"[UserProfile] Error loading profile: {e}")
        return None


def save_profile(profile: UserProfile) -> bool:
    """
    Save user profile to disk.
    
    Args:
        profile: UserProfile instance to save.
    
    Returns:
        True if save succeeded, False otherwise.
    """
    profile_path = _get_profile_path()
    
    try:
        with open(profile_path, "w", encoding="utf-8") as f:
            json.dump(profile.to_dict(), f, indent=2)
        return True
    except IOError as e:
        print(f"[UserProfile] Error saving profile: {e}")
        return False


def profile_exists_and_confirmed() -> bool:
    """
    Check if a confirmed profile exists.
    
    Returns:
        True if profile exists and is confirmed, False otherwise.
    """
    profile = load_profile()
    return profile is not None and profile.confirmed and bool(profile.name)


def get_or_create_profile() -> UserProfile:
    """
    Get existing profile or create a new empty one.
    
    Returns:
        Existing UserProfile if found, new empty profile otherwise.
    """
    profile = load_profile()
    if profile is None:
        profile = UserProfile()
    return profile
