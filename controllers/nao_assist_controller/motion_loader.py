"""
Motion Loader - Robust loading of Webots motion files.

This module provides a helper to load motion files relative to the controller
directory, with proper path resolution, existence checks, and VALIDATION that
wbu_motion_new actually succeeded (not just that file exists).

IMPORTANT: Webots Motion() constructor does NOT return None when wbu_motion_new
fails - it returns a Motion object that wraps a NULL pointer. We must validate
by calling getDuration() to detect invalid motions.

WINDOWS PATH FIX: Webots wbu_motion_new() may fail with non-ASCII paths (e.g.,
"3º year"). This module implements fallback strategies:
1. Try relative path (Webots resolves relative to controller CWD)
2. Try absolute path
3. If path contains non-ASCII, copy files to ASCII-safe temp directory
"""

from pathlib import Path
from typing import Dict, Optional, Any, Tuple
import os
import shutil
import tempfile

# Controller directory - resolved once at import time
CONTROLLER_DIR = Path(__file__).resolve().parent
MOTIONS_DIR = CONTROLLER_DIR / "motions"

# Expected motion files - core locomotion
MOTION_FILES = {
    "forward": "Forwards50.motion",
    "turn_left": "TurnLeft40.motion",
    "turn_right": "TurnRight40.motion",
}

# Stand-up motion files - optional for fall recovery
# These are available in Webots NAO robot resources:
# WEBOTS_HOME/projects/robots/softbank/nao/motions/
STANDUP_MOTION_FILES = {
    "standup_front": "StandUpFromFront.motion",
    "standup_back": "StandUpFromBack.motion",
}

LOG_PREFIX = "[MOTION_LOADER]"

# Cache for ASCII-safe temp directory (created once if needed)
_ascii_safe_motions_dir: Optional[Path] = None


def _has_non_ascii(path: str) -> bool:
    """Check if path contains non-ASCII characters."""
    try:
        path.encode('ascii')
        return False
    except UnicodeEncodeError:
        return True


def _get_ascii_safe_motions_dir() -> Path:
    """
    Create or return an ASCII-safe temp directory with motion files.
    
    Copies motion files to a temp directory with only ASCII characters in path.
    This works around Webots wbu_motion_new failing on non-ASCII Windows paths.
    """
    global _ascii_safe_motions_dir
    
    if _ascii_safe_motions_dir is not None and _ascii_safe_motions_dir.exists():
        return _ascii_safe_motions_dir
    
    # Create temp dir in system temp (guaranteed ASCII on Windows)
    temp_base = Path(tempfile.gettempdir()) / "webots_nao_motions"
    temp_base.mkdir(exist_ok=True)
    
    print(f"{LOG_PREFIX} Creating ASCII-safe motion directory: {temp_base}")
    
    # Copy all locomotion motion files
    for filename in MOTION_FILES.values():
        src = MOTIONS_DIR / filename
        dst = temp_base / filename
        if src.exists():
            shutil.copy2(src, dst)
            print(f"{LOG_PREFIX}   Copied: {filename}")
    
    # Copy all stand-up motion files (for fall recovery)
    for filename in STANDUP_MOTION_FILES.values():
        src = MOTIONS_DIR / filename
        dst = temp_base / filename
        if src.exists():
            shutil.copy2(src, dst)
            print(f"{LOG_PREFIX}   Copied: {filename}")
    
    _ascii_safe_motions_dir = temp_base
    return temp_base


def get_motion_path(filename: str) -> Path:
    """
    Get the absolute path to a motion file.
    
    Args:
        filename: Motion filename (e.g., "Forwards50.motion")
    
    Returns:
        Absolute Path object to the motion file.
    """
    return MOTIONS_DIR / filename


def is_motion_valid(motion) -> bool:
    """
    Check if a Motion object is valid (not wrapping a NULL pointer).
    
    Webots Motion() constructor returns an object even when wbu_motion_new fails.
    We detect this by calling getDuration() - invalid motions return 0 or raise.
    
    Args:
        motion: Webots Motion object to validate
    
    Returns:
        True if motion is valid and usable, False otherwise.
    """
    if motion is None:
        return False
    
    try:
        # getDuration() returns 0 for invalid/NULL motions
        duration = motion.getDuration()
        # A valid motion should have duration > 0
        return duration is not None and duration > 0
    except Exception:
        return False


def _try_load_motion_with_path(Motion, path_str: str) -> Tuple[Any, bool]:
    """
    Attempt to load a motion with a given path string.
    
    Returns:
        Tuple of (motion_object_or_None, is_valid)
    """
    try:
        motion = Motion(path_str)
        valid = is_motion_valid(motion)
        return (motion if valid else None, valid)
    except Exception:
        return (None, False)


def load_motion(filename: str) -> Any:
    """
    Load a single motion file with fallback path strategies.
    
    Tries multiple loading strategies:
    1. Relative path (e.g., "motions/Forwards50.motion") - Webots CWD is controller dir
    2. Absolute path
    3. ASCII-safe temp directory (for non-ASCII paths on Windows)
    
    Args:
        filename: Motion filename (e.g., "Forwards50.motion")
    
    Returns:
        Webots Motion object (validated as working).
    
    Raises:
        FileNotFoundError: If the motion file does not exist.
        RuntimeError: If all loading strategies fail.
        ImportError: If Webots Motion class is not available.
    """
    from controller import Motion
    
    abs_path = get_motion_path(filename)
    rel_path = f"motions/{filename}"
    
    print(f"{LOG_PREFIX} Loading motion: {filename}")
    print(f"{LOG_PREFIX}   Absolute path: {abs_path}")
    print(f"{LOG_PREFIX}   Relative path: {rel_path}")
    print(f"{LOG_PREFIX}   File exists: {abs_path.exists()}")
    print(f"{LOG_PREFIX}   Path has non-ASCII: {_has_non_ascii(str(abs_path))}")
    
    if not abs_path.exists():
        print(f"{LOG_PREFIX}   ERROR: File not found!")
        raise FileNotFoundError(
            f"Motion file not found: {abs_path}\n"
            f"Expected location: {MOTIONS_DIR}\n"
            f"Please ensure the motions/ folder exists and contains {filename}"
        )
    
    # Strategy 1: Try relative path first (Webots sets CWD to controller dir)
    print(f"{LOG_PREFIX}   Trying relative path: {rel_path}")
    motion, valid = _try_load_motion_with_path(Motion, rel_path)
    if valid:
        duration = motion.getDuration()
        print(f"{LOG_PREFIX}   SUCCESS (relative path) - duration: {duration}ms")
        return motion
    print(f"{LOG_PREFIX}   Relative path failed")
    
    # Strategy 2: Try absolute path
    print(f"{LOG_PREFIX}   Trying absolute path: {abs_path}")
    motion, valid = _try_load_motion_with_path(Motion, str(abs_path))
    if valid:
        duration = motion.getDuration()
        print(f"{LOG_PREFIX}   SUCCESS (absolute path) - duration: {duration}ms")
        return motion
    print(f"{LOG_PREFIX}   Absolute path failed")
    
    # Strategy 3: If path has non-ASCII, try ASCII-safe temp directory
    if _has_non_ascii(str(abs_path)):
        print(f"{LOG_PREFIX}   Path contains non-ASCII characters - trying ASCII-safe workaround")
        ascii_safe_dir = _get_ascii_safe_motions_dir()
        ascii_safe_path = ascii_safe_dir / filename
        
        if ascii_safe_path.exists():
            print(f"{LOG_PREFIX}   Trying ASCII-safe path: {ascii_safe_path}")
            motion, valid = _try_load_motion_with_path(Motion, str(ascii_safe_path))
            if valid:
                duration = motion.getDuration()
                print(f"{LOG_PREFIX}   SUCCESS (ASCII-safe path) - duration: {duration}ms")
                return motion
            print(f"{LOG_PREFIX}   ASCII-safe path also failed")
    
    # All strategies failed
    raise RuntimeError(
        f"wbu_motion_new() failed for file: {filename}\n"
        f"The file exists at {abs_path} but Webots could not load it.\n"
        f"All loading strategies (relative, absolute, ASCII-safe) failed.\n"
        f"Possible causes:\n"
        f"  1. Path encoding issue (non-ASCII characters)\n"
        f"  2. Motion file format incompatible with this Webots version\n"
        f"  3. Robot model doesn't match the motion joint names"
    )


def load_all_motions() -> Dict[str, Any]:
    """
    Load all expected motion files.
    
    Returns:
        Dictionary with keys 'forward', 'turn_left', 'turn_right' mapping to Motion objects.
    
    Raises:
        FileNotFoundError: If any motion file is missing.
        RuntimeError: If any Motion load fails (NULL pointer).
    """
    print(f"{LOG_PREFIX} Loading all motions from: {MOTIONS_DIR}")
    
    motions = {}
    for key, filename in MOTION_FILES.items():
        motions[key] = load_motion(filename)
    
    print(f"{LOG_PREFIX} All {len(motions)} motions loaded and VALIDATED successfully")
    return motions


def try_load_standup_motions() -> Dict[str, Any]:
    """
    Attempt to load stand-up motions for fall recovery.
    
    Stand-up motions are OPTIONAL - returns empty dict if not available.
    These motions can be found in Webots NAO robot resources:
    WEBOTS_HOME/projects/robots/softbank/nao/motions/
    
    Returns:
        Dictionary with available stand-up motions:
        - 'standup_front': Motion for recovering from front fall
        - 'standup_back': Motion for recovering from back fall
        Empty dict if no stand-up motions available.
    """
    print(f"{LOG_PREFIX} Checking for stand-up motions...")
    
    try:
        from controller import Motion
    except ImportError:
        print(f"{LOG_PREFIX}   Webots Motion class not available")
        return {}
    
    standup_motions = {}
    
    for key, filename in STANDUP_MOTION_FILES.items():
        abs_path = get_motion_path(filename)
        rel_path = f"motions/{filename}"
        
        if not abs_path.exists():
            print(f"{LOG_PREFIX}   [{key}] {filename} - NOT FOUND (optional)")
            continue
        
        print(f"{LOG_PREFIX}   [{key}] {filename} - found, loading...")
        
        motion = None
        valid = False
        
        # Try relative path
        motion, valid = _try_load_motion_with_path(Motion, rel_path)
        
        # Try absolute path
        if not valid:
            motion, valid = _try_load_motion_with_path(Motion, str(abs_path))
        
        # Try ASCII-safe path (needed for non-ASCII paths like "3º year")
        if not valid and _has_non_ascii(str(abs_path)):
            ascii_safe_dir = _get_ascii_safe_motions_dir()
            ascii_safe_path = ascii_safe_dir / filename
            
            # Copy the stand-up motion file if not already in temp dir
            if not ascii_safe_path.exists() and abs_path.exists():
                shutil.copy2(abs_path, ascii_safe_path)
                print(f"{LOG_PREFIX}   Copied to ASCII-safe dir: {filename}")
            
            if ascii_safe_path.exists():
                motion, valid = _try_load_motion_with_path(Motion, str(ascii_safe_path))
        
        if valid and motion is not None:
            duration = motion.getDuration()
            standup_motions[key] = motion
            print(f"{LOG_PREFIX}   [{key}] LOADED - duration: {duration}ms")
        else:
            print(f"{LOG_PREFIX}   [{key}] LOAD FAILED (file exists but could not load)")
    
    if standup_motions:
        print(f"{LOG_PREFIX} Stand-up motions available: {list(standup_motions.keys())}")
    else:
        print(f"{LOG_PREFIX} No stand-up motions available")
        print(f"{LOG_PREFIX} To enable fall recovery, copy from Webots:")
        print(f"{LOG_PREFIX}   WEBOTS_HOME/projects/robots/softbank/nao/motions/")
        print(f"{LOG_PREFIX}   - StandUpFromFront.motion")
        print(f"{LOG_PREFIX}   - StandUpFromBack.motion")
    
    return standup_motions


def list_available_motions() -> Dict[str, bool]:
    """
    List all expected motion files and their availability.
    
    Returns:
        Dict mapping motion key to whether file exists.
    """
    available = {}
    
    for key, filename in MOTION_FILES.items():
        available[key] = get_motion_path(filename).exists()
    
    for key, filename in STANDUP_MOTION_FILES.items():
        available[key] = get_motion_path(filename).exists()
    
    return available


def try_load_all_motions() -> tuple[Dict[str, Any], bool, Optional[str]]:
    """
    Attempt to load all motions with fallback strategies, returning success status.
    
    This is useful for graceful degradation when motions aren't available.
    Uses multiple loading strategies for each motion:
    1. Relative path
    2. Absolute path  
    3. ASCII-safe temp directory (for non-ASCII Windows paths)
    
    Returns:
        Tuple of:
        - Dictionary of loaded motions (may be partially filled or empty)
        - Boolean indicating if ALL motions loaded successfully
        - Error message string if loading failed, None if success
    """
    import os
    
    print(f"{LOG_PREFIX} " + "=" * 60)
    print(f"{LOG_PREFIX} MOTION LOADING - DIAGNOSTICS")
    print(f"{LOG_PREFIX} " + "=" * 60)
    print(f"{LOG_PREFIX} os.getcwd():     {os.getcwd()}")
    print(f"{LOG_PREFIX} Controller dir:  {CONTROLLER_DIR}")
    print(f"{LOG_PREFIX} Motions dir:     {MOTIONS_DIR}")
    print(f"{LOG_PREFIX} Motions exists:  {MOTIONS_DIR.exists()}")
    print(f"{LOG_PREFIX} Path has non-ASCII: {_has_non_ascii(str(CONTROLLER_DIR))}")
    
    # Try to get Webots version
    try:
        from controller import Robot
        # Can't easily get version without robot instance, but log we're in Webots context
        print(f"{LOG_PREFIX} Webots controller module: available")
    except ImportError:
        print(f"{LOG_PREFIX} Webots controller module: NOT AVAILABLE")
    
    print(f"{LOG_PREFIX} " + "-" * 60)
    
    motions = {}
    errors = []
    
    try:
        from controller import Motion
    except ImportError as e:
        error_msg = f"Webots Motion class not available: {e}"
        print(f"{LOG_PREFIX} ERROR: {error_msg}")
        return {}, False, error_msg
    
    for key, filename in MOTION_FILES.items():
        abs_path = get_motion_path(filename)
        rel_path = f"motions/{filename}"
        
        print(f"{LOG_PREFIX} [{key}] {filename}")
        print(f"{LOG_PREFIX}   Absolute: {abs_path}")
        print(f"{LOG_PREFIX}   Relative: {rel_path}")
        print(f"{LOG_PREFIX}   Exists:   {abs_path.exists()}")
        
        if not abs_path.exists():
            errors.append(f"{filename}: file not found at {abs_path}")
            print(f"{LOG_PREFIX}   RESULT: FILE NOT FOUND")
            continue
        
        motion = None
        load_method = None
        
        # Strategy 1: Relative path
        motion, valid = _try_load_motion_with_path(Motion, rel_path)
        if valid:
            load_method = "relative"
        
        # Strategy 2: Absolute path
        if not valid:
            motion, valid = _try_load_motion_with_path(Motion, str(abs_path))
            if valid:
                load_method = "absolute"
        
        # Strategy 3: ASCII-safe temp directory
        if not valid and _has_non_ascii(str(abs_path)):
            ascii_safe_dir = _get_ascii_safe_motions_dir()
            ascii_safe_path = ascii_safe_dir / filename
            if ascii_safe_path.exists():
                motion, valid = _try_load_motion_with_path(Motion, str(ascii_safe_path))
                if valid:
                    load_method = "ascii-safe"
        
        if valid and motion is not None:
            duration = motion.getDuration()
            motions[key] = motion
            print(f"{LOG_PREFIX}   RESULT: OK ({load_method}, duration={duration}ms)")
        else:
            errors.append(f"{filename}: wbu_motion_new failed (file exists but Webots can't load it)")
            print(f"{LOG_PREFIX}   RESULT: LOAD FAILED - all strategies exhausted")
    
    print(f"{LOG_PREFIX} " + "-" * 60)
    
    if errors:
        error_msg = "Motion loading failed:\n" + "\n".join(f"  - {e}" for e in errors)
        print(f"{LOG_PREFIX} FAILED: {len(errors)} error(s)")
        for e in errors:
            print(f"{LOG_PREFIX}   - {e}")
        return motions, False, error_msg
    
    print(f"{LOG_PREFIX} SUCCESS: All {len(motions)} motions loaded and validated")
    return motions, True, None


def sanity_check() -> None:
    """
    Print diagnostic information about motion files and attempt to load them.
    
    Prints:
    - Working directory, controller directory, motions directory
    - Path encoding status (ASCII-safe or not)
    - Each expected motion file path and whether it exists
    - Attempts to load each motion with fallback strategies
    """
    import os
    
    print("\n" + "=" * 70)
    print("MOTION LOADER SANITY CHECK")
    print("=" * 70)
    print(f"os.getcwd():          {os.getcwd()}")
    print(f"Controller directory: {CONTROLLER_DIR}")
    print(f"Motions directory:    {MOTIONS_DIR}")
    print(f"Motions dir exists:   {MOTIONS_DIR.exists()}")
    print(f"Path has non-ASCII:   {_has_non_ascii(str(CONTROLLER_DIR))}")
    
    if _has_non_ascii(str(CONTROLLER_DIR)):
        print()
        print("WARNING: Path contains non-ASCII characters!")
        print("This may cause wbu_motion_new() to fail on Windows.")
        print("The loader will try to work around this by copying motions")
        print("to an ASCII-safe temp directory.")
    
    print()
    
    # First check file existence
    print("Step 1: Checking file existence...")
    all_exist = True
    for key, filename in MOTION_FILES.items():
        path = get_motion_path(filename)
        exists = path.exists()
        status = "EXISTS" if exists else "MISSING"
        print(f"  [{key:10}] {filename:20} -> {path}")
        print(f"              Status: {status}")
        if not exists:
            all_exist = False
    
    print()
    if not all_exist:
        print("STATUS: MISSING FILES!")
        print()
        print("TO FIX: Ensure the following structure exists:")
        print(f"  {MOTIONS_DIR}/")
        for filename in MOTION_FILES.values():
            print(f"    {filename}")
        print()
        print("You can copy motion files from Webots NAO robot resources or")
        print("use the files in: legacy/nao_voice_nav_old/motions/")
        print("=" * 70 + "\n")
        return
    
    # Step 2: Try to actually load motions with fallback strategies
    print("Step 2: Attempting to load motions with Webots Motion API...")
    print("(Trying: relative path -> absolute path -> ASCII-safe path)")
    print()
    
    try:
        from controller import Motion
    except ImportError as e:
        print(f"ERROR: Cannot import Motion class: {e}")
        print("This sanity check must be run inside a Webots controller context.")
        print("=" * 70 + "\n")
        return
    
    all_valid = True
    for key, filename in MOTION_FILES.items():
        abs_path = get_motion_path(filename)
        rel_path = f"motions/{filename}"
        
        print(f"  [{key:10}] {filename}")
        
        motion = None
        load_method = None
        
        # Try relative path
        motion, valid = _try_load_motion_with_path(Motion, rel_path)
        if valid:
            load_method = "relative"
        
        # Try absolute path
        if not valid:
            motion, valid = _try_load_motion_with_path(Motion, str(abs_path))
            if valid:
                load_method = "absolute"
        
        # Try ASCII-safe path
        if not valid and _has_non_ascii(str(abs_path)):
            ascii_safe_dir = _get_ascii_safe_motions_dir()
            ascii_safe_path = ascii_safe_dir / filename
            if ascii_safe_path.exists():
                motion, valid = _try_load_motion_with_path(Motion, str(ascii_safe_path))
                if valid:
                    load_method = "ascii-safe"
        
        if valid and motion is not None:
            duration = motion.getDuration()
            print(f"              VALID ({load_method}) - duration: {duration}ms")
        else:
            print(f"              FAILED - all loading strategies failed")
            all_valid = False
    
    print()
    if all_valid:
        print("STATUS: All motion files loaded successfully!")
    else:
        print("STATUS: MOTION LOADING FAILED!")
        print()
        print("The files exist but Webots wbu_motion_new() could not load them.")
        print()
        print("Possible causes:")
        print("  1. Path encoding issue (non-ASCII characters like 'º')")
        print("  2. Motion file format incompatible with this Webots version")
        print("  3. Robot model doesn't match the motion joint names")
        print()
        print("Solutions:")
        if _has_non_ascii(str(CONTROLLER_DIR)):
            print("  >>> PATH ENCODING ISSUE DETECTED <<<")
            print("  Your path contains non-ASCII: e.g., '3º year'")
            print("  Option A: Move project to ASCII-only path (e.g., C:\\Projects\\NAO)")
            print("  Option B: The loader should auto-copy to temp dir (check logs)")
            print()
        print("  1. Get fresh motion files from Webots installation:")
        print("     WEBOTS_HOME/projects/robots/softbank/nao/motions/")
        print("  2. Ensure motions match NAO robot model")
    
    print("=" * 70 + "\n")


if __name__ == "__main__":
    # Run sanity check when executed directly
    sanity_check()
