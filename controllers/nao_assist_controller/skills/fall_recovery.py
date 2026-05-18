"""
Fall Recovery and Stability Manager for joint_gait locomotion fallback.

When the robot is using joint_gait (direct motor control), this module provides:
1. Command ramping: Gradually increase velocity commands from 0 to full over N seconds
   - Prevents abrupt starts that can cause loss of balance
   - Gives gait synthesis time to stabilize
2. Instability detection and response
3. Recovery tracking after falls

Design:
- Each new movement (rotation, walk) starts a ramp timer
- Velocity commands are scaled by ramp factor (0.0 → 1.0) during ramp window
- After ramp completes, commands pass through unmodified
"""

import time


class FallRecoveryManager:
    """Manages command ramping and stability state for joint_gait backend."""
    
    # Configuration constants
    RAMP_DURATION_SEC = 2.5  # Time to ramp from 0 → 100% command
    STABILITY_SETTLING_SEC = 1.0  # Time after fall before allowing new movements
    
    def __init__(self):
        """Initialize the recovery manager."""
        self._movement_start_time = None
        self._fall_time = None
        self._has_fallen = False
        self._is_ramping = False
        
    def start_movement(self):
        """
        Call this when beginning a new movement (rotation or walk).
        Marks the start of a new command ramp cycle.
        """
        self._movement_start_time = time.time()
        self._is_ramping = True
        
    def stop_movement(self):
        """Call this when stopping movement (e.g., at target, or on fall)."""
        self._movement_start_time = None
        self._is_ramping = False
        
    def mark_fall(self):
        """Call this when a fall is detected."""
        self._fall_time = time.time()
        self._has_fallen = True
        self._is_ramping = False
        
    def on_recovery_complete(self):
        """Call this after stand-up motion completes."""
        self._has_fallen = False
        self._fall_time = None
        
    def should_allow_movement(self):
        """
        Check if enough time has passed after a fall before allowing new movements.
        
        Returns:
            bool: True if OK to command movement, False if still settling after fall
        """
        if not self._has_fallen:
            return True
        if self._fall_time is None:
            return True

        time_since_fall = time.time() - self._fall_time
        return time_since_fall >= self.STABILITY_SETTLING_SEC
    
    def get_command_scale(self):
        """
        Get the scale factor (0.0 to 1.0) to apply to velocity commands.
        
        During ramp-up phase, returns gradually increasing scale.
        After ramp completes, returns 1.0.
        
        Returns:
            float: Scale factor in range [0.0, 1.0]
        """
        if not self._is_ramping or self._movement_start_time is None:
            return 1.0
        
        elapsed = time.time() - self._movement_start_time
        
        # Ramp complete
        if elapsed >= self.RAMP_DURATION_SEC:
            self._is_ramping = False
            return 1.0
        
        # Quadratic ease-in: avoids velocity discontinuity at t=0 that linear ramp causes.
        t = elapsed / self.RAMP_DURATION_SEC
        scale = t * t
        return min(scale, 1.0)
    
    def apply_ramp_to_commands(self, x_cmd, theta_cmd):
        """
        Apply command ramping to velocity commands.
        
        Args:
            x_cmd (float): Forward velocity command (typically ±0.12)
            theta_cmd (float): Rotational velocity command (typically ±0.10)
            
        Returns:
            tuple: (x_scaled, theta_scaled) with ramping applied
        """
        scale = self.get_command_scale()
        return (x_cmd * scale, theta_cmd * scale)
    
    def get_ramp_status(self):
        """
        Return diagnostic info about current ramp/recovery state.
        
        Returns:
            dict: Status info (ramp_percent, is_ramping, settled_after_fall, etc.)
        """
        if self._is_ramping and self._movement_start_time is not None:
            elapsed = time.time() - self._movement_start_time
            ramp_pct = min(100 * elapsed / self.RAMP_DURATION_SEC, 100.0)
        else:
            ramp_pct = 100.0
        
        status = {
            'ramping': self._is_ramping,
            'ramp_percent': ramp_pct,
            'has_fallen': self._has_fallen,
            'settled_after_fall': not self._has_fallen or self.should_allow_movement(),
        }
        
        return status
