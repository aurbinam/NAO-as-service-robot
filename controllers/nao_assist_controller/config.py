"""
Configuration constants for the NAO Assist Controller.
"""

# Webots simulation timestep (ms) — must match world basicTimeStep
TIMESTEP_MS = 32

# TCP SETTINGS (voice_listener.py sends commands via TCP socket)
TCP_HOST = "127.0.0.1"
TCP_PORT = 5005

# IDENTITY SETTINGS
# How long to wait for a TCP name before falling back to keyboard input
NAME_CAPTURE_TIMEOUT_SECONDS = 20

# SPEECH / TTS TIMING SETTINGS
SPEAK_VOLUME = 1.0
SPEECH_SECONDS_PER_CHAR  = 0.085   # base wait per character
MIN_SPEECH_SECONDS       = 2.2     # floor: even short phrases get this minimum
EXTRA_PAUSE_AFTER_LINE   = 0.8     # pause injected after each spoken line
PUNCTUATION_BONUS_PER_MARK = 0.25  # extra time per punctuation mark
PUNCTUATION_MARKS        = ".?!,;:"

# LOGGING
LOG_PREFIX = "[NAO_ASSIST]"
