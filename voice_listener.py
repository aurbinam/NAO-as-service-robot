"""
Voice Listener for NAO Navigation
Uses sounddevice (works with Python 3.14) + Google Speech API

No PyAudio needed!

FIXED: Uses persistent TCP connection that stays alive.
"""

import io
import socket
import wave
import time
import numpy as np
import sounddevice as sd
import requests

# TCP connection settings - matches NAO controller
HOST = "127.0.0.1"
PORT = 5005

# Audio settings
SAMPLE_RATE = 16000
CHANNELS = 1
DURATION = 4  # seconds to record

# Supported destinations
DESTINATIONS = ["kitchen", "bedroom", "living room", "bathroom", "home", "stop"]

# Global persistent socket
_sock = None


def _ensure_connected():
    """Ensure we have a connection to NAO. Retries until successful."""
    global _sock
    if _sock is not None:
        return True
    
    while _sock is None:
        try:
            _sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            _sock.connect((HOST, PORT))
            print(f"[VoiceListener] Connected to {HOST}:{PORT}")
            return True
        except ConnectionRefusedError:
            print(f"[VoiceListener] Connection refused - is Webots running? Retrying in 1s...")
            _sock = None
            time.sleep(1.0)
        except Exception as e:
            print(f"[VoiceListener] Connection error: {e} - retrying in 1s...")
            _sock = None
            time.sleep(1.0)


def _disconnect():
    """Close socket safely."""
    global _sock
    if _sock:
        try:
            _sock.close()
        except:
            pass
    _sock = None


def _send_message(msg):
    """Send a message, reconnecting if needed."""
    global _sock
    if not msg.endswith('\n'):
        msg += '\n'
    
    _ensure_connected()
    
    try:
        _sock.sendall(msg.encode('utf-8'))
        return True
    except (BrokenPipeError, ConnectionResetError, OSError) as e:
        print(f"[VoiceListener] Send failed: {e} - reconnecting...")
        _disconnect()
        _ensure_connected()
        try:
            _sock.sendall(msg.encode('utf-8'))
            return True
        except Exception as e2:
            print(f"[VoiceListener] Retry failed: {e2}")
            _disconnect()
            return False


def send_command(dest: str):
    """Send a destination command to the NAO controller via TCP"""
    if _send_message(dest):
        print(f"[SENT] {dest}")
        return True
    return False


def send_name_to_nao(name: str):
    """Send captured name to NAO controller via TCP."""
    msg = f"NAME:{name}"
    if _send_message(msg):
        print(f"[VoiceListener] Sent name to NAO: {name}")
        return True
    print(f"[VoiceListener] Failed to send name")
    return False


def record_audio(duration=DURATION):
    """Record audio from microphone using sounddevice"""
    print(f"🎤 Recording for {duration} seconds... SPEAK NOW!")
    audio = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE, 
                   channels=CHANNELS, dtype='int16')
    sd.wait()  # Wait until recording is finished
    print("Processing...")
    return audio


def audio_to_wav(audio_data):
    """Convert numpy array to WAV format in memory"""
    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(2)  # 16-bit = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio_data.tobytes())
    buffer.seek(0)
    return buffer.read()


def recognize_google(audio_data):
    """Send audio to Google Speech Recognition API"""
    wav_data = audio_to_wav(audio_data)
    
    # Google Speech Recognition API (free tier, no API key needed)
    url = "http://www.google.com/speech-api/v2/recognize"
    params = {
        "client": "chromium",
        "lang": "en-US",
        "key": "AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw"  # Public Chromium key
    }
    headers = {"Content-Type": "audio/l16; rate=16000"}
    
    try:
        response = requests.post(url, params=params, headers=headers, data=wav_data, timeout=10)
        
        # Parse response (returns multiple JSON objects)
        for line in response.text.strip().split('\n'):
            if line:
                import json
                try:
                    result = json.loads(line)
                    if 'result' in result and len(result['result']) > 0:
                        alternatives = result['result'][0].get('alternative', [])
                        if alternatives:
                            return alternatives[0].get('transcript', '')
                except json.JSONDecodeError:
                    continue
        return None
    except Exception as e:
        print(f"[API Error] {e}")
        return None


def normalize_command(transcript: str) -> str:
    """
    Normalize a voice transcript into a clean command string.
    
    Preserves intent and converts shortcuts:
      - "please go to kitchen" -> "go to kitchen"
      - "kitchen" (alone) -> "go to kitchen"
      - "go kitchen" -> "go to kitchen"
      - "open the kitchen door" -> "open kitchen door"
      - "list places" -> "list places"
      - "stop" -> "stop"
    
    Args:
        transcript: Raw speech-to-text result
    
    Returns:
        Normalized command string ready for NAO.
    """
    if not transcript:
        return ""
    
    # Lowercase and strip
    text = transcript.lower().strip()
    
    # Remove polite prefixes
    polite_prefixes = [
        "please ", "can you ", "could you ", "would you ",
        "i want you to ", "i need you to ", "i'd like you to ",
        "nao ", "robot ", "hey ", "okay ", "ok ",
    ]
    for prefix in polite_prefixes:
        if text.startswith(prefix):
            text = text[len(prefix):].strip()
    
    # Remove trailing please
    if text.endswith(" please"):
        text = text[:-7].strip()
    
    # Handle "stop" command
    if text == "stop" or text.startswith("stop"):
        return "stop"
    
    # Handle list places variants
    list_patterns = ["list places", "show places", "what places", 
                     "where can you go", "list locations", "available places"]
    for pattern in list_patterns:
        if pattern in text:
            return "list places"
    
    # Handle door commands: "open (the) X door" / "close (the) X door"
    import re
    door_open_match = re.search(r"open\s+(?:the\s+)?(\w+)\s+door", text)
    if door_open_match:
        door_label = door_open_match.group(1)
        return f"open {door_label} door"
    
    door_close_match = re.search(r"(?:close|shut)\s+(?:the\s+)?(\w+)\s+door", text)
    if door_close_match:
        door_label = door_close_match.group(1)
        return f"close {door_label} door"
    
    # Handle "go to X" variants
    go_patterns = [
        r"go\s+to\s+(?:the\s+)?(.+)",
        r"navigate\s+to\s+(?:the\s+)?(.+)",
        r"take\s+me\s+to\s+(?:the\s+)?(.+)",
        r"walk\s+to\s+(?:the\s+)?(.+)",
        r"go\s+(?:the\s+)?(.+)",  # "go kitchen" without "to"
    ]
    for pattern in go_patterns:
        match = re.match(pattern, text)
        if match:
            target = match.group(1).strip()
            # Remove trailing punctuation
            target = re.sub(r"[.!?,]+$", "", target).strip()
            if target:
                return f"go to {target}"
    
    # Check if text is just a destination name (convert to "go to X")
    for dest in DESTINATIONS:
        if text == dest or text == f"the {dest}":
            return f"go to {dest}"
    
    # Default: return cleaned text as-is (don't discard unknown commands)
    return text


def extract_name(text: str) -> str:
    """
    Extract a name from spoken text like:
      - "My name is Aris"
      - "I'm Aris"
      - "I am Aris"
      - "It's Aris"
      - "Aris" (just the name)
    
    Returns the name or None if not found.
    """
    if not text:
        return None
    
    text_lower = text.lower().strip()
    original_words = text.strip().split()
    
    # Patterns to match (we'll extract what comes after)
    patterns = [
        "my name is ",
        "i'm ",
        "i am ",
        "it's ",
        "this is ",
        "call me ",
        "they call me ",
    ]
    
    for pattern in patterns:
        if pattern in text_lower:
            # Find position and extract the rest
            idx = text_lower.find(pattern)
            name_part = text[idx + len(pattern):].strip()
            # Take first 1-3 words as the name
            name_words = name_part.split()[:3]
            if name_words:
                return " ".join(name_words)
    
    # If no pattern matched but text is short (1-3 words), assume it's just the name
    if len(original_words) <= 3:
        # Filter out common filler words
        fillers = {"um", "uh", "well", "so", "like", "the", "a", "an"}
        name_words = [w for w in original_words if w.lower() not in fillers]
        if name_words:
            return " ".join(name_words)
    
    return None


def main():
    print("=" * 50)
    print("NAO Voice Listener")
    print("=" * 50)
    print(f"Server: {HOST}:{PORT}")
    print(f"Destinations: {', '.join(DESTINATIONS)}")
    print("=" * 50)
    print()
    print("Say things like:")
    print("  'Go to kitchen'")
    print("  'Take me to the bedroom'")
    print("  'My name is Aris'")
    print("  'Stop'")
    print()
    print("Press Ctrl+C to exit")
    print()
    
    # Test microphone
    print("Testing microphone...")
    try:
        devices = sd.query_devices()
        default_input = sd.query_devices(kind='input')
        print(f"Using: {default_input['name']}")
    except Exception as e:
        print(f"[ERROR] No microphone found: {e}")
        return
    
    # Pre-connect to NAO (will retry until successful)
    print(f"\nConnecting to NAO at {HOST}:{PORT}...")
    _ensure_connected()
    
    print("\nReady! Press Enter to start recording (Ctrl+C to quit)\n")
    
    while True:
        try:
            input("Press Enter to record...")
            
            # Record audio
            audio = record_audio()
            
            # Recognize speech
            text = recognize_google(audio)
            
            if text:
                print(f"[HEARD] '{text}'")
                
                # Check for name patterns first (during identity capture)
                name = extract_name(text)
                text_lower = text.lower().strip()
                is_name_phrase = any(p in text_lower for p in [
                    "my name is", "i'm ", "i am ", "call me"
                ])
                
                if is_name_phrase and name:
                    # This is explicitly a name introduction
                    print(f"[NAME DETECTED] {name}")
                    send_name_to_nao(name)
                else:
                    # Normalize and send as command (preserves intent)
                    command = normalize_command(text)
                    if command:
                        send_command(command)
                    else:
                        print(f"[?] Couldn't normalize command from: '{text}'")
            else:
                print("[?] Couldn't understand. Try speaking louder/clearer.")
            
            print()
            
        except KeyboardInterrupt:
            print("\n\nClosing connection...")
            _disconnect()
            print("Goodbye!")
            break


if __name__ == "__main__":
    main()
