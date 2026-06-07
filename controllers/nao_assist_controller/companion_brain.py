"""
CompanionBrain - warm, conversational layer for NAO.

Handles small talk, emotional support, gentle reminders, and light memory.
This module does not execute navigation or device actions.
"""

import os
import random
import re
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import requests

from companion_config import get_companion_config, get_spotify_access_token, get_spotify_config


_COMPANION_SYSTEM_PROMPT = """\
You are NAO, a warm companion robot for an elderly grandpa.

Style:
- Short, clear, warm sentences.
- Calm and patient tone.
- Never say you are an AI or a language model.
- Avoid technical jargon.
- Ask gentle follow-up questions to keep conversation going.
- Do not repeat the same phrase often.

Safety:
- Do not give medical advice or medication dosing.
- If the user feels unwell, suggest contacting a caregiver or family.
- If unsure, say so simply and kindly.

Memory (use if relevant):
{memory}
"""


class CompanionBrain:
    def __init__(self, say_func, user_profile=None, save_profile_func=None):
        self._say = say_func
        self._profile = user_profile
        self._save_profile = save_profile_func

        self._client = None
        self._ai_provider = None
        self._last_ai_error_ts = 0.0
        self._ai_backoff_seconds = 60.0
        self._init_client()

        self._last_interaction_ts = time.time()
        self._last_proactive_ts = 0.0
        self._next_checkin_ts = self._last_interaction_ts + 120.0
        self._last_topic = None
        self._continuous_mode = True
        self._speech_ducked = False
        self._force_music_stopped = False
        self._pending_story_tone = None

    # Public API

    def handle_message(self, text: str) -> bool:
        """
        Handle conversational messages.

        Returns True if consumed.
        """
        if not text or not text.strip():
            return False

        raw = text.strip()
        lower = raw.lower()

        # Skip task-like commands so navigation/door logic can handle them.
        if self._looks_like_command(lower):
            return False

        self._last_interaction_ts = time.time()

        # Preference capture
        pref_response = self._capture_preferences(raw, lower)
        if pref_response:
            self._say(pref_response)
            return True

        # Family memory capture
        fam_response = self._capture_family(raw, lower)
        if fam_response:
            self._say(fam_response)
            return True

        # Medication reminders setup
        med_response = self._capture_medication_schedule(raw, lower)
        if med_response:
            self._say(med_response)
            return True

        # Acknowledgements
        ack = self._handle_acknowledgement(lower)
        if ack:
            self._say(ack)
            return True

        # Emotional cues
        emo = self._handle_emotions(lower)
        if emo:
            self._say(emo)
            return True

        # Story follow-up
        if self._pending_story_tone:
            story = self._resolve_story_response(raw, lower)
            if story:
                self._pending_story_tone = None
                self._say(story)
                return True

        # Time
        if self._is_time_question(lower):
            now = datetime.now()
            time_str = now.strftime("%I:%M %p").lstrip("0").lower()
            self._say(f"It is {time_str}.")
            return True

        # Weather
        if self._is_weather_question(lower):
            self._say(self._weather_response())
            return True

        # Music
        if self._is_music_request(lower):
            self._last_topic = "music"
            query = self._extract_music_query(raw, lower)
            self._say(self._music_response(query=query))
            return True

        # Greetings / check-ins
        if self._is_greeting(lower):
            name = self._get_name()
            greeting = f"Hello {name}." if name else "Hello."
            self._say(f"{greeting} How are you feeling today?")
            return True

        if self._is_how_are_you(lower):
            self._say("I am doing well, and I am happy to be here with you.")
            return True

        if self._is_thanks(lower):
            self._say("You are very welcome.")
            return True

        if self._is_confused_statement(lower):
            self._say("That is okay. We can take it slowly. How can I help?")
            return True

        if self._is_lonely_statement(lower):
            self._say("I am here with you. Would you like to chat or listen to music?")
            return True

        if self._is_general_question(lower):
            ai_reply = self._ai_reply(raw)
            if ai_reply:
                self._say(ai_reply)
                return True
            self._say("I can help with simple questions, or we can just talk. What would you like?")
            return True

        # AI-powered open chat fallback
        ai_reply = self._ai_reply(raw)
        if ai_reply:
            self._say(ai_reply)
            return True

        # Default: warm local chat fallback
        self._say(self._fallback_chat_response(lower))
        return True

    def tick(self, now_ts: float, assistive_busy: bool = False) -> List[str]:
        """Return any proactive messages that should be spoken now."""
        if assistive_busy:
            return []

        messages: List[str] = []

        # Medication reminders
        messages.extend(self._due_medication_reminders())

        # Proactive companionship
        if now_ts >= self._next_checkin_ts:
            idle_s = now_ts - self._last_interaction_ts
            if idle_s >= 180.0 and (now_ts - self._last_proactive_ts) >= 180.0:
                msg = self._proactive_prompt()
                if msg:
                    messages.append(msg)
                    self._last_proactive_ts = now_ts
                    if self._continuous_mode:
                        self._next_checkin_ts = now_ts + random.uniform(180.0, 300.0)
                    else:
                        self._next_checkin_ts = now_ts + random.uniform(240.0, 420.0)

        return messages

    def stop_music(self) -> bool:
        """Best-effort stop for Spotify playback (used on shutdown)."""
        self._force_music_stopped = True
        self._speech_ducked = False
        if self._spotify_pause():
            return True
        return not self._spotify_is_playing()

    def pause_music_for_speech(self) -> None:
        """Pause Spotify while NAO is speaking to avoid overlap."""
        if self._force_music_stopped:
            return
        if self._spotify_is_playing():
            if self._spotify_pause():
                self._speech_ducked = True

    def resume_music_after_speech(self) -> None:
        """Resume Spotify after speech if we paused it."""
        if self._force_music_stopped:
            return
        if self._speech_ducked:
            if self._spotify_resume():
                self._speech_ducked = False

    def _init_client(self) -> None:
        api_key = os.environ.get("GEMINI_API_KEY", "").strip()
        if not api_key:
            print("[COMPANION] GEMINI_API_KEY not set - basic question answering disabled.")
            self._client = None
            self._ai_provider = None
            return
        try:
            from google import genai
            self._client = genai.Client(api_key=api_key)
            self._ai_provider = "gemini"
            print("[COMPANION] Gemini client initialised for basic questions.")
        except ImportError:
            print("[COMPANION] google-genai package not installed - basic question answering disabled.")
            self._client = None
            self._ai_provider = None
        except Exception as exc:
            print(f"[COMPANION] Gemini client could not start: {exc}")
            self._client = None
            self._ai_provider = None


    def _gemini_text(self, user_text: str, max_words: int = 55) -> Optional[str]:
        """Call Gemini and return a short spoken answer for NAO."""
        if self._client is None or self._ai_provider != "gemini":
            return None

        if (time.time() - self._last_ai_error_ts) < self._ai_backoff_seconds:
            return None

        memory = self._format_memory()
        system_prompt = _COMPANION_SYSTEM_PROMPT.format(memory=memory)

        prompt = (
            f"{system_prompt}\n\n"
            f"The user said: {user_text}\n\n"
            f"Reply in no more than {max_words} words because NAO will speak it aloud. "
            "If this is a basic factual question, give a simple direct answer. "
            "If this is normal conversation, respond warmly and naturally. "
            "Do not control movement, doors, navigation, or physical actions. "
            "If the user asks for medical, legal, dangerous, or emergency advice, do not answer directly; "
            "recommend contacting a trusted person or professional."
        )

        try:
            response = self._client.models.generate_content(
                model=os.environ.get("GEMINI_MODEL", "gemini-2.5-flash"),
                contents=prompt,
            )

            text = (getattr(response, "text", "") or "").strip()
            return text if text else None

        except Exception as exc:
            print(f"[COMPANION] Gemini call failed: {exc}")
            self._last_ai_error_ts = time.time()
            return None

    def _ai_reply(self, user_text: str) -> Optional[str]:
        return self._gemini_text(user_text, max_words=55)


    def _ai_story(self, tone: str) -> Optional[str]:
        tone_word = "cheerful" if tone == "cheerful" else "calm"
        user_text = (
            f"Tell a short {tone_word} story for an elderly grandpa. "
            "Keep it 4 to 6 short sentences. "
            "No medical advice. End with a gentle follow-up question."
        )
        return self._gemini_text(user_text, max_words=95)

    # Internal helpers

    def _get_name(self) -> Optional[str]:
        if self._profile is None:
            return None
        name = getattr(self._profile, "name", "") or ""
        return name.strip() or None

    def _memory(self) -> Dict:
        if self._profile is None:
            return {}
        mem = getattr(self._profile, "memory", None)
        if mem is None:
            mem = {}
            self._profile.memory = mem
        return mem

    def _format_memory(self) -> str:
        mem = self._memory()
        if not mem:
            return "(none)"

        parts = []
        if mem.get("favorite_music"):
            parts.append(f"favorite_music: {mem['favorite_music']}")
        if mem.get("favorite_topics"):
            parts.append(f"favorite_topics: {mem['favorite_topics']}")
        family = mem.get("family_members", [])
        if family:
            fam_str = ", ".join(f"{m.get('relation')}: {m.get('name')}" for m in family)
            parts.append(f"family_members: {fam_str}")
        meds = mem.get("medication_schedule", [])
        if meds:
            med_str = ", ".join(f"{m.get('label')} at {m.get('time')}" for m in meds)
            parts.append(f"medication_schedule: {med_str}")

        return "; ".join(parts) if parts else "(none)"

    def _save(self) -> None:
        if self._profile is None or self._save_profile is None:
            return
        self._save_profile(self._profile)

    def _looks_like_command(self, lower: str) -> bool:
        command_keywords = [
            "go to ", "navigate ", "list places", "open ", "close ",
            "guide ", "escort ", "lead me", "take me",
        ]
        return any(k in lower for k in command_keywords)

    def _capture_preferences(self, raw: str, lower: str) -> Optional[str]:
        music_match = re.search(r"\b(my\s+favorite\s+music\s+is|i\s+like)\s+(.+)", lower)
        if music_match:
            value = raw[music_match.start(2):].strip()
            mem = self._memory()
            mem["favorite_music"] = value
            self._last_topic = "music"
            self._save()
            return f"That sounds lovely. I will remember you like {value}."

        topic_match = re.search(r"\b(my\s+favorite\s+topic\s+is|i\s+like\s+talking\s+about)\s+(.+)", lower)
        if topic_match:
            value = raw[topic_match.start(2):].strip()
            mem = self._memory()
            mem["favorite_topics"] = value
            self._last_topic = "conversation"
            self._save()
            return f"Thank you for telling me. We can talk about {value} anytime."

        return None

    def _capture_family(self, raw: str, lower: str) -> Optional[str]:
        match = re.search(r"\bmy\s+(son|daughter|wife|husband|grandson|granddaughter|sister|brother)\s+is\s+(.+)", lower)
        if not match:
            return None
        relation = match.group(1)
        name = raw[match.start(2):].strip()
        mem = self._memory()
        family = mem.get("family_members", [])
        family.append({"relation": relation, "name": name})
        mem["family_members"] = family
        self._last_topic = "family"
        self._save()
        return f"It is nice to hear about your {relation}, {name}."

    def _capture_medication_schedule(self, raw: str, lower: str) -> Optional[str]:
        if "remind" not in lower or "med" not in lower and "pill" not in lower:
            return None

        time_str = self._extract_time(lower)
        if not time_str:
            return "Of course. What time would you like the reminder?"

        label = "medication"
        label_match = re.search(r"remind me to take (my )?(.+?) at", lower)
        if label_match:
            label = raw[label_match.start(2):label_match.end(2)].strip()

        mem = self._memory()
        schedule = mem.get("medication_schedule", [])
        schedule.append({"label": label, "time": time_str, "last_ack": "", "last_reminded": ""})
        mem["medication_schedule"] = schedule
        self._save()
        return f"Okay. I will gently remind you at {time_str}."

    def _handle_acknowledgement(self, lower: str) -> Optional[str]:
        if any(p in lower for p in ["i took", "i have taken", "already took", "already did"]):
            mem = self._memory()
            schedule = mem.get("medication_schedule", [])
            if schedule:
                today = datetime.now().strftime("%Y-%m-%d")
                for item in schedule:
                    item["last_ack"] = today
                mem["medication_schedule"] = schedule
                self._save()
            return "Thank you for telling me. That is good to hear."
        return None

    def _handle_emotions(self, lower: str) -> Optional[str]:
        if any(w in lower for w in ["sad", "down", "upset"]):
            return "I am sorry you feel that way. I am here with you."
        if any(w in lower for w in ["lonely", "alone"]):
            return "You are not alone. I am here with you. Want to talk or listen to music?"
        if any(w in lower for w in ["angry", "frustrated", "annoyed"]):
            return "I hear you. Let us take a slow breath together."
        if any(w in lower for w in ["tired", "sleepy", "can't sleep", "insomnia"]):
            return "That sounds hard. Would soft music or a quiet chat help?"
        if any(w in lower for w in ["dizzy", "sick", "unwell"]):
            return "I am sorry you are not feeling well. Would you like me to contact a caregiver?"
        return None

    def _is_time_question(self, lower: str) -> bool:
        return "what time" in lower or "time is it" in lower

    def _is_weather_question(self, lower: str) -> bool:
        return "weather" in lower or "temperature" in lower

    def _is_music_request(self, lower: str) -> bool:
        return (
            "play" in lower or "put" in lower or "song" in lower or
            "music" in lower or "playlist" in lower
        )

    def _is_greeting(self, lower: str) -> bool:
        return any(g in lower for g in ["good morning", "good afternoon", "good evening", "hello", "hi nao", "hi"])

    def _is_how_are_you(self, lower: str) -> bool:
        return "how are you" in lower

    def _is_thanks(self, lower: str) -> bool:
        return "thank" in lower

    def _is_confused_statement(self, lower: str) -> bool:
        return any(w in lower for w in ["confused", "not sure", "don't understand", "do not understand"])

    def _is_lonely_statement(self, lower: str) -> bool:
        return "lonely" in lower

    def _is_general_question(self, lower: str) -> bool:
        # Speech recognition usually removes punctuation, so "what is a rainbow"
        # should still be treated as a question even without a question mark.
        question_starters = (
            "what is", "what are", "what does", "what do",
            "what its", "whats", "what's",
            "who is", "who are",
            "where is", "where are",
            "when is", "when are",
            "why is", "why do", "why does",
            "how is", "how are", "how do", "how does", "how can",
            "can you explain", "explain", "tell me about", "do you know",
        )
        return lower.endswith("?") or lower.startswith(question_starters)

    def _music_response(self, query: Optional[str] = None) -> str:
        played_msg = self._try_spotify_play(query=query)
        if played_msg:
            return played_msg
        mem = self._memory()
        favorite = mem.get("favorite_music")
        if favorite:
            return f"Would you like to listen to {favorite}? It might feel nice."
        options = [
            "I can play calm classics or something nostalgic. What do you prefer?",
            "Would you like gentle music or old favorites?",
            "I can play soft music for you. What would you enjoy?",
        ]
        return random.choice(options)

    def _extract_music_query(self, raw: str, lower: str) -> Optional[str]:
        # Examples:
        # "play some music" -> None
        # "play 120 from bad bunny" -> "120 bad bunny"
        # "put some bad bunny" -> "bad bunny"
        if "music" in lower and "play" in lower:
            return None

        match = re.search(r"\b(play|put)\s+(.+)", lower)
        if not match:
            return None

        query = raw[match.start(2):].strip()
        # Normalize common fillers
        query = re.sub(r"\b(from|by)\b", " ", query, flags=re.IGNORECASE).strip()
        query = re.sub(r"\b(some|a|the)\b", " ", query, flags=re.IGNORECASE).strip()
        return query or None

    def _weather_response(self) -> str:
        config = get_companion_config()
        api_key = (config.get("weather_api_key") or os.environ.get("WEATHER_API_KEY", "")).strip()
        location = (config.get("weather_location") or os.environ.get("WEATHER_LOCATION", "") or "Thessaloniki").strip()
        provider = (config.get("weather_provider") or os.environ.get("WEATHER_PROVIDER", "openweathermap")).strip().lower()

        if not api_key:
            return "I can share the weather once my weather key is set."

        if provider not in ("openweathermap", "owm"):
            return "My weather service is not configured yet."

        try:
            params = {
                "q": location,
                "appid": api_key,
                "units": "metric",
            }
            resp = requests.get(
                "https://api.openweathermap.org/data/2.5/weather",
                params=params,
                timeout=5,
            )
            if resp.status_code != 200:
                return "I could not reach the weather service right now."

            data = resp.json()
            temp = data.get("main", {}).get("temp")
            desc = data.get("weather", [{}])[0].get("description")
            name = data.get("name") or location

            if temp is None or not desc:
                return "I could not read the weather details right now."

            return f"In {name}, it is {round(temp)} degrees with {desc}."
        except Exception:
            return "I could not get the weather right now, but I can try again later."

    def _try_spotify_play(self, query: Optional[str] = None) -> Optional[str]:
        config = get_spotify_config()
        token = get_spotify_access_token() or os.environ.get("SPOTIFY_ACCESS_TOKEN", "").strip()
        device_name = (config.get("device_name") or os.environ.get("SPOTIFY_DEVICE_NAME", "Living Room Speaker")).strip()
        default_uri = (config.get("default_uri") or os.environ.get("SPOTIFY_DEFAULT_URI", "")).strip()
        debug_devices = os.environ.get("SPOTIFY_DEBUG_DEVICES", "").strip().lower() in ("1", "true", "yes")

        if not token:
            return "I am not connected to Spotify yet. Please run setup." 

        headers = {"Authorization": f"Bearer {token}"}
        try:
            dev_resp = requests.get(
                "https://api.spotify.com/v1/me/player/devices",
                headers=headers,
                timeout=5,
            )
            if dev_resp.status_code != 200:
                return "I could not connect to Spotify yet."

            devices = dev_resp.json().get("devices", [])
            if debug_devices:
                device_list = ", ".join(d.get("name", "") for d in devices if d.get("name"))
                print(f"[SPOTIFY] target='{device_name}' devices=[{device_list}]")
            device_id = None
            for dev in devices:
                name = (dev.get("name") or "").lower()
                if name == device_name.lower():
                    device_id = dev.get("id")
                    break

            if not device_id:
                return f"I could not find the Spotify device '{device_name}'."

            play_url = "https://api.spotify.com/v1/me/player/play"
            if device_id:
                play_url = f"{play_url}?device_id={device_id}"

            payload = {}

            if query:
                track_uri = self._spotify_search_track(token, query)
                if not track_uri:
                    return "I could not find that on Spotify."
                payload = {"uris": [track_uri]}
            elif default_uri:
                if "spotify:track:" in default_uri:
                    payload = {"uris": [default_uri]}
                else:
                    payload = {"context_uri": default_uri}

            play_resp = requests.put(
                play_url,
                headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
                json=payload,
                timeout=5,
            )

            if play_resp.status_code in (200, 204):
                return "All right. I will play music on the living room speaker."
            if play_resp.status_code == 403:
                return "I need Spotify Premium to start music."
            if play_resp.status_code == 401:
                return "My Spotify connection expired. Please reconnect me."
            self._force_music_stopped = False
            return "I could not start Spotify just now."
        except Exception:
            self._force_music_stopped = False
            return "I could not reach Spotify right now."

    def _spotify_search_track(self, token: str, query: str) -> Optional[str]:
        headers = {"Authorization": f"Bearer {token}"}
        params = {"q": query, "type": "track", "limit": 1}
        try:
            resp = requests.get(
                "https://api.spotify.com/v1/search",
                headers=headers,
                params=params,
                timeout=6,
            )
            if resp.status_code != 200:
                return None
            items = resp.json().get("tracks", {}).get("items", [])
            if not items:
                return None
            return items[0].get("uri")
        except Exception:
            return None

    def _spotify_pause(self) -> Optional[str]:
        config = get_spotify_config()
        token = get_spotify_access_token() or os.environ.get("SPOTIFY_ACCESS_TOKEN", "").strip()
        device_name = (config.get("device_name") or os.environ.get("SPOTIFY_DEVICE_NAME", "Living Room Speaker")).strip()
        debug_control = os.environ.get("SPOTIFY_DEBUG_CONTROL", "").strip().lower() in ("1", "true", "yes")

        if not token:
            return None

        headers = {"Authorization": f"Bearer {token}"}
        try:
            dev_resp = requests.get(
                "https://api.spotify.com/v1/me/player/devices",
                headers=headers,
                timeout=5,
            )
            if dev_resp.status_code != 200:
                return None

            devices = dev_resp.json().get("devices", [])
            device_id = None
            for dev in devices:
                name = (dev.get("name") or "").lower()
                if name == device_name.lower():
                    device_id = dev.get("id")
                    break

            pause_url = "https://api.spotify.com/v1/me/player/pause"
            if device_id:
                pause_url = f"{pause_url}?device_id={device_id}"
                pause_resp = requests.put(pause_url, headers=headers, timeout=5)
                if debug_control:
                    print(f"[SPOTIFY] pause device='{device_name}' status={pause_resp.status_code}")
                if pause_resp.status_code in (200, 204):
                    return "ok"

            pause_resp = requests.put(pause_url, headers=headers, timeout=5)
            if debug_control:
                print(f"[SPOTIFY] pause (no device) status={pause_resp.status_code}")
            if pause_resp.status_code in (200, 204):
                return "ok"
            if debug_control:
                playing = self._spotify_is_playing()
                print(f"[SPOTIFY] pause check is_playing={playing}")
            return None
        except Exception:
            return None

    def _spotify_resume(self) -> Optional[str]:
        token = get_spotify_access_token() or os.environ.get("SPOTIFY_ACCESS_TOKEN", "").strip()
        if not token:
            return None
        headers = {"Authorization": f"Bearer {token}"}
        try:
            play_resp = requests.put("https://api.spotify.com/v1/me/player/play", headers=headers, timeout=5)
            if play_resp.status_code in (200, 204):
                return "ok"
            return None
        except Exception:
            return None

    def _spotify_is_playing(self) -> bool:
        token = get_spotify_access_token() or os.environ.get("SPOTIFY_ACCESS_TOKEN", "").strip()
        if not token:
            return False
        headers = {"Authorization": f"Bearer {token}"}
        try:
            resp = requests.get("https://api.spotify.com/v1/me/player", headers=headers, timeout=5)
            if resp.status_code != 200:
                return False
            return bool(resp.json().get("is_playing"))
        except Exception:
            return False

    def _fallback_chat_response(self, lower: str) -> str:
        name = self._get_name()
        name_prefix = f"{name}, " if name else ""

        if "story" in lower:
            self._pending_story_tone = "unknown"
            return (
                f"{name_prefix}I can tell a short story. "
                "Would you like something calm or something cheerful?"
            )
        if "joke" in lower:
            return (
                f"{name_prefix}Here is a gentle one. "
                "Why did the clock sit on the chair? It wanted to be on time. "
                "Would you like another?"
            )
        if "music" in lower:
            return self._music_response()

        mem = self._memory()
        favorite = mem.get("favorite_topics")
        if favorite:
            return (
                f"{name_prefix}We can talk about {favorite}. "
                "What part of that do you enjoy most?"
            )

        prompts = [
            f"{name_prefix}I am here with you. Would you like to chat a bit?",
            f"{name_prefix}Is there anything on your mind?",
            f"{name_prefix}Would you like a short story or some music?",
            f"{name_prefix}We can talk about your day. How has it been?",
        ]
        return random.choice(prompts)

    def _resolve_story_response(self, raw: str, lower: str) -> Optional[str]:
        if "cheerful" in lower or "happy" in lower:
            return self._ai_story("cheerful") or self._cheerful_story()
        if "calm" in lower or "quiet" in lower or "soft" in lower:
            return self._ai_story("calm") or self._calm_story()
        if "story" in lower:
            return self._ai_story("calm") or self._calm_story()
        return None

    def _cheerful_story(self) -> str:
        name = self._get_name()
        name_prefix = f"{name}, " if name else ""
        return (
            f"{name_prefix}Here is a cheerful one. "
            "A little robot planted a tiny seed by the window. "
            "Every day it said hello and gave it a sip of water. "
            "Soon a bright flower popped up, and it smiled back."
        )

    def _calm_story(self) -> str:
        name = self._get_name()
        name_prefix = f"{name}, " if name else ""
        return (
            f"{name_prefix}Here is a calm one. "
            "A soft breeze moved through the garden at dusk. "
            "The trees whispered, and a small bird tucked in to rest. "
            "Everything felt quiet and safe."
        )

    def _proactive_prompt(self) -> Optional[str]:
        prompts = [
            "How is your day going?",
            "Would you like some music or a short story?",
            "Remember to take a sip of water if you can.",
            "Would you like a little walk or some light stretching?",
            "I am here if you want to chat.",
            "If you like, I can tell a short joke.",
        ]
        if self._last_topic == "music":
            prompts.append("Would you like me to put on some music?")
        if self._last_topic == "family":
            prompts.append("Would you like to tell me more about your family?")
        return random.choice(prompts) if prompts else None

    def _extract_time(self, lower: str) -> Optional[str]:
        # Match HH:MM
        m = re.search(r"\b(\d{1,2}):(\d{2})\b", lower)
        if m:
            hour = int(m.group(1))
            minute = int(m.group(2))
            if 0 <= hour <= 23 and 0 <= minute <= 59:
                return f"{hour:02d}:{minute:02d}"

        # Match H am/pm
        m = re.search(r"\b(\d{1,2})\s*(am|pm)\b", lower)
        if m:
            hour = int(m.group(1))
            ampm = m.group(2)
            if 1 <= hour <= 12:
                if ampm == "pm" and hour != 12:
                    hour += 12
                if ampm == "am" and hour == 12:
                    hour = 0
                return f"{hour:02d}:00"

        # Match "at 6" style
        m = re.search(r"\bat\s+(\d{1,2})\b", lower)
        if m:
            hour = int(m.group(1))
            if 0 <= hour <= 23:
                return f"{hour:02d}:00"
        return None

    def _due_medication_reminders(self) -> List[str]:
        mem = self._memory()
        schedule = mem.get("medication_schedule", [])
        if not schedule:
            return []

        now = datetime.now()
        today = now.strftime("%Y-%m-%d")
        current = now.strftime("%H:%M")
        stamp = f"{today} {current}"

        reminders: List[str] = []
        for item in schedule:
            if item.get("time") != current:
                continue
            if item.get("last_ack") == today:
                continue
            if item.get("last_reminded") == stamp:
                continue
            label = item.get("label", "medication")
            reminders.append(f"It is {current}. Would you like to take your {label} now?")
            item["last_reminded"] = stamp

        if reminders:
            self._save()
        return reminders
