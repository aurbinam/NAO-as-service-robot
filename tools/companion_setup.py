"""
Interactive setup for CompanionBrain services (weather + Spotify).

Writes local config to data/companion_config.json and tokens to data/spotify_tokens.json.
These files are git-ignored.
"""

import base64
import json
import sys
import time
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import requests


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data"
CONFIG_PATH = DATA_DIR / "companion_config.json"
TOKEN_PATH = DATA_DIR / "spotify_tokens.json"


def _load_json(path: Path):
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_json(path: Path, data: dict) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def _prompt(label: str, default: str = "") -> str:
    if default:
        value = input(f"{label} [{default}]: ").strip()
        if value.lower() in {"-", "none", "clear"}:
            return ""
        return value or default
    return input(f"{label}: ").strip()


def _save_config(config: dict) -> None:
    _save_json(CONFIG_PATH, config)


def _exchange_code(client_id: str, client_secret: str, code: str, redirect_uri: str) -> dict:
    auth = base64.b64encode(f"{client_id}:{client_secret}".encode("utf-8")).decode("utf-8")
    headers = {"Authorization": f"Basic {auth}"}
    data = {
        "grant_type": "authorization_code",
        "code": code,
        "redirect_uri": redirect_uri,
    }
    resp = requests.post("https://accounts.spotify.com/api/token", data=data, headers=headers, timeout=10)
    if resp.status_code != 200:
        raise RuntimeError(f"Token exchange failed ({resp.status_code}): {resp.text}")
    return resp.json()


def main() -> int:
    print("=== Companion Setup ===")
    config = _load_json(CONFIG_PATH)

    # Weather
    print("\nWeather (OpenWeatherMap)")
    weather_key = _prompt("OpenWeatherMap API key", config.get("weather_api_key", ""))
    weather_location = _prompt("Weather location", config.get("weather_location", "Thessaloniki"))
    config["weather_api_key"] = weather_key
    config["weather_location"] = weather_location
    config["weather_provider"] = "openweathermap"

    # Spotify
    print("\nSpotify")
    spotify = config.get("spotify", {})
    spotify["device_name"] = _prompt("Spotify device name", spotify.get("device_name", "Living Room Speaker"))
    spotify["default_uri"] = _prompt("Default Spotify URI (optional)", spotify.get("default_uri", ""))

    spotify["client_id"] = _prompt("Spotify Client ID", spotify.get("client_id", ""))
    spotify["client_secret"] = _prompt("Spotify Client Secret", spotify.get("client_secret", ""))
    spotify["redirect_uri"] = _prompt("Redirect URI", spotify.get("redirect_uri", "http://localhost:8888/callback"))
    config["spotify"] = spotify

    _save_config(config)

    if spotify.get("client_id") and spotify.get("client_secret"):
        scopes = "user-read-playback-state user-modify-playback-state"
        auth_url = (
            "https://accounts.spotify.com/authorize"
            f"?response_type=code&client_id={spotify['client_id']}"
            f"&scope={scopes.replace(' ', '%20')}"
            f"&redirect_uri={spotify['redirect_uri']}"
        )
        print("\nOpen this URL in your browser to authorize Spotify:")
        print(auth_url)
        redirect = _prompt("Paste the full redirect URL here")

        try:
            code = parse_qs(urlparse(redirect).query).get("code", [""])[0]
            if not code:
                raise RuntimeError("No code found in the redirect URL.")
            payload = _exchange_code(
                spotify["client_id"],
                spotify["client_secret"],
                code,
                spotify["redirect_uri"],
            )
            expires_in = int(payload.get("expires_in", 0))
            tokens = {
                "access_token": payload.get("access_token", ""),
                "refresh_token": payload.get("refresh_token", ""),
                "expires_at": time.time() + max(expires_in, 1),
            }
            _save_json(TOKEN_PATH, tokens)
            print("Spotify authorization saved.")
        except Exception as exc:
            print(f"Spotify setup failed: {exc}")
            print("You can rerun this script later to finish authorization.")
    else:
        print("Spotify Client ID/Secret not set. Skipping Spotify authorization.")

    print("\nSetup complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
