"""
Companion configuration and credential helpers.

Stores secrets in local JSON files under data/ (git-ignored).
"""

import base64
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional

import requests


_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_DATA_DIR = _PROJECT_ROOT / "data"
_CONFIG_PATH = _DATA_DIR / "companion_config.json"
_TOKEN_PATH = _DATA_DIR / "spotify_tokens.json"


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _save_json(path: Path, data: Dict[str, Any]) -> None:
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def get_companion_config() -> Dict[str, Any]:
    return _load_json(_CONFIG_PATH)


def save_companion_config(config: Dict[str, Any]) -> None:
    _save_json(_CONFIG_PATH, config)


def save_spotify_tokens(tokens: Dict[str, Any]) -> None:
    _save_json(_TOKEN_PATH, tokens)


def _load_spotify_tokens() -> Dict[str, Any]:
    return _load_json(_TOKEN_PATH)


def get_spotify_config(config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    config = config or get_companion_config()
    return config.get("spotify", {})


def get_spotify_access_token() -> Optional[str]:
    config = get_spotify_config()
    client_id = config.get("client_id", "").strip()
    client_secret = config.get("client_secret", "").strip()
    if not client_id or not client_secret:
        return None

    tokens = _load_spotify_tokens()
    access_token = tokens.get("access_token", "").strip()
    refresh_token = tokens.get("refresh_token", "").strip()
    expires_at = float(tokens.get("expires_at", 0))

    if access_token and time.time() < (expires_at - 30):
        return access_token

    if not refresh_token:
        return None

    try:
        auth = base64.b64encode(f"{client_id}:{client_secret}".encode("utf-8")).decode("utf-8")
        headers = {"Authorization": f"Basic {auth}"}
        data = {"grant_type": "refresh_token", "refresh_token": refresh_token}
        resp = requests.post("https://accounts.spotify.com/api/token", data=data, headers=headers, timeout=8)
        if resp.status_code != 200:
            return None
        payload = resp.json()
        access_token = payload.get("access_token", "").strip()
        expires_in = int(payload.get("expires_in", 0))
        new_refresh = payload.get("refresh_token", "").strip() or refresh_token
        if access_token:
            save_spotify_tokens({
                "access_token": access_token,
                "refresh_token": new_refresh,
                "expires_at": time.time() + max(expires_in, 1),
            })
            return access_token
    except Exception:
        return None

    return None
