# NAO as a Service Robot

An assistive humanoid for elderly home care, built on the NAO platform in the
Webots simulator. The robot recognises and remembers its user, holds spoken
conversation, gives medication reminders, offers companionship, and escorts the
user between rooms, opening doors along the route.

The design keeps a strict separation of concerns: a language model only
interprets speech and converses, while deterministic logic handles all movement
and safety-critical action.

## Repository layout

```
worlds/                              Webots world (house, NAO, doors)
controllers/
  nao_assist_controller/             main robot controller
    nao_assist_controller.py         entry point, control loop, TCP server
    intelligence/                    interpreter, task planner, executor, monitor
    skills/                          navigation, doorway crossing, door control
    house/                           house configuration loader
    companion_brain.py               conversational companion
    ai_command_parser.py             deterministic command parser
  door_manager/                      separate door controller (alternative path)
voice_listener.py                    external speech-to-text process
requirements.txt
```

## Requirements

- [Webots](https://cyberbotics.com/) (provides the `controller` module)
- Python 3.10+ and the packages in `requirements.txt`:

```
pip install -r requirements.txt
```

## API keys (optional)

The system runs without any keys, with reduced features. Set these in the
environment to enable the language features:

| Variable | Purpose | Without it |
|----------|---------|------------|
| `ANTHROPIC_API_KEY` | Claude command interpretation | falls back to keyword parsing |
| `GEMINI_API_KEY`    | Gemini conversational answers | companion answers disabled |

Optional extras: `WEATHER_API_KEY` / `WEATHER_LOCATION` (weather replies) and
`SPOTIFY_*` (music playback). No keys are stored in the repository.

## Running

1. Open `worlds/My project.wbt` in Webots and start the simulation.
2. (Optional) run the voice listener in a separate terminal for speech input:
   ```
   python voice_listener.py
   ```
   Without it, commands can be typed into `controllers/nao_assist_controller/cmd.txt`.
3. Example commands:
   - `list places`
   - `go to <room>`
   - `guide me to <room>`
   - `open the <label> door` / `close the <label> door`
   - `reset`

Commands reach the controller over a TCP socket on port `5005`.
