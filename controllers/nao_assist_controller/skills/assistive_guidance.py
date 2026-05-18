"""
Assistive Guidance Skill - Escorts user to a destination.

Sequence (ASSISTIVE GUIDANCE MODE):
1. Locate user (DEF GRANDPA node) and navigate to them (~1m distance)
2. Say: "I am here to help you. We will go to <dest>. Please follow me."
3. Signal Grandpa controller: "start_walk"
4. Navigate to destination via RobotBrain (handles doors, replanning)
5. Signal Grandpa: "arrived" on success, "stop" on failure
6. Say arrival/failure message
"""

from typing import Tuple

LOG_PREFIX = "[ASSIST_GUIDE]"

ARRIVE_AT_USER_M = 1.0


def _send_signal(emitter, msg: str) -> None:
    if emitter is None:
        return
    try:
        emitter.send(msg.encode("utf-8"))
        print(f"{LOG_PREFIX} Signal sent to Grandpa: '{msg}'")
    except Exception as exc:
        print(f"{LOG_PREFIX} Signal send failed: {exc}")


def _get_user_position(robot) -> Tuple[float, float, float]:
    try:
        node = robot.getFromDef("GRANDPA")
        if node is not None:
            pos = node.getPosition()
            return (pos[0], pos[1], pos[2])
    except Exception as exc:
        print(f"{LOG_PREFIX} Could not read GRANDPA position: {exc}")
    return (0.0, 0.6, 0.0)


def _get_route_doors(brain, destination: str) -> list:
    try:
        return brain._executor._house.get_route_doors_for_target(destination) or []
    except Exception:
        return []


def execute_assistive_guidance(
    destination: str,
    robot,
    brain,
    say_func,
    emitter=None,
) -> bool:
    """
    Full ASSISTIVE GUIDANCE MODE sequence.

    Args:
        destination: Resolved house target ID (e.g. "kitchen", "bedroom")
        robot:       Webots Supervisor instance
        brain:       RobotBrain — owns NavigationController + door planner
        say_func:    Controller say() callable
        emitter:     super_emitter device (channel 2) for Grandpa signals

    Returns:
        True on successful arrival.
    """
    dest_readable = destination.replace("_", " ")

    # 1. Navigate to user
    user_pos = _get_user_position(robot)
    ux, uy = user_pos[0], user_pos[1]
    print(f"{LOG_PREFIX} User at ({ux:.2f}, {uy:.2f}) — approaching")

    reached_user = brain._executor._navigate_to_point(ux, uy, arrive_dist=ARRIVE_AT_USER_M)
    if not reached_user:
        say_func("I could not reach you. Please come a little closer.")
        return False

    # 2. Greet and announce
    say_func(
        f"I am here to help you. "
        f"We will go to the {dest_readable}. "
        f"Please follow me."
    )

    # 3. Signal Grandpa: start walking
    _send_signal(emitter, "start_walk")

    # 4. Navigate to destination (brain handles doors + replanning)
    route_doors = _get_route_doors(brain, destination)
    if route_doors:
        say_func("Please wait while I check a safe path.")

    print(f"{LOG_PREFIX} Leading to '{destination}' (doors on route: {route_doors})")
    success = brain.handle_command(f"go to {destination}")

    # 5. Signal Grandpa: done
    _send_signal(emitter, "arrived" if success else "stop")

    # 6. Arrival or failure message
    if success:
        say_func(
            f"We have arrived at the {dest_readable}. "
            f"Is there anything else I can help you with?"
        )
    else:
        say_func(f"I am sorry, I had trouble reaching the {dest_readable}.")

    return success
