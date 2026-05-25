"""
RobotBrain - top-level orchestrator with explicit state machine.

Full 4-layer intelligent architecture:

  User text
      |
      v
  [Layer 1: GenAIInterpreter]  NL -> structured intent
      |
      v
  [Layer 2: TaskPlanner]       intent -> action plan
      |
      v
  [Layer 3: Executor]          one action at a time -> ExecutionResult
      |
      v
  [Layer 4: ExecutionMonitor]  detects stuck/diverging -> replan signal

State machine:
  IDLE -> UNDERSTANDING -> PLANNING -> EXECUTING -> MONITORING
       -> REPLANNING -> COMPLETED | FAILED
"""

from enum import Enum, auto
from typing import List, Tuple

from intelligence.genai_interpreter import GenAIInterpreter
from intelligence.task_planner import TaskPlanner
from intelligence.executor import Executor, ExecutionResult
from intelligence.monitor import ExecutionMonitor, MonitorSignal

LOG_PREFIX = "[BRAIN]"


class RobotState(Enum):
    IDLE = auto()
    UNDERSTANDING = auto()
    PLANNING = auto()
    EXECUTING = auto()
    MONITORING = auto()
    REPLANNING = auto()
    COMPLETED = auto()
    FAILED = auto()


class RobotBrain:
    """
    All user commands enter through handle_command().
    No navigation is ever triggered directly from raw text.
    """

    MAX_STEP_RETRIES = 3

    def __init__(self, robot, house_config, say_func):
        rooms = house_config.list_targets()
        doors = house_config.list_doors()

        self._interpreter = GenAIInterpreter(rooms, doors)
        self._planner = TaskPlanner(house_config)
        self._executor = Executor(robot, house_config, say_func)
        self._monitor = ExecutionMonitor(max_retries=self.MAX_STEP_RETRIES)
        self._say = say_func

        self._state: RobotState = RobotState.IDLE
        self._plan: List[Tuple] = []
        self._step: int = 0
        self._last_failure_cause = None
        self._collision_count = 0
        self._safety_margin = 0.70

        print(f"{LOG_PREFIX} Initialised. Rooms: {rooms}. Doors: {doors}.")

    # ------------------------------------------------------------------ #
    #  Public entry point                                                  #
    # ------------------------------------------------------------------ #

    def handle_command(self, user_input: str) -> bool:
        """
        Main entry point. Returns True on success, False on failure.

        Flow:
          IDLE -> UNDERSTANDING -> PLANNING
               -> EXECUTING/MONITORING loop
               -> COMPLETED or FAILED
        """
        # -- Layer 1: interpret --
        self._transition(RobotState.UNDERSTANDING)
        print(f"{LOG_PREFIX} Command: '{user_input}'")

        intent = self._interpreter.interpret(user_input)

        if intent.get("intent") == "unknown":
            self._say("I am not sure I understood. Could you say it again, please?")
            return self._fail("interpretation_failed")

        # -- Layer 2: plan --
        self._transition(RobotState.PLANNING)

        try:
            # Pass NAO's live position so the planner can prepend a controlled
            # exit when NAO starts inside a room (exit = inverse of entry).
            current_xy = self._executor._robot_pos_2d()
            plan = self._planner.plan(intent, current_xy=current_xy)
        except ValueError as exc:
            self._say("I am sorry, I cannot do that right now. Let's try something simpler.")
            return self._fail(f"planning_error:{exc}")

        errors = self._planner.validate(plan)
        if errors:
            self._say("I found a problem with that plan. Let us try another way.")
            return self._fail(f"invalid_plan:{errors}")

        self._plan = plan
        self._step = 0
        self._log_plan()

        # -- Layers 3 + 4: execute with monitoring --
        return self._run_plan()

    @property
    def state(self) -> RobotState:
        return self._state

    # ------------------------------------------------------------------ #
    #  Execution loop                                                      #
    # ------------------------------------------------------------------ #

    def _run_plan(self) -> bool:
        while self._step < len(self._plan):
            action = self._plan[self._step]

            # -- Layer 3: execute --
            self._transition(RobotState.EXECUTING)
            self._say(self._describe(action))

            self._monitor.begin_attempt()
            result = self._executor.execute(action)

            # -- Layer 4: monitor --
            self._transition(RobotState.MONITORING)

            if result.is_success():
                print(f"{LOG_PREFIX} Step {self._step + 1} succeeded.")
                self._step += 1
                if self._step == len(self._plan):
                    self._planner.reset_safety_margin()
                    self._safety_margin = 0.70
                    self._collision_count = 0
                continue

            # Step failed - attempt replanning
            recovered = self._handle_failure(action, result)
            if not recovered:
                self._say("I could not complete that task. I am sorry.")
                return self._fail(f"step_failed:{result.reason}")

        self._transition(RobotState.COMPLETED)
        self._say("All done. Would you like anything else?")
        return True

    # ------------------------------------------------------------------ #
    #  Replanning                                                          #
    # ------------------------------------------------------------------ #

    def _handle_failure(self, action: Tuple, result: ExecutionResult) -> bool:
        """
        Handle a failed step via replanning.
        Returns True if retry is allowed, False if budget exhausted.
        """
        self._transition(RobotState.REPLANNING)

        allowed = self._monitor.record_retry()
        if not allowed:
            self._say(
                f"I have retried {self._monitor.max_retries} times "
                "without success. Stopping."
            )
            return False

        attempt = self._monitor.retry_count
        reason = result.reason or "unknown"

        if "fall" in reason:
            msg = "I fell while moving. I will recover and try again."
        elif "obstacle" in reason or reason == "target_not_reached":
            # Could be wall proximity detected by sonar
            self._collision_count += 1
            new_margin = self._safety_margin + 0.20 * self._collision_count
            self._safety_margin = min(new_margin, 1.50)
            self._planner.set_safety_margin(self._safety_margin)
            msg = "I see something in the way. I will take a safer path."
        elif "not_reached" in reason:
            msg = "The direct path seems blocked. I will adjust my approach."
        elif "progress" in reason or "stuck" in reason:
            msg = "I am not making progress. I will try a different way."
        else:
            msg = "I ran into a small issue. I will try again."

        self._say(msg)
        print(f"{LOG_PREFIX} Replanning step {self._step + 1}: {reason}, "
              f"attempt {attempt}/{self._monitor.max_retries}")
        return True   # retry same step in _run_plan loop

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _transition(self, new_state: RobotState):
        print(f"{LOG_PREFIX} {self._state.name} -> {new_state.name}")
        self._state = new_state

    def _fail(self, reason: str) -> bool:
        self._transition(RobotState.FAILED)
        print(f"{LOG_PREFIX} Task failed: {reason}")
        return False

    def _log_plan(self):
        print(f"{LOG_PREFIX} Plan ({len(self._plan)} step(s)):")
        for i, action in enumerate(self._plan):
            print(f"{LOG_PREFIX}   {i+1:2d}.  {' '.join(str(a) for a in action)}")

    def _describe(self, action: Tuple) -> str:
        def _pretty(name: str) -> str:
            return str(name).replace("_", " ")

        atype = action[0]

        if atype == "navigate":
            return f"All right. I will go to the {_pretty(action[1])}."
        if atype == "navigate_door_approach":
            return f"I am approaching the {_pretty(action[1])} door."
        if atype == "verify_safe_approach":
            return "I am lining up to use the door safely."
        if atype == "open_door":
            return f"I am opening the {_pretty(action[1])} door."
        if atype == "cross_doorway":
            return f"I am stepping through the {_pretty(action[1])} doorway."
        if atype == "close_door":
            return f"I am closing the {_pretty(action[1])} door."
        if atype == "door_operation":
            op = "opening" if str(action[1]).lower() == "open" else "closing"
            return f"I am {op} the {_pretty(action[2])} door."

        return f"I am executing action: {atype}."
