"""
Behavior Tree primitives for NAO navigation.

Implements the standard BT node types:
  Sequence  - AND: run children left-to-right; fail on first FAILURE
  Selector  - OR:  run children left-to-right; succeed on first SUCCESS
  Condition - leaf that evaluates a bool predicate (no side effects)
  Action    - leaf that performs work and returns a BTStatus

All nodes are stateless between ticks EXCEPT Sequence, which remembers
which child it was RUNNING to support resumable multi-step sequences.
Reset a Sequence by calling reset() or receiving SUCCESS/FAILURE.

Status contract:
  SUCCESS  - node completed its goal
  FAILURE  - node could not complete its goal
  RUNNING  - node is mid-execution; tick again next cycle
"""

from enum import Enum, auto
from typing import Callable, List


class BTStatus(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()


class BTNode:
    """Abstract base for all BT nodes."""
    def tick(self) -> BTStatus:
        raise NotImplementedError

    def reset(self):
        pass


# ---------------------------------------------------------------------------
# Composite nodes
# ---------------------------------------------------------------------------

class Sequence(BTNode):
    """
    AND node — succeeds only when ALL children succeed in order.

    Memory: remembers the running child index so execution resumes
    at the correct child on the next tick (not from scratch).
    Resets index on SUCCESS or FAILURE.
    """

    def __init__(self, name: str, children: List[BTNode]):
        self.name     = name
        self.children = children
        self._idx     = 0

    def tick(self) -> BTStatus:
        while self._idx < len(self.children):
            status = self.children[self._idx].tick()
            if status == BTStatus.RUNNING:
                return BTStatus.RUNNING
            if status == BTStatus.FAILURE:
                self._idx = 0
                return BTStatus.FAILURE
            self._idx += 1
        self._idx = 0
        return BTStatus.SUCCESS

    def reset(self):
        self._idx = 0
        for c in self.children:
            c.reset()


class Selector(BTNode):
    """
    OR node — succeeds on first child that succeeds.

    Stateless: always starts from child 0 each tick.
    """

    def __init__(self, name: str, children: List[BTNode]):
        self.name     = name
        self.children = children

    def tick(self) -> BTStatus:
        for child in self.children:
            status = child.tick()
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE

    def reset(self):
        for c in self.children:
            c.reset()


# ---------------------------------------------------------------------------
# Leaf nodes
# ---------------------------------------------------------------------------

class Condition(BTNode):
    """Stateless predicate. SUCCESS if fn() is True, FAILURE otherwise."""

    def __init__(self, name: str, fn: Callable[[], bool]):
        self.name = name
        self._fn  = fn

    def tick(self) -> BTStatus:
        return BTStatus.SUCCESS if self._fn() else BTStatus.FAILURE


class Action(BTNode):
    """
    Action leaf. fn() must return a BTStatus.

    fn() may be blocking (e.g. walk_to_target loops internally) or
    non-blocking (returns RUNNING and advances shared state next tick).
    """

    def __init__(self, name: str, fn: Callable[[], BTStatus]):
        self.name = name
        self._fn  = fn

    def tick(self) -> BTStatus:
        return self._fn()


# ---------------------------------------------------------------------------
# Decorator nodes
# ---------------------------------------------------------------------------

class Inverter(BTNode):
    """Flips SUCCESS <-> FAILURE; passes RUNNING through."""

    def __init__(self, child: BTNode):
        self.child = child

    def tick(self) -> BTStatus:
        s = self.child.tick()
        if s == BTStatus.SUCCESS:
            return BTStatus.FAILURE
        if s == BTStatus.FAILURE:
            return BTStatus.SUCCESS
        return BTStatus.RUNNING


class ForceSuccess(BTNode):
    """Always returns SUCCESS regardless of child result."""

    def __init__(self, child: BTNode):
        self.child = child

    def tick(self) -> BTStatus:
        self.child.tick()
        return BTStatus.SUCCESS
