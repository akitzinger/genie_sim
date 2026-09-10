# Copyright (c) 2026, Alexander Kitzinger
# Author: Alexander Kitzinger
# License: MIT
"""The fixed dual-arm pick-and-place state set and the demo sequence order."""
from __future__ import annotations

from enum import Enum
from typing import List


class TaskState(str, Enum):
    DEFAULT = "default"
    PICK_READY = "pick_ready"
    PICK = "pick"
    PICK_HOLD = "pick_hold"
    PLACE_HOLD = "place_hold"
    PLACE_READY = "place_ready"
    PLACE = "place"


# One full pick-and-place cycle, used by the `run_cycle` service.
SEQUENCE: List[TaskState] = [
    TaskState.DEFAULT,
    TaskState.PICK_READY,
    TaskState.PICK,
    TaskState.PICK_HOLD,
    TaskState.PLACE_HOLD,
    TaskState.PLACE_READY,
    TaskState.PLACE,
    TaskState.DEFAULT,
]

# Linear transition order (no repeats), used to validate individual `go_*`
# transitions: from any state, only the next state in this list — or
# DEFAULT, from anywhere — is a legal transition. See
# DualArmPickPlaceTask._check_transition.
ORDER: List[TaskState] = [
    TaskState.DEFAULT,
    TaskState.PICK_READY,
    TaskState.PICK,
    TaskState.PICK_HOLD,
    TaskState.PLACE_HOLD,
    TaskState.PLACE_READY,
    TaskState.PLACE,
]
