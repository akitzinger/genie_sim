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
    HOLD = "hold"
    PLACE_READY = "place_ready"
    PLACE = "place"


# One full pick-and-place cycle, used by the `run_cycle` service.
SEQUENCE: List[TaskState] = [
    TaskState.DEFAULT,
    TaskState.PICK_READY,
    TaskState.PICK,
    TaskState.HOLD,
    TaskState.PLACE_READY,
    TaskState.PLACE,
    TaskState.DEFAULT,
]
