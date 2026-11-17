"""The pick-and-place task's logic, with the stand-in robot (tests/fakes.py).

Checks that every step can be carried out in this room: the robot docks at
both tables, the hand reaches the bottle (real arm kinematics), holds it,
and sets it down on the mark.
"""

import numpy as np
from fakes import FakeG1

from tasks.pick_and_place.layout import (
    BOTTLE_HEIGHT,
    BOTTLE_START,
    HEADING_AT_B,
    PLACE_MARK,
    START,
    TABLE_A,
    TABLE_B,
    TABLE_SIZE,
    TIME_LIMIT,
    table_box,
)
from tasks.pick_and_place.mission import PickAndPlace

ON_MARK = 0.03  # m: the bottle's center within 3 cm of the mark's center counts as on it


def test_bottle_ends_on_the_mark():
    robot = FakeG1(
        start=START,
        tables=[table_box(TABLE_A), table_box(TABLE_B, heading=HEADING_AT_B)],
        bottle=(BOTTLE_START[0], BOTTLE_START[1], TABLE_SIZE[2] + BOTTLE_HEIGHT / 2),
        table_height=TABLE_SIZE[2],
    )
    mission = PickAndPlace(robot)
    while not mission.done and robot.time() < TIME_LIMIT:
        mission.update()
        robot.step()
    off = float(np.hypot(robot.bottle[0] - PLACE_MARK[0], robot.bottle[1] - PLACE_MARK[1]))
    print(f"\nfinished at t={robot.time():.1f} s, bottle {100 * off:.1f} cm from the mark, failed: {mission.failed}")
    assert mission.done and mission.failed is None
    assert not robot.held  # let go
    assert abs(robot.bottle[2] - (TABLE_SIZE[2] + BOTTLE_HEIGHT / 2)) < 1e-6  # standing on table B
    assert off < ON_MARK
