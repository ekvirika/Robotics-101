"""The tell-the-robot room, as plain numbers: the pick-and-place room (same
tables, same mark, same start) with several objects in it.

Each table is worked from one side, as in the pick-and-place task: table A
facing +x, table B facing -y. Objects stand on a table; "depth" is how far an
object is from the edge the robot works at.
"""

import math
from dataclasses import dataclass

from tasks.pick_and_place.layout import (  # noqa: F401  the same room
    HEADING_AT_A,
    HEADING_AT_B,
    PLACE_MARK,
    START,
    TABLE_A,
    TABLE_B,
    TABLE_SIZE,
    table_box,
)

HEIGHT = {"bottle": 0.15, "can": 0.12, "box": 0.10}  # m, standing upright


@dataclass(frozen=True)
class Thing:
    name: str  # what the simulator calls it (Perception.find)
    color: str
    kind: str
    xy: tuple  # where it starts, on a table

    @property
    def height(self):
        return HEIGHT[self.kind]


OBJECTS = (
    Thing("blue_bottle", "blue", "bottle", (1.38, 0.0)),
    Thing("red_can", "red", "can", (1.38, 0.24)),
    Thing("yellow_bottle", "yellow", "bottle", (1.38, -0.24)),
    Thing("green_box", "green", "box", (0.32, -1.78)),
)

TABLES = {  # name: (center, heading the robot faces when it works there)
    "table_a": (TABLE_A, HEADING_AT_A),
    "table_b": (TABLE_B, HEADING_AT_B),
}

VIEWPOINTS = {  # where the robot stands to look at each table: 1 m in front of its working edge
    "table_a": (TABLE_A[0] - TABLE_SIZE[0] / 2 - 1.0, TABLE_A[1], HEADING_AT_A),
    "table_b": (TABLE_B[0], TABLE_B[1] + TABLE_SIZE[0] / 2 + 1.0, HEADING_AT_B),
}

PLACES = {  # go_to(place)
    "start": START,
    "table_a": VIEWPOINTS["table_a"],
    "table_b": VIEWPOINTS["table_b"],
    "mark": VIEWPOINTS["table_b"],
}

TIME_LIMIT = 300.0  # s, for one spoken or typed request


def box_of(table):
    center, heading = TABLES[table]
    return table_box(center, heading=heading)


def table_under(xy, margin=0.0):
    """The table whose top is under (x, y), or None."""
    for name in TABLES:
        x0, y0, x1, y1 = box_of(name)
        if x0 + margin <= xy[0] <= x1 - margin and y0 + margin <= xy[1] <= y1 - margin:
            return name
    return None


def depth_on(table, xy):
    """How far (x, y) is from the edge the robot works at (m)."""
    center, heading = TABLES[table]
    edge = TABLE_SIZE[0] / 2
    c, s = math.cos(heading), math.sin(heading)
    return (xy[0] - center[0]) * c + (xy[1] - center[1]) * s + edge
