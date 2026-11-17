"""The pick-and-place task's world, as plain numbers (used by the Isaac Lab
environment, the mission and the tests).

The robot starts at the origin facing +x; table A is in front of it, table B
to its right. Both tables are 0.8 m high: the G1's hand reaches best a little
above its pelvis (about 0.75 m).
"""

import math

START = (0.0, 0.0, 0.0)
TABLE_SIZE = (0.6, 0.8, 0.8)  # depth, width, height
TABLE_A = (1.6, 0.0)  # center, in front of the robot
TABLE_B = (0.6, -2.0)  # center, to the robot's right
BOTTLE_START = (1.38, 0.0)  # on table A, 8 cm from its near edge
PLACE_MARK = (0.6, -1.78)  # on table B, 8 cm from its near edge
BOTTLE_HEIGHT = 0.15  # m: the bottle (area.scene.bottle)
HEADING_AT_A = 0.0  # face +x at table A
HEADING_AT_B = -math.pi / 2  # face -y at table B

TIME_LIMIT = 120.0  # s


def table_box(center, size=TABLE_SIZE, heading=0.0):
    """A table's footprint (x_min, y_min, x_max, y_max); heading: which way the
    robot faces when it works at it (table B is turned by 90 degrees)."""
    depth, width = (size[0], size[1]) if abs(math.sin(heading)) < 0.5 else (size[1], size[0])
    return (center[0] - depth / 2, center[1] - width / 2, center[0] + depth / 2, center[1] + width / 2)
