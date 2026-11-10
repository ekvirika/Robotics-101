import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pyarrow as pa
from dora import Node

from controller import command

GOALS = [(7.5, 0.0), (6.0, 3.5), (1.0, 3.0), (3.5, -3.5), (0.0, 0.0)]
REACHED = 0.3

node = Node()
pose = None
goal_index = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "pose":
        pose = event["value"].to_pylist()
    if event["id"] == "scan" and pose is not None:
        x, y, theta = pose
        goal = GOALS[goal_index % len(GOALS)]
        if math.hypot(goal[0] - x, goal[1] - y) < REACHED:
            print(
                f"planner: reached goal {goal_index + 1} "
                f"at ({goal[0]:.1f}, {goal[1]:.1f})",
                flush=True,
            )
            goal_index += 1
            goal = GOALS[goal_index % len(GOALS)]
        v, w = command(x, y, theta, goal, event["value"].to_numpy())
        node.send_output("cmd_vel", pa.array([v, w]))
        node.send_output("goal", pa.array(list(goal)))
