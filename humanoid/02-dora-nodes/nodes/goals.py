"""The task: which angle the arm should go to next.

On a real robot this would be a mission planner or an operator.
Here it walks through a list of goals, one per timer tick, and starts
over at the end, so the arm keeps moving until you stop the dataflow.
"""

import math
import os

import pyarrow as pa
from dora import Node

GOALS_DEG = [float(g) for g in os.environ["GOALS_DEG"].split(",")]

node = Node()
next_goal = 0

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        goal = GOALS_DEG[next_goal]
        node.log_info(f"new goal: {goal:.0f} deg")
        node.send_output("goal", pa.array([math.radians(goal)]))
        next_goal = (next_goal + 1) % len(GOALS_DEG)  # after the last one, back to the first
