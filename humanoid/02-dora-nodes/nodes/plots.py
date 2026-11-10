"""Live charts in Rerun: goal, plan and real angle on one chart, motor torque on another.

The robot sends 500 states a second; a chart doesn't need that many.
So this node only keeps the newest values and draws them on its own timer.
"""

import math
import os

import rerun as rr
from dora import Node

if os.environ.get("VIEWER", "rerun") == "save":
    os.makedirs("out", exist_ok=True)
    rr.init("arm-plots")
    rr.save("out/arm-plots.rrd")
else:
    rr.init("arm-plots", spawn=True)

node = Node()
state = None  # newest [time, angle, speed, torque]
plan = None  # newest planned angle
goal = None  # newest goal

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "joint_state":
        state = event["value"].to_pylist()
    elif event["id"] == "reference":
        plan = event["value"][0].as_py()
    elif event["id"] == "goal":
        goal = event["value"][0].as_py()

    elif event["id"] == "tick" and state is not None:
        time, angle, speed, torque = state
        rr.set_time("sim_time", duration=time)
        rr.log("angle_deg/real", rr.Scalars(math.degrees(angle)))
        rr.log("torque_nm/motor", rr.Scalars(torque))
        if plan is not None:
            rr.log("angle_deg/plan", rr.Scalars(math.degrees(plan)))
        if goal is not None:
            rr.log("angle_deg/goal", rr.Scalars(math.degrees(goal)))
