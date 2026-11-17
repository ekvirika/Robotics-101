"""Navigation: walks the robot to the goal without bumping into anyone.

Inputs:  tracks     the people around the robot (10 Hz), from tracker
         base_pose  where the robot is (50 Hz), from sim (real robot: localization)
Outputs: cmd_vel    walking velocity (forward, left, turn), to locomotion (50 Hz)

It plans on every `tracks` message (10 Hz) and steers on every `base_pose`
(50 Hz). The method is in crowd/planner.py.
"""

import os

import numpy as np
import pyarrow as pa
from dora import Node

from crowd import messages as crowd_messages
from crowd.planner import Planner
from g1 import messages
from g1.simulator import yaw_of

REPORT_EVERY = 5.0  # s between progress lines

GOAL = [float(v) for v in os.environ["GOAL"].split(",")]
planner = Planner(GOAL)
position = None
last_time = None
next_report = 0.0
reported = False
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "tracks" and position is not None:
        _, people = crowd_messages.unpack_tracks(event["value"].to_numpy())
        planner.plan(position, people)
        time = last_time
        if planner.arrived and not reported:
            node.log_info(f"t={time:5.1f}s  arrived at the goal")
            reported = True
        elif not planner.arrived and time >= next_report:
            next_report = time + REPORT_EVERY
            distance = np.hypot(GOAL[0] - position[0], GOAL[1] - position[1])
            node.log_info(
                f"t={time:5.1f}s  {distance:4.1f} m to go, {len(people)} people in view"
            )

    elif event["id"] == "base_pose":
        time, position, quat = messages.unpack_pose(event["value"].to_numpy())
        dt = 0.0 if last_time is None else time - last_time
        last_time = time
        cmd_vel = planner.to_cmd_vel(yaw_of(quat), dt)
        node.send_output("cmd_vel", pa.array(cmd_vel))
