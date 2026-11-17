"""dora node: picks a walking command toward the goal that keeps clear of people.

Inputs:  tracks       the people the robot knows about, from tracker (10 Hz)
         base_pose    where the robot is, from sim (50 Hz)
         cmd_vel      the command the safety node really let through: the
                      smoothing continues from it (like Nav2's velocity smoother)
Outputs: cmd_vel_nav  walking command: forward, left, turn (on every base_pose), to safety

The logic is g1.planner.Planner; this node only passes messages.
Settings: GOAL "x,y" (m).
"""

import os

import numpy as np
import pyarrow as pa
from dora import Node

from g1 import messages
from g1.planner import Planner

planner = Planner(tuple(float(v) for v in os.environ["GOAL"].split(",")))
position = None
last_time = None
arrived_reported = False
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "tracks" and position is not None:
        _, tracks = messages.unpack_tracks(event["value"].to_numpy())
        planner.plan(position, tracks)
    elif event["id"] == "cmd_vel":
        planner.cmd[:] = messages.unpack_cmd_vel(event["value"].to_numpy())
    elif event["id"] == "base_pose":
        time, position, heading = messages.unpack_pose(event["value"].to_numpy())
        dt = 0.0 if last_time is None else time - last_time  # robot time since the last command
        last_time = time
        cmd = planner.to_cmd_vel(heading, dt)
        node.send_output("cmd_vel_nav", pa.array(messages.pack_cmd_vel(cmd)))
        if planner.arrived and not np.any(cmd) and not arrived_reported:
            node.log_info(f"t={time:5.1f}s  arrived at the goal")
            arrived_reported = True
