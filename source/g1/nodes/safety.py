"""dora node: the safety layer between the planner and the robot.

Every walking command passes through here; it is only ever slowed down or
stopped, never made faster (speed and separation monitoring, ISO/TS 15066;
the same place in the graph as ROS 2 Nav2's Collision Monitor).

Inputs:  cmd_vel_nav  the planner's walking command (50 Hz)
         tracks       the people the robot knows about, from tracker (10 Hz)
         base_pose    where the robot is, from sim (50 Hz)
Outputs: cmd_vel      the command the robot may carry out (to sim and back to the planner)

The logic is g1.safety.SpeedSeparationMonitor; this node only passes messages.
"""

import numpy as np
import pyarrow as pa
from dora import Node

from g1 import messages
from g1.safety import SpeedSeparationMonitor

safety = SpeedSeparationMonitor()
pose = None
tracks = np.zeros((0, 4))
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "base_pose":
        _, (x, y), heading = messages.unpack_pose(event["value"].to_numpy())
        pose = (x, y, heading)
    elif event["id"] == "tracks":
        _, tracks = messages.unpack_tracks(event["value"].to_numpy())
    elif event["id"] == "cmd_vel_nav":
        wanted = messages.unpack_cmd_vel(event["value"].to_numpy())
        # no pose yet: nothing is known about the surroundings, so don't move
        cmd = safety.limit(wanted, pose, tracks) if pose is not None else (0.0, 0.0, 0.0)
        node.send_output("cmd_vel", pa.array(messages.pack_cmd_vel(cmd)))
