"""Perception: turns lidar scans into a list of people and their velocities.

Inputs:  scan       one turn of the lidar (10 Hz), from sim
         base_pose  where the robot is (50 Hz), from sim (real robot: localization)
Outputs: tracks     [time, then x, y, vx, vy per person], world frame (10 Hz)

The method is in crowd/tracker.py.
"""

import pyarrow as pa
from dora import Node

from crowd import messages as crowd_messages
from crowd.tracker import Tracker
from g1 import messages
from g1.simulator import yaw_of

tracker = Tracker()
pose = None  # the robot's latest (x, y, heading)
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "base_pose":
        _, position, quat = messages.unpack_pose(event["value"].to_numpy())
        pose = (position[0], position[1], yaw_of(quat))

    elif event["id"] == "scan" and pose is not None:
        time, ranges = crowd_messages.unpack_scan(event["value"].to_numpy())
        tracks = tracker.update(ranges, pose)
        message = crowd_messages.pack_tracks(time, tracks)
        node.send_output("tracks", pa.array(message))
