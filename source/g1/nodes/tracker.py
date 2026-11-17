"""dora node: finds the people in each lidar scan and follows them.

Inputs:  scan       one turn of the lidar, from sim (10 Hz)
         base_pose  where the robot is, from sim (50 Hz)
Outputs: tracks     the people the robot knows about: x, y, vx, vy each (10 Hz)

The logic is g1.tracker.Tracker; this node only passes messages.
"""

import pyarrow as pa
from dora import Node

from g1 import messages
from g1.tracker import Tracker

tracker = Tracker()
pose = None
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "base_pose":
        _, (x, y), heading = messages.unpack_pose(event["value"].to_numpy())
        pose = (x, y, heading)
    elif event["id"] == "scan" and pose is not None:
        time, ranges = messages.unpack_scan(event["value"].to_numpy())
        tracks = tracker.update(ranges, pose)
        node.send_output("tracks", pa.array(messages.pack_tracks(time, tracks)))
