import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pyarrow as pa
from dora import Node

from world import Pose, collides, lidar, step

DT = 0.02

node = Node()
pose = Pose(0.0, 0.0, 0.0)
v, w = 0.0, 0.0
collisions = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "cmd_vel":
        v, w = event["value"].to_pylist()
    if event["id"] == "tick":
        candidate = step(pose, v, w, DT)
        if collides(candidate):
            collisions += 1
            if collisions % 25 == 1:
                logging.warning("bumper: blocked at x=%.2f y=%.2f", pose.x, pose.y)
        else:
            pose = candidate
        node.send_output("pose", pa.array([pose.x, pose.y, pose.theta]))
        node.send_output("scan", pa.array(lidar(pose)))
