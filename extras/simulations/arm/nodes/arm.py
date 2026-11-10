import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pyarrow as pa
from dora import Node

from kinematics import forward

DT = 0.01
KP = 40.0
KD = 9.0
MAX_SPEED = 3.0

node = Node()
q = [0.0, 0.0]
dq = [0.0, 0.0]
target = [0.0, 0.0]
target_velocity = [0.0, 0.0]
TARGET_PERIOD = 0.05
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "joint_targets":
        new_target = event["value"].to_pylist()
        target_velocity = [
            math.atan2(math.sin(n - o), math.cos(n - o)) / TARGET_PERIOD
            for n, o in zip(new_target, target)
        ]
        target = new_target
    if event["id"] == "tick":
        for i in range(2):
            error = math.atan2(math.sin(target[i] - q[i]), math.cos(target[i] - q[i]))
            acceleration = KP * error + KD * (target_velocity[i] - dq[i])
            dq[i] = max(-MAX_SPEED, min(MAX_SPEED, dq[i] + acceleration * DT))
            q[i] += dq[i] * DT
        elbow, hand = forward(*q)
        node.send_output("joint_state", pa.array(q + dq))
        node.send_output("hand", pa.array(list(hand)))
