"""Legs and waist: walk with Unitree's trained policy, or stand still and balance.

Inputs:  low_state      from sim, 500 Hz
         cmd_vel        wanted walking velocity (forward, left, turn), from task
Outputs: joint_command  15 joints (legs + waist): target, kp, kd, torque
         standing       1 = standing still and steady, 0 = walking or stopping

It works every 20 ms of robot time (the policy's rate), measured with the
robot's own clock in low_state, so it stays in step with the physics even
if the computer is busy.
"""

import os

import numpy as np
import pyarrow as pa
from dora import Node

from g1 import messages, robot
from g1.locomotion import POLICY_DT, Locomotion

locomotion = Locomotion(os.environ["POLICY"])
cmd_vel = (0.0, 0.0, 0.0)
next_update = 0.0
no_extra_torque = np.zeros(15)  # the policy was trained without extra torque
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "cmd_vel":
        cmd_vel = tuple(event["value"].to_numpy())

    elif event["id"] == "low_state":
        state = messages.unpack_low_state(event["value"].to_numpy())
        time, q, dq, quat, gyro = state
        if time < next_update - 1e-6:
            continue  # not 20 ms since the last update yet
        next_update = time + POLICY_DT

        legs_q, legs_dq = q[robot.LEGS], dq[robot.LEGS]
        target, kp, kd = locomotion.update(time, cmd_vel, legs_q, legs_dq, quat, gyro)
        command = messages.pack_joint_command(target, kp, kd, no_extra_torque)
        node.send_output("joint_command", pa.array(command))
        standing = 1.0 if locomotion.is_standing(time) else 0.0
        node.send_output("standing", pa.array([standing]))
