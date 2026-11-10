"""Arms and hands: carries out one arm command at a time from the task.

Inputs:  low_state      from sim, 500 Hz
         arm_command    [id, kind, x, y, z, yaw] from task (kinds: g1/messages.py)
Outputs: joint_command  28 joints (arms + hands): target, kp, kd, gravity torque
         arm_status     [id, ok] when a command is finished (or can't be done)

It works every 20 ms of robot time, like the locomotion node.
"""

import os

import pyarrow as pa
from dora import Node

from g1 import messages, robot
from g1.arm import ArmController

UPDATE_DT = 0.02  # s

arm = ArmController(os.environ["ROBOT"])
command_id = None  # the command in progress
time = 0.0
next_update = 0.0
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "arm_command":
        new_id, kind, position, yaw = messages.unpack_arm_command(
            event["value"].to_numpy()
        )
        if arm.execute(time, kind, position, yaw):
            command_id = new_id
        else:
            node.log_warn(f"arm command {new_id}: can't reach {position.round(3)}")
            status = messages.pack_arm_status(new_id, False)
            node.send_output("arm_status", pa.array(status))

    elif event["id"] == "low_state":
        time, q, dq, quat, gyro = messages.unpack_low_state(event["value"].to_numpy())
        if time < next_update - 1e-6:
            continue  # not 20 ms since the last update yet
        next_update = time + UPDATE_DT

        target, kp, kd, tau = arm.update(time, q[robot.UPPER_BODY])
        command = messages.pack_joint_command(target, kp, kd, tau)
        node.send_output("joint_command", pa.array(command))
        if command_id is not None and not arm.busy():
            status = messages.pack_arm_status(command_id, True)
            node.send_output("arm_status", pa.array(status))
            command_id = None
