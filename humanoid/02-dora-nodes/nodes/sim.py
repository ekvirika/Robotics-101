"""The robot. Here it's MuJoCo; on the real robot this node is the motor driver.

Every tick (2 ms): apply the latest motor torque, step the physics, and
send the joint state, like a real joint's encoder would.
"""

import os

import mujoco
import pyarrow as pa
from dora import Node

model = mujoco.MjModel.from_xml_path(os.environ["MODEL"])
data = mujoco.MjData(model)
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "torque":
        data.ctrl[0] = event["value"][0].as_py()  # the motor clips it to +-20 Nm

    elif event["id"] == "tick":
        mujoco.mj_step(model, data)  # physics: move forward 2 ms
        # [time (s), angle (rad), speed (rad/s), torque the motor really gave (Nm)]
        node.send_output(
            "joint_state",
            pa.array([data.time, data.qpos[0], data.qvel[0], data.actuator_force[0]]),
        )
