"""The joint controller: follows the plan (same controller.py as lesson 1).

Real robots keep their own copy of the robot model (URDF/MJCF) to work out
the weight and inertia terms. This node does the same: it loads pendulum.xml
and asks MuJoCo for those two numbers at the measured angle.
"""

import os

import mujoco
import pyarrow as pa
from dora import Node

from controller import KD, joint_torque  # from ../01-mujoco-basics

model = mujoco.MjModel.from_xml_path(os.environ["MODEL"])
robot = mujoco.MjData(model)  # our own copy, only used for calculations
node = Node()
reference = None
stopped = False

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "reference":
        reference = event["value"].to_pylist()

    elif event["id"] == "estop":
        if not stopped:
            node.log_warn("emergency stop: braking, ignoring the plan")
        stopped = True

    elif event["id"] == "joint_state":
        time, angle, speed, _ = event["value"].to_pylist()

        # ask the model: weight torque and inertia at this angle
        robot.qpos[0] = angle
        robot.qvel[0] = speed
        mujoco.mj_forward(model, robot)
        weight_torque = robot.qfrc_bias[0]

        if stopped:
            torque = -KD * speed  # only brake, like a real e-stop
        elif reference is None:
            torque = weight_torque  # no plan yet: just hold the arm up
        else:
            ref_angle, ref_speed, ref_accel = reference
            torque = joint_torque(
                ref_angle=ref_angle,
                ref_speed=ref_speed,
                ref_accel=ref_accel,
                angle=angle,
                speed=speed,
                inertia=robot.M[0],
                weight_torque=weight_torque,
            )
        node.send_output("torque", pa.array([torque]))
