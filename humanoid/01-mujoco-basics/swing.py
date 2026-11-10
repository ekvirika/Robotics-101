"""Lesson 1 without a window: the arm starts hanging and lifts to the target. Prints numbers.

Run:  uv run python swing.py               (target 90 degrees)
      uv run python swing.py --target 45
"""

import argparse
import math
from pathlib import Path

import mujoco

from controller import joint_torque
from trajectory import Trajectory

parser = argparse.ArgumentParser()
parser.add_argument("--target", type=float, default=90.0, help="target angle in degrees")
args = parser.parse_args()

# model = the fixed description of the robot (parts, joints, masses), from the XML
model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("pendulum.xml")))
# data = the state that changes every step (angles, speeds, motor commands)
data = mujoco.MjData(model)

print(f"arm mass: {model.body('arm').mass[0]:.2f} kg, physics step: {model.opt.timestep} s")
goal = math.radians(args.target)
trajectory = Trajectory(start_angle=data.qpos[0])

for step in range(2500):  # 2500 x 2 ms = 5 seconds
    ref_angle, ref_speed, ref_accel = trajectory.step(goal, model.opt.timestep)
    data.ctrl[0] = joint_torque(
        ref_angle=ref_angle,
        ref_speed=ref_speed,
        ref_accel=ref_accel,
        angle=data.qpos[0],  # qpos = joint angle (rad)
        speed=data.qvel[0],  # qvel = joint speed (rad/s)
        inertia=data.M[0],  # the arm's inertia around the shoulder, from MuJoCo
        weight_torque=data.qfrc_bias[0],  # torque the arm's weight needs right now
    )
    mujoco.mj_step(model, data)  # physics: move forward 2 ms

    if step % 100 == 0:  # print every 0.2 s
        print(
            f"t={data.time:4.1f}s  angle={math.degrees(data.qpos[0]):6.1f} deg  "
            f"speed={math.degrees(data.qvel[0]):5.1f} deg/s  motor={data.actuator_force[0]:6.2f} Nm"
        )
