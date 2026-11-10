"""Lesson 1: a one-motor robot arm in a 3D window.

Run (on macOS the viewer needs `mjpython` instead of `python`):
    uv run mjpython watch.py

Right panel -> Control -> target_angle_deg: drag it, the arm follows.
(motor_torque_AUTO is set by the controller: just watch it.)
Backspace = reset. Space = pause. Double-click the arm, then Ctrl + right-drag = push it.
"""

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from controller import joint_torque
from trajectory import Trajectory

model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("pendulum.xml")))
data = mujoco.MjData(model)

MOTOR, TARGET = 0, 1  # positions in data.ctrl
START_TARGET = 90.0
trajectory = Trajectory(start_angle=data.qpos[0])


def step():
    target = math.radians(data.ctrl[TARGET])
    # move the green ghost to the target (a rotation around y, stored as a quaternion)
    data.mocap_quat[0] = [math.cos(target / 2), 0, math.sin(target / 2), 0]

    ref_angle, ref_speed, ref_accel = trajectory.step(target, model.opt.timestep)
    data.ctrl[MOTOR] = joint_torque(
        ref_angle=ref_angle,
        ref_speed=ref_speed,
        ref_accel=ref_accel,
        angle=data.qpos[0],  # joint angle (rad)
        speed=data.qvel[0],  # joint speed (rad/s)
        inertia=data.M[0],  # the arm's inertia around the shoulder, from MuJoCo
        weight_torque=data.qfrc_bias[0],  # MuJoCo computes it: 2.06 kg x 9.81 x 0.319 m x sin(angle)
    )
    mujoco.mj_step(model, data)  # physics: move forward 2 ms


data.ctrl[TARGET] = START_TARGET
last_time = 0.0
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0, 0, 0.7]
    viewer.cam.distance = 2.8
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -10

    while viewer.is_running():
        frame_start = time.time()
        with viewer.lock():
            if data.time < last_time:  # Backspace was pressed
                data.ctrl[TARGET] = START_TARGET
                trajectory = Trajectory(start_angle=data.qpos[0])
            for _ in range(8):  # 8 x 2 ms = 16 ms per frame, ~60 fps
                step()
            last_time = data.time
            numbers = (
                f"{math.degrees(data.qpos[0]):6.1f} deg\n"
                f"{data.ctrl[TARGET]:6.1f} deg\n"
                f"{data.actuator_force[MOTOR]:6.2f} Nm\n"
                f"{data.qfrc_bias[0]:6.2f} Nm"
            )
        viewer.set_texts(
            (
                mujoco.mjtFontScale.mjFONTSCALE_150,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                "arm angle\ntarget (green)\nmotor torque\nweight needs",
                numbers,
            )
        )
        viewer.sync()
        time.sleep(max(0.0, 8 * model.opt.timestep - (time.time() - frame_start)))
