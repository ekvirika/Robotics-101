"""The 3D window: shows the arm where the robot says it is.

It doesn't run physics. It keeps the newest measured angle and, on its own
timer (about 60 times a second), copies it into its own copy of the model
and draws it. macOS: the MuJoCo window needs mjpython (see dataflow.yml).
"""

import math
import os

import mujoco
import mujoco.viewer
from dora import Node

model = mujoco.MjModel.from_xml_path(os.environ["MODEL"])
scene = mujoco.MjData(model)  # only for drawing

with mujoco.viewer.launch_passive(model, scene) as viewer:
    viewer.cam.lookat[:] = [0, 0, 0.7]
    viewer.cam.distance = 2.8
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -10
    viewer.sync()  # draw the first frame (the slowest one) before any messages come
    node = Node()  # connect only now, so messages don't pile up while the window opens

    for event in node:
        if event["type"] == "STOP" or not viewer.is_running():
            break
        if event["type"] != "INPUT":
            continue

        if event["id"] == "joint_state":
            scene.qpos[0] = event["value"][1].as_py()  # [time, angle, speed, torque]

        elif event["id"] == "goal":
            goal = event["value"][0].as_py()
            # turn the green ghost to the goal (quaternion for a turn around y)
            scene.mocap_quat[0] = [math.cos(goal / 2), 0, math.sin(goal / 2), 0]

        elif event["id"] == "tick":
            mujoco.mj_forward(model, scene)  # update positions for drawing
            viewer.sync()
