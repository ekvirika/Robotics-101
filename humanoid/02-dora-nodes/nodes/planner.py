"""The plan: turns a goal into a smooth trajectory (same trajectory.py as lesson 1).

Runs once per joint state, so it plans at the same rate the robot moves.
"""

import pyarrow as pa
from dora import Node

from trajectory import Trajectory  # from ../01-mujoco-basics (PYTHONPATH in dataflow.yml)

node = Node()
goal = None
trajectory = None
last_time = None

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue

    if event["id"] == "goal":
        goal = event["value"][0].as_py()

    elif event["id"] == "joint_state":
        time, angle, speed, torque = event["value"].to_pylist()
        if trajectory is None:  # first message: start the plan where the arm is
            trajectory = Trajectory(start_angle=angle)
            last_time = time
            continue
        if goal is None:  # no goal yet: stay where we are
            goal = angle

        dt = time - last_time  # time since the last state, from the robot's clock
        last_time = time
        ref_angle, ref_speed, ref_accel = trajectory.step(goal, dt)
        node.send_output("reference", pa.array([ref_angle, ref_speed, ref_accel]))
