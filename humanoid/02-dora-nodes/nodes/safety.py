"""Safety monitor: watches the robot and stops it if something is wrong.

Real robots run a separate safety check that doesn't trust the controller.
It triggers the emergency stop when:
- the arm leaves its allowed range,
- the arm moves faster than the plan ever allows,
- the robot goes silent (no joint state for INPUT_TIMEOUT, set in dataflow.yml).
"""

import math
import os

import pyarrow as pa
from dora import Node

MAX_ANGLE = math.radians(float(os.environ["MAX_ANGLE_DEG"]))
MAX_SPEED = float(os.environ["MAX_SPEED"])  # rad/s

node = Node()
stopped = False


def emergency_stop(reason):
    global stopped
    if not stopped:
        node.log_error(f"EMERGENCY STOP: {reason}")
        node.send_output("estop", pa.array([True]))
        stopped = True


for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT_CLOSED" and event["id"] == "joint_state":
        emergency_stop("no joint state from the robot")  # input_timeout fired
    if event["type"] == "INPUT" and event["id"] == "joint_state":
        time, angle, speed, torque = event["value"].to_pylist()
        if abs(angle) > MAX_ANGLE:
            emergency_stop(f"angle {math.degrees(angle):.0f} deg is out of range")
        if abs(speed) > MAX_SPEED:
            emergency_stop(f"speed {speed:.2f} rad/s is too fast")
