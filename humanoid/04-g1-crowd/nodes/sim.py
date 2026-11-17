"""The robot, the square and the people: MuJoCo physics at 500 Hz, in a 3D window.

On the real robot this node is the G1 itself with its lidar. Here the
CrowdWorld class (crowd/world.py) plays that part, and also moves the people.

Inputs:  tick            2 ms timer: one physics step
         lower_body_cmd  joint commands for legs + waist (from locomotion)
         upper_body_cmd  joint commands for arms + hands (from arm)
         tracks          the people the robot knows about, only to draw them
Outputs: low_state       joint angles/speeds + IMU, every step (500 Hz)
         base_pose       where the robot is, every 10 steps (50 Hz)
         scan            one turn of the lidar, every 50 steps (10 Hz)

Red rings on the ground show where the robot thinks people are (its tracks).
macOS: the MuJoCo window needs mjpython (see dataflow.yml).
"""

import os
import time

import mujoco
import mujoco.viewer
import numpy as np
import pyarrow as pa
from dora import Node

from crowd import messages as crowd_messages
from crowd.world import LIDAR_RATE, CrowdWorld
from g1 import messages, robot

POSE_EVERY = 10  # steps per pose message: 500 Hz / 10 = 50 Hz
SCAN_EVERY = round(500 / LIDAR_RATE)  # steps per lidar turn: 500 / 10 = 50
DRAW_EVERY = 8  # ticks per window frame: 500 / 8 = about 60 frames per second
REPORT_EVERY = 5000  # steps between speed reports: every 10 s of robot time
RING = np.array([0.9, 0.1, 0.1, 0.6])  # colour of a track ring (red, see-through)

start = [float(v) for v in os.environ["START"].split(",")]
world = CrowdWorld(
    os.environ["SCENE"], start, int(os.environ["PEOPLE"]), int(os.environ["SEED"])
)
have_lower = have_upper = False
ticks = 0
steps = 0
started_at = None
tracks = np.zeros((0, 4))
warned = False


def draw_tracks(scene):
    """A flat red ring under every person the robot is tracking."""
    scene.ngeom = 0
    for x, y, _, _ in tracks[: scene.maxgeom]:
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [0.35, 0.004, 0],
            [x, y, 0.006],
            np.eye(3).ravel(),
            RING,
        )
        scene.ngeom += 1


with mujoco.viewer.launch_passive(world.model, world.data) as viewer:
    viewer.cam.lookat[:] = [0.0, 0.0, 0.5]  # the middle of the square
    viewer.cam.distance = 17
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -50
    viewer.sync()  # draw the first (slow) frame before connecting
    node = Node()

    for event in node:
        if event["type"] == "STOP" or not viewer.is_running():
            break
        if event["type"] != "INPUT":
            continue

        if event["id"] == "lower_body_cmd":
            command = messages.unpack_joint_command(event["value"].to_numpy())
            world.set_command(robot.LOWER_BODY, *command)
            have_lower = True

        elif event["id"] == "upper_body_cmd":
            command = messages.unpack_joint_command(event["value"].to_numpy())
            world.set_command(robot.UPPER_BODY, *command)
            have_upper = True

        elif event["id"] == "tracks":
            _, tracks = crowd_messages.unpack_tracks(event["value"].to_numpy())

        elif event["id"] == "tick":
            ticks += 1
            if have_lower and have_upper:
                world.step()
                steps += 1
                if steps % POSE_EVERY == 0:
                    pose = messages.pack_pose(world.time, *world.base_pose())
                    node.send_output("base_pose", pa.array(pose))
                if steps % SCAN_EVERY == 0:
                    scan = crowd_messages.pack_scan(world.time, world.scan())
                    node.send_output("scan", pa.array(scan))
                if world.touched and not warned:
                    node.log_warn(f"t={world.time:5.1f}s  a person touched the robot")
                    warned = True

            q, dq = world.joint_state()
            quat, gyro = world.imu()
            state = messages.pack_low_state(world.time, q, dq, quat, gyro)
            node.send_output("low_state", pa.array(state))

            if ticks % DRAW_EVERY == 0:
                with viewer.lock():
                    draw_tracks(viewer.user_scn)
                viewer.sync()
            if steps == 1:
                started_at = time.monotonic()
            elif steps > 0 and steps % REPORT_EVERY == 0:
                speed = world.time / (time.monotonic() - started_at)
                node.log_info(
                    f"t={world.time:5.1f}s  running at {100 * speed:.0f}% of real time"
                )
