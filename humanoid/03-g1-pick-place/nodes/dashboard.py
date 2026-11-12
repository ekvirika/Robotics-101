"""What the robot does, live in Rerun: a 3D map of the room, charts, the mission log.

It only listens, it sends nothing. You can remove it from the dataflow and the
robot does exactly the same thing.

Inputs:  base_pose     where the robot is and how far it leans, 50 Hz (sim)
         bottle_pose   where the bottle is, 50 Hz (sim)
         cmd_vel       the walking command (task)
         standing      1 = legs stand still and steady (locomotion)
         arm_command   the hand target, in the body frame (task)
         arm_status    which arm command finished (arm)
         step          the mission step as text (task)
         tick          draw timer

The tables and the green mark come from robot/scene.xml, so the map always
matches the simulation.

VIEWER=rerun (default) opens the Rerun window, VIEWER=save writes out/g1.rrd.
"""

import os

import mujoco
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from dora import Node

from g1 import frames, messages
from g1.simulator import yaw_of

TRAIL_POINTS = 2000  # how much of the walked path to keep (50 Hz: 40 s)
HAND_KINDS = {
    messages.REST: "arm down",
    messages.MOVE_JOINTS: "joint move",
    messages.MOVE_LINE: "line move",
    messages.OPEN: "open hand",
    messages.CLOSE: "close hand",
}
WOOD = (140, 97, 56)
WHITE = (225, 225, 220)
GREEN = (50, 200, 80)
BLUE = (50, 140, 230)
ROBOT = (240, 150, 60)
HAND = (230, 70, 160)


def start_rerun():
    if os.environ.get("VIEWER", "rerun") == "save":
        os.makedirs("out", exist_ok=True)
        rr.init("g1-dashboard")
        rr.save("out/g1.rrd")
    else:
        rr.init("g1-dashboard", spawn=True)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(name="room", origin="world"),
                rrb.Vertical(
                    rrb.TextLogView(name="mission", origin="log"),
                    rrb.TimeSeriesView(name="walk command", origin="cmd_vel"),
                    rrb.TimeSeriesView(name="balance", origin="balance"),
                    rrb.TimeSeriesView(name="standing", origin="standing"),
                ),
                column_shares=[3, 2],
            ),
            collapse_panels=True,
        )
    )


def log_room(scene_path):
    """Tables and the place mark, read from the MuJoCo scene. Logged once."""
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for name, color in (("table_a_top", WOOD), ("table_b_top", WHITE)):
        geom = model.geom(name).id
        rr.log(
            f"world/{name}",
            rr.Boxes3D(
                centers=[data.geom_xpos[geom]],
                half_sizes=[model.geom_size[geom]],
                colors=[color],
                fill_mode="solid",
            ),
            static=True,
        )
    mark = data.site("place_target").xpos
    rr.log(
        "world/place_mark",
        rr.Points3D([mark], radii=[0.04], colors=[GREEN], labels=["mark"]),
        static=True,
    )


start_rerun()
log_room(os.environ["SCENE"])
node = Node()

robot_position = None
robot_heading = 0.0
trail = []
time = 0.0

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    name = event["id"]

    if name == "base_pose":
        time, robot_position, quat = messages.unpack_pose(event["value"].to_numpy())
        robot_heading = yaw_of(quat)
        rr.set_time("sim_time", duration=time)
        rr.log("balance/tilt_deg", rr.Scalars(np.degrees(frames.tilt_of(quat))))
        trail.append(robot_position.copy())
        del trail[:-TRAIL_POINTS]

    elif name == "bottle_pose":
        _, bottle, _ = messages.unpack_pose(event["value"].to_numpy())
        rr.set_time("sim_time", duration=time)
        rr.log(
            "world/bottle",
            rr.Boxes3D(
                centers=[bottle], half_sizes=[[0.03, 0.03, 0.075]], colors=[BLUE]
            ),
        )

    elif name == "cmd_vel":
        forward, left, turn = event["value"].to_numpy()
        rr.set_time("sim_time", duration=time)
        rr.log("cmd_vel/forward_m_s", rr.Scalars(forward))
        rr.log("cmd_vel/left_m_s", rr.Scalars(left))
        rr.log("cmd_vel/turn_rad_s", rr.Scalars(turn))

    elif name == "standing":
        rr.set_time("sim_time", duration=time)
        rr.log("standing/steady", rr.Scalars(event["value"].to_numpy()[0]))

    elif name == "step":
        rr.set_time("sim_time", duration=time)
        rr.log("log", rr.TextLog(event["value"][0].as_py(), level="INFO"))

    elif name == "arm_command" and robot_position is not None:
        command_id, kind, target, _ = messages.unpack_arm_command(
            event["value"].to_numpy()
        )
        rr.set_time("sim_time", duration=time)
        rr.log(
            "log",
            rr.TextLog(f"arm #{command_id}: {HAND_KINDS[kind]}", level="DEBUG"),
        )
        if kind in (messages.MOVE_JOINTS, messages.MOVE_LINE):
            world = frames.to_world(target, robot_position, robot_heading)
            rr.log(
                "world/hand_target",
                rr.Points3D([world], radii=[0.02], colors=[HAND], labels=["hand"]),
            )

    elif name == "arm_status":
        command_id, ok = messages.unpack_arm_status(event["value"].to_numpy())
        rr.set_time("sim_time", duration=time)
        level = "DEBUG" if ok else "ERROR"
        rr.log("log", rr.TextLog(f"arm #{command_id}: {'done' if ok else 'failed'}", level=level))

    elif name == "tick" and robot_position is not None:
        rr.set_time("sim_time", duration=time)
        ahead = 0.3 * np.array([np.cos(robot_heading), np.sin(robot_heading), 0.0])
        rr.log(
            "world/robot",
            rr.Arrows3D(origins=[robot_position], vectors=[ahead], colors=[ROBOT]),
        )
        rr.log(
            "world/robot_path",
            rr.LineStrips3D([np.array(trail)], colors=[ROBOT], radii=[0.005]),
        )
