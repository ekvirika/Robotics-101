"""dora node: tell the robot, live in Rerun, on the computer with the screen.

It only listens and sends nothing: without it the robot does exactly the same.
Isaac's livestream viewer shows the robot itself; this shows what the robot
thinks: what it was told, what it remembers and where, what it does now.

Inputs:  base_pose  time, x, y, heading of the robot (sim), drawn at most 10 times a second
         objects    the memory (skills): id, color, kind, position, table, in_hand, can_pick
         held       the object in the hand, or -1 (skills)
         status     the step running now (skills)
         log        what was heard, each skill call and result, each reply (agent)

The room (tables, the green mark) comes from layout.py, the same numbers the
skills and the simulator use.

DASHBOARD_LAYOUT=audience (default) shows what reads from across a room: the map
and the conversation. DASHBOARD_LAYOUT=engineer adds the memory as a table and
the skill steps. VIEWER=save writes out/tell_the_robot.rrd instead of opening Rerun.
"""

import json
import os

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from dora import Node
from g1 import messages

from tasks.tell_the_robot.layout import HEIGHT, PLACE_MARK, TABLE_SIZE, TABLES, box_of

LAYOUT = os.environ.get("DASHBOARD_LAYOUT", "audience")
ROBOT_EVERY = 0.1  # s of robot time between drawn robot poses
TRAIL_POINTS = 600  # how much of the walked path to keep (10 Hz: 60 s)
COLORS = {"red": (220, 50, 50), "yellow": (240, 200, 30), "green": (60, 180, 80), "blue": (60, 130, 230)}
TABLE_COLORS = {"table_a": (140, 97, 56), "table_b": (225, 225, 220)}
MARK = (50, 200, 80)
ROBOT = (240, 150, 60)
WIDTH = {"bottle": 0.06, "can": 0.066, "box": 0.05}


def start_rerun():
    if os.environ.get("VIEWER", "rerun") == "save":
        os.makedirs("out", exist_ok=True)
        rr.init("tell-the-robot")
        rr.save("out/tell_the_robot.rrd")
    else:
        rr.init("tell-the-robot", spawn=True)
    room = rrb.Spatial3DView(name="what the robot remembers", origin="world")
    talk = rrb.TextLogView(name="conversation", origin="talk")
    if LAYOUT == "engineer":
        right = rrb.Vertical(
            talk,
            rrb.TextDocumentView(name="memory", origin="memory"),
            rrb.TextLogView(name="skill steps", origin="steps"),
            row_shares=[2, 1, 1],
        )
    else:
        right = talk
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(room, right, column_shares=[3, 2]), collapse_panels=True))


def log_room():
    """The tables and the green mark, from layout.py. Logged once."""
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for name in TABLES:
        x0, y0, x1, y1 = box_of(name)
        rr.log(
            f"world/{name}",
            rr.Boxes3D(
                centers=[((x0 + x1) / 2, (y0 + y1) / 2, TABLE_SIZE[2] / 2)],
                half_sizes=[((x1 - x0) / 2, (y1 - y0) / 2, TABLE_SIZE[2] / 2)],
                colors=[TABLE_COLORS[name]],
                labels=[name],
                fill_mode="solid",
            ),
            static=True,
        )
    rr.log(
        "world/mark",
        rr.Points3D([(PLACE_MARK[0], PLACE_MARK[1], TABLE_SIZE[2] + 0.005)], radii=[0.05], colors=[MARK]),
        static=True,
    )


def object_boxes(remembered, held):
    """Rerun boxes for the memory: the object in the hand gets a thick outline."""
    centers, half_sizes, colors, labels, radii = [], [], [], [], []
    for o in remembered:
        kind = o["kind"]
        x, y, z = o["position"]
        width = WIDTH.get(kind, 0.06)
        centers.append((x, y, z))
        half_sizes.append((width / 2, width / 2, HEIGHT.get(kind, 0.1) / 2))
        colors.append(COLORS.get(o["color"], (200, 200, 200)))
        hand = " (in hand)" if o["id"] == held else ""
        labels.append(f"#{o['id']} {o['color']} {kind}{hand}")
        radii.append(0.008 if o["id"] == held else 0.002)
    return rr.Boxes3D(centers=centers, half_sizes=half_sizes, colors=colors, labels=labels, radii=radii)


def memory_text(remembered):
    lines = [
        f"#{o['id']:<2} {o['color']:<7} {o['kind']:<7} {o.get('table') or 'off a table':<11} "
        f"{'in hand' if o['in_hand'] else (o['can_pick'] or 'can pick')}"
        for o in remembered
    ]
    return "\n".join(lines) or "nothing seen yet"


start_rerun()
log_room()
node = Node()

time, last_drawn, trail = 0.0, -1e9, []
remembered, held = [], -1

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    name = event["id"]
    if name == "base_pose":
        time, position, heading = messages.unpack_pose(event["value"].to_numpy())
        if time - last_drawn < ROBOT_EVERY:
            continue
        last_drawn = time
        rr.set_time("sim_time", duration=time)
        trail.append((position[0], position[1], 0.01))
        del trail[:-TRAIL_POINTS]
        ahead = 0.3 * np.array([np.cos(heading), np.sin(heading), 0.0])
        rr.log("world/robot", rr.Arrows3D(origins=[(position[0], position[1], 0.01)], vectors=[ahead], colors=[ROBOT]))
        rr.log("world/robot_path", rr.LineStrips3D([trail], colors=[ROBOT], radii=[0.005]))
        continue
    rr.set_time("sim_time", duration=time)
    text = event["value"][0].as_py()
    if name == "objects":
        remembered = json.loads(text)
        rr.log("world/memory", object_boxes(remembered, held))
        rr.log("memory", rr.TextDocument(memory_text(remembered)))
    elif name == "held":
        held = int(text)
        if remembered:
            rr.log("world/memory", object_boxes(remembered, held))
    elif name == "status":
        rr.log("steps", rr.TextLog(text, level="DEBUG"))
    elif name == "log":
        rr.log("talk", rr.TextLog(text, level="INFO"))
