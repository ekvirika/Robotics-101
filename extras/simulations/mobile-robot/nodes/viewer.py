import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rerun as rr
from dora import Node

from common.viz import start_rerun
from world import ARENA_CENTRE, ARENA_HALF, OBSTACLES, ROBOT_RADIUS, beam_angles

node = Node()
enabled = start_rerun("mobile-robot")
if enabled:
    rr.log(
        "world/obstacles",
        rr.Points2D(OBSTACLES[:, :2], radii=OBSTACLES[:, 2], colors=[120, 120, 120]),
        static=True,
    )
    cx, cy = ARENA_CENTRE
    corners = [
        [cx - ARENA_HALF, cy - ARENA_HALF],
        [cx + ARENA_HALF, cy - ARENA_HALF],
        [cx + ARENA_HALF, cy + ARENA_HALF],
        [cx - ARENA_HALF, cy + ARENA_HALF],
        [cx - ARENA_HALF, cy - ARENA_HALF],
    ]
    rr.log("world/walls", rr.LineStrips2D([corners], colors=[60, 60, 60]), static=True)
trail = []
pose = None
step_count = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT" or not enabled:
        continue
    if event["id"] == "pose":
        pose = event["value"].to_numpy()
        step_count += 1
        rr.set_time("step", sequence=step_count)
        trail.append(pose[:2].tolist())
        heading = pose[:2] + 0.4 * np.array([np.cos(pose[2]), np.sin(pose[2])])
        rr.log(
            "world/robot",
            rr.Points2D([pose[:2]], radii=ROBOT_RADIUS, colors=[40, 110, 220]),
        )
        rr.log(
            "world/robot/heading",
            rr.LineStrips2D([[pose[:2], heading]], colors=[40, 110, 220]),
        )
        if step_count % 5 == 0:
            rr.log(
                "world/trail", rr.LineStrips2D([trail[-2000:]], colors=[150, 180, 240])
            )
    if event["id"] == "scan" and pose is not None:
        ranges = event["value"].to_numpy()
        angles = pose[2] + beam_angles()
        ends = (
            pose[:2]
            + np.stack([np.cos(angles), np.sin(angles)], axis=1) * ranges[:, None]
        )
        rr.log("world/lidar", rr.Points2D(ends, radii=0.04, colors=[230, 90, 40]))
    if event["id"] == "goal":
        rr.log(
            "world/goal",
            rr.Points2D([event["value"].to_numpy()], radii=0.2, colors=[40, 180, 60]),
        )
