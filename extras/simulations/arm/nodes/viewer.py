import logging
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rerun as rr
from dora import Node

from common.viz import start_rerun
from kinematics import forward

node = Node()
enabled = start_rerun("arm")
target = None
errors = []
step_count = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "target":
        target = event["value"].to_pylist()
        if enabled:
            rr.log(
                "arm/target", rr.Points2D([target], radii=0.03, colors=[40, 180, 60])
            )
    if event["id"] == "joint_state":
        q1, q2, dq1, dq2 = event["value"].to_pylist()
        elbow, hand = forward(q1, q2)
        step_count += 1
        if target is not None:
            errors.append(math.dist(hand, target))
        if enabled:
            rr.set_time("step", sequence=step_count)
            rr.log(
                "arm/links",
                rr.LineStrips2D(
                    [[(0, 0), elbow, hand]], radii=0.02, colors=[40, 110, 220]
                ),
            )
            rr.log(
                "arm/joints",
                rr.Points2D([(0, 0), elbow, hand], radii=0.035, colors=[20, 20, 20]),
            )
            rr.log("plots/q1", rr.Scalars(q1))
            rr.log("plots/q2", rr.Scalars(q2))
        if step_count % 200 == 0 and errors:
            recent = errors[-200:]
            logging.info(
                "tracking error, last 2 s: mean %.1f cm, max %.1f cm",
                100 * sum(recent) / len(recent),
                100 * max(recent),
            )
