import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pyarrow as pa
from dora import Node

from kinematics import inverse

ELBOW_UP = os.environ.get("ELBOW", "up") == "up"

node = Node()
unreachable = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "target":
        x, y = event["value"].to_pylist()
        solution = inverse(x, y, ELBOW_UP)
        if solution is None:
            unreachable += 1
            if unreachable % 20 == 1:
                logging.warning(
                    "ik: target (%.2f, %.2f) out of reach, holding last joint targets",
                    x,
                    y,
                )
            continue
        node.send_output("joint_targets", pa.array(list(solution)))
