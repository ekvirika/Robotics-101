import math

import pyarrow as pa
from dora import Node

node = Node()
t = 0.0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        t += 0.05
        radius = 0.35 + 0.25 * math.sin(0.3 * t)
        node.send_output(
            "target",
            pa.array(
                [radius * math.cos(0.8 * t) + 0.1, radius * math.sin(0.8 * t) + 0.3]
            ),
        )
