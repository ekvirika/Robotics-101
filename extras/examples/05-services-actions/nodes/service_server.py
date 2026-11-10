import math

import pyarrow as pa
from dora import Node

node = Node()
t = 0.0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "tick":
        t += 0.2
    if event["id"] == "request":
        width_mm = 40.0 + 30.0 * math.sin(t)
        node.send_service_response("response", pa.array([width_mm]), event["metadata"])
