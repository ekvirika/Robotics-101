import random

import pyarrow as pa
from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        node.send_output("value", pa.array([random.randint(0, 30)]))
