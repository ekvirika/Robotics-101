import pyarrow as pa
from dora import Node

node = Node()
count = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        count += 1
        node.send_output("count", pa.array([count], type=pa.uint64()))
