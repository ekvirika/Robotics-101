import pyarrow as pa
from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "count":
        value = event["value"][0].as_py()
        node.send_output("doubled", pa.array([value * 2]))
