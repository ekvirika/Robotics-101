import pyarrow as pa
from dora import Node

from common.frames import boxes_from_arrow

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "boxes":
        labels = sorted(b["label"] for b in boxes_from_arrow(event["value"]))
        node.send_output("detections", pa.array(labels), event["metadata"])
