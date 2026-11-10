import os

import pyarrow as pa
from dora import Node

VERSION = os.environ.get("FILTER_VERSION", "v1")
THRESHOLD = int(os.environ.get("THRESHOLD", "10"))

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "value":
        value = event["value"][0].as_py()
        if value >= THRESHOLD:
            node.send_output(
                "value", pa.array([value]), {"filter": f"{VERSION} (>= {THRESHOLD})"}
            )
