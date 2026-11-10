import os
import socket

import pyarrow as pa
from dora import Node

node = Node()
where = os.environ.get("DORA_MACHINE_ID") or socket.gethostname()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "value":
        value = event["value"][0].as_py()
        node.send_output(
            "result",
            pa.array([value * value]),
            {"computed_on": where, "pid": os.getpid()},
        )
