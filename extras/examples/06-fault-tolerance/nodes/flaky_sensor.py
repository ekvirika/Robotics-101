import os
import sys
import time

import pyarrow as pa
from dora import Node

CRASH_AFTER = int(os.environ.get("CRASH_AFTER", "15"))
SILENT_AT = int(os.environ.get("SILENT_AT", "0"))

node = Node()
started = time.strftime("%H:%M:%S")
print(f"sensor started at {started}", flush=True)
sent = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        sent += 1
        node.send_output("reading", pa.array([sent]), {"started": started})
        if sent == SILENT_AT:
            print("sensor going silent for 3 s without crashing", flush=True)
            time.sleep(3)
        if sent == CRASH_AFTER:
            print(f"sensor crashing after {sent} readings", flush=True)
            sys.exit(1)
