import os

from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT":
        meta = event["metadata"]
        print(
            f"display (pid {os.getpid()}): {event['value'][0].as_py():>4}  "
            f"computed by pid {meta.get('pid')}",
            flush=True,
        )
