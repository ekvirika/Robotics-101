import time

from dora import Node

node = Node()
previous = None
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "reading":
        value = event["value"][0].as_py()
        if previous is not None and value > previous + 1:
            print(
                f"slow_consumer: got {value}, "
                f"skipped {value - previous - 1} stale readings",
                flush=True,
            )
        previous = value
        time.sleep(0.35)
