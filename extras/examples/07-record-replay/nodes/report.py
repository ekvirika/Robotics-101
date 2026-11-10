from dora import Node

from common.frames import boxes_from_arrow

node = Node()
count = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "boxes":
        count += 1
        if count % 20 == 0:
            labels = sorted(b["label"] for b in boxes_from_arrow(event["value"]))
            print(f"report: {count} box messages, last frame saw {labels}", flush=True)
print(f"report: done, {count} box messages in total", flush=True)
