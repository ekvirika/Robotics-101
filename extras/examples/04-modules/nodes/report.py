from dora import Node

node = Node()
seen = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "detections":
        seen += 1
        if seen % 15 == 0:
            print(f"detections #{seen}: {event['value'].to_pylist()}", flush=True)
