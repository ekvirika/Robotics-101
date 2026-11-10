from dora import Node

node = Node()
last_start = None
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT_CLOSED":
        print(f"monitor: input {event['id']} closed, no reading for 2 s", flush=True)
    if event["type"] == "INPUT" and event["id"] == "reading":
        started = event["metadata"].get("started")
        if started != last_start:
            print(
                f"monitor: readings from sensor instance started {started}", flush=True
            )
            last_start = started
