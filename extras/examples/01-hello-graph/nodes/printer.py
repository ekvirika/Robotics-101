from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT":
        print(f"{event['id']:>8}: {event['value'][0].as_py()}", flush=True)
