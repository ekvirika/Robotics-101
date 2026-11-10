from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT":
        print(
            f"display: {event['value'][0].as_py():>3}  "
            f"passed filter {event['metadata'].get('filter')}",
            flush=True,
        )
