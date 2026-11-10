from dora import Node

node = Node()
print("logger: added to the running graph", flush=True)
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT":
        print(f"logger: raw sensor value {event['value'][0].as_py()}", flush=True)
