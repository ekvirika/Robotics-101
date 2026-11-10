from dora import Node

node = Node()
count = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "pose":
        count += 1
        if count % 10 == 0:
            pose = event["value"].to_pylist()[0]
            print(
                f"turtle at x={pose['x']:.2f} y={pose['y']:.2f} "
                f"theta={pose['theta']:.2f}",
                flush=True,
            )
