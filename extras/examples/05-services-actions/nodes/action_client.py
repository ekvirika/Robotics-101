import uuid

import pyarrow as pa
from dora import Node

GOALS = [("shelf 4", 3.0, False), ("dock", 6.0, True)]

node = Node()
active = None
next_goal = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "tick" and active is None and next_goal < len(GOALS):
        name, distance, cancel_halfway = GOALS[next_goal]
        next_goal += 1
        active = {
            "id": str(uuid.uuid4()),
            "name": name,
            "distance": distance,
            "cancel": cancel_halfway,
            "sent_cancel": False,
        }
        node.send_output(
            "goal", pa.array([distance]), {"goal_id": active["id"], "name": name}
        )
        print(f"mission  -> goal: drive to {name}, {distance:.1f} m", flush=True)
    if (
        event["id"] == "feedback"
        and active
        and event["metadata"].get("goal_id") == active["id"]
    ):
        left = event["value"][0].as_py()
        print(
            f"mission  <- feedback: {left:.2f} m left to {active['name']}", flush=True
        )
        if (
            active["cancel"]
            and not active["sent_cancel"]
            and left < active["distance"] / 2
        ):
            node.send_output("cancel", pa.array([True]), {"goal_id": active["id"]})
            active["sent_cancel"] = True
            print(f"mission  -> cancel {active['name']}", flush=True)
    if (
        event["id"] == "result"
        and active
        and event["metadata"].get("goal_id") == active["id"]
    ):
        print(
            f"mission  <- result: {active['name']} "
            f"{event['metadata'].get('goal_status')}",
            flush=True,
        )
        active = None
