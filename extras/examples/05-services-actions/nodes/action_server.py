import pyarrow as pa
from dora import Node

SPEED_M_PER_TICK = 0.25

node = Node()
goal = None
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "goal":
        goal = {"id": event["metadata"]["goal_id"], "left": event["value"][0].as_py()}
    if (
        event["id"] == "cancel"
        and goal
        and event["metadata"].get("goal_id") == goal["id"]
    ):
        node.send_output(
            "result",
            pa.array([goal["left"]]),
            {"goal_id": goal["id"], "goal_status": "canceled"},
        )
        goal = None
    if event["id"] == "tick" and goal:
        goal["left"] = max(0.0, goal["left"] - SPEED_M_PER_TICK)
        node.send_output("feedback", pa.array([goal["left"]]), {"goal_id": goal["id"]})
        if goal["left"] == 0.0:
            node.send_output(
                "result",
                pa.array([0.0]),
                {"goal_id": goal["id"], "goal_status": "succeeded"},
            )
            goal = None
