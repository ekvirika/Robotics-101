import pyarrow as pa
from dora import Node

node = Node()
pending: set[str] = set()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "tick":
        request_id = node.send_service_request("request", pa.array(["width"]))
        pending.add(request_id)
        print(
            f"planner  -> asks gripper width   request_id={request_id[-8:]}", flush=True
        )
    if event["id"] == "response":
        request_id = event["metadata"].get("request_id", "")
        if request_id in pending:
            pending.discard(request_id)
            print(
                f"planner  <- width {event['value'][0].as_py():.1f} mm      "
                f"request_id={request_id[-8:]}",
                flush=True,
            )
