import pyarrow as pa
from dora import Node

TWIST = pa.struct(
    [
        (
            "linear",
            pa.struct([("x", pa.float64()), ("y", pa.float64()), ("z", pa.float64())]),
        ),
        (
            "angular",
            pa.struct([("x", pa.float64()), ("y", pa.float64()), ("z", pa.float64())]),
        ),
    ]
)

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        twist = {
            "linear": {"x": 1.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 1.0},
        }
        node.send_output("cmd_vel", pa.array([twist], type=TWIST))
