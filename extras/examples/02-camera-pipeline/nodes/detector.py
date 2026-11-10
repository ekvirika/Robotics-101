import os

import numpy as np
from dora import Node

from common.frames import boxes_to_arrow, decode_image

MIN_CONFIDENCE = float(os.environ.get("MIN_CONFIDENCE", "0.4"))
COLOURS = {
    "red": lambda r, g, b: (r > 150) & (g < 100) & (b < 100),
    "green": lambda r, g, b: (g > 140) & (r < 100) & (b < 110),
    "blue": lambda r, g, b: (b > 150) & (r < 100) & (g < 140),
}


def detect_colours(image: np.ndarray) -> list[dict]:
    r, g, b = (image[..., i].astype(np.int16) for i in range(3))
    boxes = []
    for label, test in COLOURS.items():
        ys, xs = np.nonzero(test(r, g, b))
        if xs.size < 50:
            continue
        boxes.append(
            {
                "label": label,
                "confidence": 1.0,
                "x1": float(xs.min()),
                "y1": float(ys.min()),
                "x2": float(xs.max()),
                "y2": float(ys.max()),
            }
        )
    return boxes


def make_yolo():
    from ultralytics import YOLO

    model = YOLO(os.environ.get("YOLO_MODEL", "yolo11n.pt"))

    def detect(image: np.ndarray) -> list[dict]:
        result = model.predict(image, conf=MIN_CONFIDENCE, verbose=False)[0]
        return [
            {
                "label": result.names[int(c)],
                "confidence": float(p),
                "x1": float(x1),
                "y1": float(y1),
                "x2": float(x2),
                "y2": float(y2),
            }
            for (x1, y1, x2, y2), p, c in zip(
                result.boxes.xyxy.tolist(),
                result.boxes.conf.tolist(),
                result.boxes.cls.tolist(),
            )
        ]

    return detect


detect = (
    make_yolo() if os.environ.get("DETECTOR", "colour") == "yolo" else detect_colours
)
node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "image":
        image = decode_image(event["value"], event["metadata"])
        boxes = detect(image)
        node.send_output("boxes", boxes_to_arrow(boxes), {"count": len(boxes)})
