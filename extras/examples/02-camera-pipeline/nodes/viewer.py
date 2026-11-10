import logging

import rerun as rr
from dora import Node

from common.frames import boxes_from_arrow, decode_image
from common.viz import start_rerun

node = Node()
enabled = start_rerun("camera-pipeline")
frames = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if event["id"] == "image":
        frames += 1
        if enabled:
            rr.set_time("frame", sequence=frames)
            rr.log(
                "camera/image",
                rr.Image(decode_image(event["value"], event["metadata"])),
            )
    if event["id"] == "boxes":
        boxes = boxes_from_arrow(event["value"])
        if enabled:
            rr.log(
                "camera/image/boxes",
                rr.Boxes2D(
                    array=[[b["x1"], b["y1"], b["x2"], b["y2"]] for b in boxes],
                    array_format=rr.Box2DFormat.XYXY,
                    labels=[f"{b['label']} {b['confidence']:.2f}" for b in boxes],
                ),
            )
        if frames % 30 == 0:
            logging.info(
                "frame %d, %d boxes: %s",
                frames,
                len(boxes),
                ", ".join(b["label"] for b in boxes),
            )
