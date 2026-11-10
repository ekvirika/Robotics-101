import os

from dora import Node

from common.frames import encode_image
from common.synthetic import render_scene

WIDTH = int(os.environ.get("IMAGE_WIDTH", "640"))
HEIGHT = int(os.environ.get("IMAGE_HEIGHT", "480"))
SOURCE = os.environ.get("CAMERA_SOURCE", "synthetic")


def open_webcam():
    import cv2

    capture = cv2.VideoCapture(int(os.environ.get("CAMERA_INDEX", "0")))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not capture.isOpened():
        raise RuntimeError("webcam not available, set CAMERA_SOURCE: synthetic")
    return cv2, capture


def read_webcam(cv2, capture):
    ok, frame = capture.read()
    if not ok:
        return None
    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


node = Node()
webcam = open_webcam() if SOURCE == "webcam" else None
frame_index = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT" or event["id"] != "tick":
        continue
    image = read_webcam(*webcam) if webcam else render_scene(frame_index, WIDTH, HEIGHT)
    frame_index += 1
    if image is None:
        continue
    value, metadata = encode_image(image)
    node.send_output("image", value, metadata)
