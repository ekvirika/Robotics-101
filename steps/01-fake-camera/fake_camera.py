"""Fake camera: sends the same messages as the official webcam.py, from a photo.

It slowly pans across a test photo that ships with ultralytics (a street with a bus
and people), so YOLO has real objects to find. No webcam needed.
"""

import os
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import ultralytics
from dora import Node

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
PHOTO = os.getenv("PHOTO", str(Path(ultralytics.__file__).parent / "assets" / "bus.jpg"))


def load_photo() -> np.ndarray:
    photo = cv2.imread(PHOTO)
    if photo is None:
        raise FileNotFoundError(f"cannot read {PHOTO}")
    scale = max(CAMERA_WIDTH * 1.4 / photo.shape[1], CAMERA_HEIGHT * 1.2 / photo.shape[0])
    return cv2.resize(photo, None, fx=scale, fy=scale)


def frame_at(photo: np.ndarray, step: int) -> np.ndarray:
    max_x = photo.shape[1] - CAMERA_WIDTH
    max_y = photo.shape[0] - CAMERA_HEIGHT
    x = int(max_x * (0.5 + 0.5 * np.sin(step / 40)))
    y = int(max_y * (0.5 + 0.5 * np.sin(step / 65)))
    return np.ascontiguousarray(photo[y : y + CAMERA_HEIGHT, x : x + CAMERA_WIDTH])


def main():
    node = Node()
    photo = load_photo()
    step = 0
    for event in node:
        if event["type"] == "INPUT":
            frame = frame_at(photo, step)
            step += 1
            node.send_output(
                "image",
                pa.array(frame.ravel()),
                metadata={
                    "width": CAMERA_WIDTH,
                    "height": CAMERA_HEIGHT,
                    "encoding": "bgr8",
                },
            )
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
