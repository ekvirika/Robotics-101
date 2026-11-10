import math

import numpy as np

from world import beam_angles

MAX_V = 0.8
MAX_W = 2.0
SAFE_DISTANCE = 0.9


def command(
    x: float, y: float, theta: float, goal: tuple[float, float], ranges: np.ndarray
) -> tuple[float, float]:
    gx, gy = goal
    heading_to_goal = math.atan2(gy - y, gx - x)
    angles = beam_angles()
    attract = np.array(
        [math.cos(heading_to_goal - theta), math.sin(heading_to_goal - theta)]
    )
    close = ranges < SAFE_DISTANCE
    weights = np.where(close, (SAFE_DISTANCE - ranges) / SAFE_DISTANCE, 0.0) ** 2
    repel = (
        -np.array([np.sum(weights * np.cos(angles)), np.sum(weights * np.sin(angles))])
        * 1.5
    )
    direction = attract + repel
    desired = math.atan2(direction[1], direction[0])
    w = float(np.clip(2.5 * desired, -MAX_W, MAX_W))
    front = float(np.min(ranges[np.abs(angles) < math.radians(30)]))
    distance = math.hypot(gx - x, gy - y)
    v = (
        MAX_V
        * max(0.0, math.cos(desired))
        * min(1.0, distance)
        * min(1.0, max(0.0, (front - 0.3) / 0.7))
    )
    return v, w
