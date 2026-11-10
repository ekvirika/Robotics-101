import math
from dataclasses import dataclass

import numpy as np

OBSTACLES = np.array(
    [
        [2.0, 1.0, 0.5],
        [4.0, -1.0, 0.6],
        [5.5, 2.0, 0.5],
        [3.0, 3.5, 0.4],
        [6.5, -2.5, 0.5],
        [1.5, -2.5, 0.4],
    ]
)
ARENA_HALF = 5.0
ARENA_CENTRE = (4.0, 0.0)
ROBOT_RADIUS = 0.25
LIDAR_BEAMS = 36
LIDAR_RANGE = 4.0


@dataclass
class Pose:
    x: float
    y: float
    theta: float


def step(pose: Pose, v: float, w: float, dt: float) -> Pose:
    theta = pose.theta + w * dt
    return Pose(
        pose.x + v * math.cos(theta) * dt,
        pose.y + v * math.sin(theta) * dt,
        (theta + math.pi) % (2 * math.pi) - math.pi,
    )


def collides(pose: Pose) -> bool:
    d = np.hypot(OBSTACLES[:, 0] - pose.x, OBSTACLES[:, 1] - pose.y)
    if np.any(d < OBSTACLES[:, 2] + ROBOT_RADIUS):
        return True
    cx, cy = ARENA_CENTRE
    return (
        abs(pose.x - cx) > ARENA_HALF - ROBOT_RADIUS
        or abs(pose.y - cy) > ARENA_HALF - ROBOT_RADIUS
    )


def lidar(pose: Pose) -> np.ndarray:
    angles = pose.theta + np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)
    ranges = np.full(LIDAR_BEAMS, LIDAR_RANGE)
    dx, dy = np.cos(angles), np.sin(angles)
    for ox, oy, r in OBSTACLES:
        fx, fy = pose.x - ox, pose.y - oy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        hit = disc >= 0
        t = -b - np.sqrt(np.where(hit, disc, 0))
        valid = hit & (t > 0)
        ranges = np.where(valid, np.minimum(ranges, t), ranges)
    cx, cy = ARENA_CENTRE
    with np.errstate(divide="ignore", invalid="ignore"):
        for wall, d, origin in (
            (cx + ARENA_HALF, dx, pose.x),
            (cx - ARENA_HALF, dx, pose.x),
            (cy + ARENA_HALF, dy, pose.y),
            (cy - ARENA_HALF, dy, pose.y),
        ):
            t = (wall - origin) / d
            ranges = np.where((t > 0) & np.isfinite(t), np.minimum(ranges, t), ranges)
    return ranges


def beam_angles() -> np.ndarray:
    return np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)
