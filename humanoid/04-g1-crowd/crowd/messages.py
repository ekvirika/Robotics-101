"""The two new messages of this lesson (the others are in g1/messages.py).

scan    sim -> tracker       one turn of the lidar                  (10 Hz)
tracks  tracker -> planner   the people the robot knows about now   (10 Hz)
"""

import numpy as np

# ---- scan: [time, range x N] ----
# N distances in meters, one per ray. Ray k points at angle k * (360 / N)
# degrees, counted to the left from the robot's forward direction.
# A ray that hits nothing within the lidar's range is infinity (as in ROS).


def pack_scan(time, ranges):
    return np.concatenate([[time], ranges])


def unpack_scan(msg):
    return msg[0], np.asarray(msg[1:])


# ---- tracks: [time, then x, y, vx, vy for each person] (world frame) ----


def pack_tracks(time, tracks):
    """tracks: array of shape (people, 4)."""
    return np.concatenate([[time], np.asarray(tracks, dtype=float).ravel()])


def unpack_tracks(msg):
    return msg[0], np.asarray(msg[1:]).reshape(-1, 4)
