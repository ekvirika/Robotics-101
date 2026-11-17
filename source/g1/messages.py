"""The messages between the dora nodes: flat lists of float64 with a fixed
layout. pack_* builds one, unpack_* splits it into named parts.

  scan          sim -> tracker, mission       one turn of the lidar                    (10 Hz)
  base_pose     sim -> everyone               where the robot is: time, x, y, heading  (50 Hz)
  robot_state   sim -> mission                time, pelvis height, speed, upper-body
                                              and hand joint angles                    (50 Hz)
  object_pose   sim -> mission                where an object is: time, x, y, z, heading
  tracks        tracker -> planner, safety    the people the robot knows about          (10 Hz)
  cmd_vel       -> sim                        walking command: forward, left, turn
  upper_body    mission -> sim                17 waist + arm joint targets (g1.joints.UPPER_BODY)
  hands         mission -> sim                14 finger joint targets (g1.joints.HANDS)
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


# ---- base_pose: [time, x, y, heading] (world frame, meters and radians) ----
# On the real robot this comes from localization (lidar + map).


def pack_pose(time, x, y, heading):
    return np.array([time, x, y, heading], dtype=float)


def unpack_pose(msg):
    return msg[0], np.asarray(msg[1:3]), msg[3]


# ---- cmd_vel: [forward m/s, left m/s, turn rad/s] (robot frame) ----


def pack_cmd_vel(cmd):
    return np.asarray(cmd, dtype=float)


def unpack_cmd_vel(msg):
    return np.asarray(msg, dtype=float)


# ---- robot_state: [time, pelvis height, speed, upper body x17, hands x14] ----

UPPER = 17
HANDS = 14


def pack_robot_state(time, pelvis_height, speed, upper_body, hands):
    return np.concatenate([[time, pelvis_height, speed], upper_body, hands])


def unpack_robot_state(msg):
    msg = np.asarray(msg)
    return msg[0], msg[1], msg[2], msg[3 : 3 + UPPER], msg[3 + UPPER : 3 + UPPER + HANDS]


# ---- object_pose: [time, x, y, z, heading] (world frame) ----


def pack_object_pose(time, position, heading):
    return np.concatenate([[time], position, [heading]])


def unpack_object_pose(msg):
    msg = np.asarray(msg)
    return msg[0], msg[1:4], msg[4]
