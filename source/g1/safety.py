"""Speed and separation monitoring: never walk faster than the robot can stop.

The planner chooses where to go; this layer, between the planner and the
robot, only ever slows the command down or stops it. It is the same split as
ROS 2 Nav2's controller and its Collision Monitor, and the rule is the speed
and separation monitoring of ISO/TS 15066 (collaborative robots):

    the free space to a person must stay at least
    S = v_h (T_r + T_s)  +  v_r T_r  +  B  +  Z

    v_h  the person's speed toward the robot (measured by the tracker)
    v_r  the robot's speed toward the person
    T_r  the robot's reaction time: from a lidar scan to a new command
    T_s  the robot's stopping time,    T_s = v_r / a
    B    the robot's braking distance, B = v_r^2 / (2 a)
    Z    measurement uncertainty

For each person (and each raw lidar point, which may be something the tracker
has not recognised, counted as standing still) it solves this for the
largest v_r it allows, and scales the whole command down until the robot's
speed toward everyone is within that. Plain Python + NumPy.
"""

import math

import numpy as np

from .planner import MAX_DECEL, PERSON_RADIUS, PLAN_DT, ROBOT_RADIUS

# The robot reacts to what it saw up to one lidar turn ago (10 Hz lidar) and
# plans every PLAN_DT: the worst case from seeing to acting is both.
REACTION_TIME = 0.1 + PLAN_DT  # s
# Where people are is known to about 3 cm (the tracker, measured in the
# lesson's tests) and lidar points to about 1 cm; keep 2 standard deviations.
UNCERTAINTY = 0.06  # m
STOP_DISTANCE = 0.05  # m of free space: closer than this, nothing moves toward it


def allowed_speed(gap, human_speed, decel=MAX_DECEL, reaction=REACTION_TIME):
    """The largest robot speed toward something `gap` meters of free space away
    that comes toward the robot at `human_speed`, from
    gap = v_h (T_r + v/a) + v T_r + v^2 / (2a), solved for v."""
    room = gap - human_speed * reaction
    if room <= 0.0:
        return 0.0
    b = reaction + human_speed / decel
    return decel * (-b + math.sqrt(b * b + 2.0 * room / decel))


class SpeedSeparationMonitor:
    def __init__(self, decel=MAX_DECEL):
        """decel: how fast the robot can brake (m/s^2), the a in the formula."""
        self.decel = decel
        self.limited = False  # did the last command get slowed down?

    def limit(self, cmd, pose, tracks, points=None):
        """The command, slowed down if it isn't safe.

        cmd: (forward, left, turn) in the robot frame. pose: (x, y, heading).
        tracks: rows (x, y, vx, vy), world frame. points: optional raw lidar
        points (x, y), world frame. Turning on the spot is always allowed.
        """
        forward, left, turn = cmd
        x, y, heading = pose
        c, s = math.cos(heading), math.sin(heading)
        velocity = np.array([c * forward - s * left, s * forward + c * left])  # world frame
        speed = float(np.hypot(*velocity))
        self.limited = False
        if speed < 1e-6:
            return (0.0, 0.0, turn)

        obstacles = [(row[:2], row[2:4], PERSON_RADIUS) for row in np.asarray(tracks).reshape(-1, 4)]
        if points is not None:
            obstacles += [(p, np.zeros(2), 0.0) for p in np.asarray(points).reshape(-1, 2)]

        scale = 1.0
        for position, person_velocity, radius in obstacles:
            offset = np.asarray(position) - (x, y)
            distance = float(np.hypot(*offset))
            if distance < 1e-6:
                return (0.0, 0.0, turn)
            toward = offset / distance  # unit vector from robot to obstacle
            robot_closing = float(velocity @ toward)
            if robot_closing <= 0.0:
                continue  # walking away from it: always fine
            human_closing = max(0.0, -float(np.asarray(person_velocity) @ toward))
            gap = distance - ROBOT_RADIUS - radius - UNCERTAINTY
            allowed = 0.0 if gap < STOP_DISTANCE else allowed_speed(gap, human_closing, self.decel)
            if robot_closing > allowed:
                scale = min(scale, allowed / robot_closing)
        if scale < 1.0:
            self.limited = True
        return (forward * scale, left * scale, turn)
