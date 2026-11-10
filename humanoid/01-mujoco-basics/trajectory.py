"""Smooth motion: speed up, cruise, slow down, stop exactly on the goal.

Real robots don't send the final goal straight to the motor controller.
First a trajectory is planned: where the arm should be at every moment.
The controller then follows it.
"""

import math

MAX_SPEED = 1.5  # rad/s (86 deg/s): top speed of the joint
MAX_ACCEL = 3.0  # rad/s^2: how fast it may speed up or slow down


class Trajectory:
    def __init__(self, start_angle):
        self.pos = start_angle  # where the arm should be now (rad)
        self.vel = 0.0  # how fast it should move now (rad/s)

    def step(self, goal, dt):
        distance = goal - self.pos
        # distance still left after this step (can't be less than 0)
        left = max(0.0, abs(distance) - abs(self.vel) * dt)

        # fastest allowed speed: top speed, but slow enough to brake in time
        speed = min(MAX_SPEED, math.sqrt(2 * MAX_ACCEL * left))
        if distance < 0:
            speed = -speed  # goal is behind: move the other way

        # change speed gradually, no faster than MAX_ACCEL
        max_change = MAX_ACCEL * dt
        if speed > self.vel + max_change:
            speed = self.vel + max_change
        if speed < self.vel - max_change:
            speed = self.vel - max_change

        accel = (speed - self.vel) / dt
        self.vel = speed
        self.pos = self.pos + self.vel * dt

        # arrived: stop exactly on the goal
        if abs(goal - self.pos) < 0.0001:
            self.pos = goal
            self.vel = 0.0

        return self.pos, self.vel, accel
