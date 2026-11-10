"""Arms and hands: smooth arm moves, gravity compensation, and the grip.

Two kinds of moves, like industrial robot arms (Universal Robots calls them
movej and movel):
  joint move  every joint goes straight to its new angle; the hand's path is a
              curve. Good for big moves through free space.
  line move   the hand goes in a straight line (IK at every step). Good near
              objects, where the path matters.
Both follow a speed profile: speed up, cruise, slow down, stop on the target
(the same idea as lesson 1's trajectory.py).

Only the right arm moves; the left arm holds its walking pose.
"""

import mujoco
import numpy as np

from g1 import messages, robot
from g1.arm_ik import ArmIK

HAND_SPEED = 0.15  # m/s, line moves
HAND_ACCEL = 0.3  # m/s^2
JOINT_SPEED = 1.0  # rad/s, joint moves (the joint that moves most sets the pace)
JOINT_ACCEL = 2.0  # rad/s^2
GRIP_TIME = 1.0  # s to close or open the hand fully


def upright_hand(yaw):
    """Hand orientation for grasping an upright bottle from the side.

    Fingers point horizontally in direction `yaw` (0 = straight ahead),
    the palm faces left (the right hand's palm side), knuckles up/down.
    Returned as a 3x3 rotation matrix (columns = the hand's x, y, z axes).
    """
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Trapezoid:
    """Speed profile for a move of known length: speed up, cruise, slow down.

    If the move is too short to reach full speed, it speeds up and slows down
    right away (a triangle instead of a trapezoid).
    """

    def __init__(self, distance, max_speed, max_accel):
        self.distance = distance
        self.accel = max_accel
        self.peak_speed = min(max_speed, np.sqrt(distance * max_accel))
        self.ramp_time = (
            self.peak_speed / max_accel
        )  # time to speed up (and to slow down)
        ramp_distance = 0.5 * max_accel * self.ramp_time**2
        cruise_time = (
            (distance - 2 * ramp_distance) / self.peak_speed if distance > 0 else 0.0
        )
        self.duration = 2 * self.ramp_time + cruise_time

    def progress(self, t):
        """Fraction of the distance covered after t seconds (0 to 1)."""
        if self.distance <= 0 or t >= self.duration:
            return 1.0
        if t <= 0:
            return 0.0
        if t < self.ramp_time:  # speeding up: s = a t^2 / 2
            s = 0.5 * self.accel * t**2
        elif t < self.duration - self.ramp_time:  # cruising at constant speed
            s = 0.5 * self.accel * self.ramp_time**2 + self.peak_speed * (
                t - self.ramp_time
            )
        else:  # slowing down: the mirror image of speeding up
            s = self.distance - 0.5 * self.accel * (self.duration - t) ** 2
        return s / self.distance


class ArmController:
    def __init__(self, model_path):
        self.ik = ArmIK(model_path, "right")
        # a second copy of the robot, to compute gravity torques (pelvis fixed)
        self.gravity_data = mujoco.MjData(self.ik.model)
        self.gravity_data.qpos[0:7] = [0, 0, 0, 1, 0, 0, 0]
        upper = [
            self.ik.model.joint(name).id for name in robot.JOINTS[robot.UPPER_BODY]
        ]
        self.upper_qpos = self.ik.model.jnt_qposadr[upper]
        self.upper_dof = self.ik.model.jnt_dofadr[upper]

        self.arm_q = robot.RIGHT_ARM_WALK.copy()  # what we command the right arm to do
        self.move = None  # the move in progress, or None
        self.grip = 0.0  # 0 = open, 1 = closed
        self.grip_goal = 0.0
        self.last_time = None

    # ---- commands ----

    def move_joints(self, time, arm_q):
        """Joint move of the right arm to the angles arm_q."""
        start = self.arm_q.copy()
        goal = np.array(arm_q, dtype=float)
        profile = Trapezoid(np.max(np.abs(goal - start)), JOINT_SPEED, JOINT_ACCEL)
        self.move = ("joints", time, profile, start, goal)

    def move_hand(self, time, position, rotation):
        """Line move: the right hand's grasp point goes straight to `position`.

        The hand turns to `rotation` at once (start line moves from a pose
        that already has it). Returns False, and doesn't move, if the arm
        can't reach the goal.
        """
        _, reached = self.ik.solve(position, rotation, self.arm_q)
        if not reached:
            return False
        start, _ = self.ik.hand_pose(self.arm_q)
        goal = np.array(position, dtype=float)
        profile = Trapezoid(np.linalg.norm(goal - start), HAND_SPEED, HAND_ACCEL)
        self.move = ("line", time, profile, start, goal, rotation)
        return True

    def solve(self, position, rotation):
        """Arm angles for a hand pose (for joint moves). None if out of reach."""
        arm_q, reached = self.ik.solve(position, rotation, self.arm_q)
        return arm_q if reached else None

    def set_grip(self, closed):
        self.grip_goal = 1.0 if closed else 0.0

    def busy(self):
        return self.move is not None or self.grip != self.grip_goal

    def execute(self, time, kind, position, yaw):
        """Start one command from the task (kinds in messages.py).

        Returns False if the hand can't reach the target (nothing moves then).
        """
        rotation = upright_hand(yaw)
        if kind == messages.REST:
            self.move_joints(time, robot.RIGHT_ARM_WALK)
        elif kind == messages.MOVE_JOINTS:
            arm_q = self.solve(position, rotation)
            if arm_q is None:
                return False
            self.move_joints(time, arm_q)
        elif kind == messages.MOVE_LINE:
            return self.move_hand(time, position, rotation)
        elif kind == messages.OPEN:
            self.set_grip(False)
        elif kind == messages.CLOSE:
            self.set_grip(True)
        return True

    # ---- the control loop ----

    def update(self, time, upper_q):
        """Call every 20 ms. upper_q: measured angles of the 28 arm + hand joints.

        Returns (targets, kp, kd, tau) for the 28 joints. tau is gravity
        compensation: the torque that holds each joint against the weight of
        what hangs on it (lesson 1's weight torque, now computed by MuJoCo
        for 7 joints at once).
        """
        dt = 0.0 if self.last_time is None else time - self.last_time
        self.last_time = time
        if self.move is not None:
            kind, start_time, profile = self.move[:3]
            s = profile.progress(time - start_time)
            if kind == "joints":
                start, goal = self.move[3:]
                self.arm_q = start + s * (goal - start)
            else:
                start, goal, rotation = self.move[3:]
                self.arm_q, _ = self.ik.solve(
                    start + s * (goal - start), rotation, self.arm_q
                )
            if s >= 1.0:
                self.move = None
        step = dt / GRIP_TIME  # close or open a little more each update
        if self.grip < self.grip_goal:
            self.grip = min(self.grip_goal, self.grip + step)
        elif self.grip > self.grip_goal:
            self.grip = max(self.grip_goal, self.grip - step)

        right_hand = robot.RIGHT_HAND_OPEN + self.grip * (
            robot.RIGHT_HAND_CLOSED - robot.RIGHT_HAND_OPEN
        )
        targets = np.concatenate(
            [robot.LEFT_ARM_WALK, robot.LEFT_HAND_OPEN, self.arm_q, right_hand]
        )
        kp = np.concatenate([robot.ARM_KP, robot.HAND_KP, robot.ARM_KP, robot.HAND_KP])
        kd = np.concatenate([robot.ARM_KD, robot.HAND_KD, robot.ARM_KD, robot.HAND_KD])

        # gravity torques at the measured angles, with the robot standing still
        self.gravity_data.qpos[self.upper_qpos] = upper_q
        mujoco.mj_forward(self.ik.model, self.gravity_data)
        tau = self.gravity_data.qfrc_bias[self.upper_dof].copy()
        return targets, kp, kd, tau
