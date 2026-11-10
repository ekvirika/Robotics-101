"""Legs and waist: walk with Unitree's trained policy, or stand still and balance.

WalkPolicy runs the neural network. Locomotion picks the mode:
  walk   when a velocity is asked for (the network moves the legs),
  stand  when the velocity has been zero for a moment (fixed angles + balance).
Real humanoids switch between walking and standing the same way.
"""

import numpy as np
import onnxruntime

from g1 import robot

# ---- Unitree's policy settings ----
# From unitree_rl_gym deploy/deploy_mujoco/configs/g1.yaml. The network was
# trained with exactly these numbers; changing them breaks it.
POLICY_DT = 0.02  # s: the network runs 50 times per second
GAIT_PERIOD = 0.8  # s: one full step cycle, left + right
# the leg pose the network was trained around (hip pitch, roll, yaw, knee, ankle x2)
POLICY_LEGS = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0] * 2)
ACTION_SCALE = 0.25  # target angle = POLICY_LEGS + 0.25 * network output
CMD_SCALE = np.array([2.0, 2.0, 0.25])  # input scaling for vx, vy, turn rate
GYRO_SCALE = 0.25
JOINT_SPEED_SCALE = 0.05
MEMORY_SIZE = 64

# ---- stopping and standing ----
SETTLE_TIME = 0.8  # s: walk in place one full step cycle at zero speed before stopping
BLEND_TIME = 1.0  # s: move the legs smoothly from the last step into the standing pose
STEADY_TIME = 0.5  # s: wait after the blend before reporting "standing"
# Balance while standing ("ankle strategy", what people do): if the body leans
# forward, push down with the toes; if it leans back, lift them. Per radian of
# lean the ankle targets move 0.5 rad, and 0.1 rad per rad/s of lean speed
# (that part damps the sway).
BALANCE_KP = 0.5
BALANCE_KD = 0.1
ANKLE_PITCH = [4, 10]  # left and right ankle pitch, positions in robot.LEGS

WALK = "walk"
STAND = "stand"


def gravity_in_body(quat):
    """Direction of gravity seen from the pelvis (what the IMU feels).

    quat = pelvis orientation (w, x, y, z). Upright gives (0, 0, -1).
    Leaning forward by an angle a gives (sin a, 0, -cos a).
    It is the rotation matrix's bottom row, negated.
    """
    w, x, y, z = quat
    return np.array(
        [2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)]
    )


class WalkPolicy:
    """Unitree's walking network: robot state + wanted velocity -> leg targets."""

    def __init__(self, onnx_path):
        self.session = onnxruntime.InferenceSession(onnx_path)
        self.reset()

    def reset(self):
        """Forget the past, like a fresh start. Call before walking again."""
        self.memory_h = np.zeros((1, 1, MEMORY_SIZE), dtype=np.float32)
        self.memory_c = np.zeros((1, 1, MEMORY_SIZE), dtype=np.float32)
        self.last_action = np.zeros(12, dtype=np.float32)

    def step(self, cmd_vel, leg_q, leg_dq, quat, gyro, walk_time):
        """One network step (every 20 ms). Returns 12 leg target angles.

        The network gets 47 numbers, in the order it was trained with:
          3  turn rates of the pelvis (gyro)
          3  gravity direction (tells it how it leans)
          3  wanted velocity: forward, sideways, turn
          12 leg angles (minus the trained pose), 12 leg speeds
          12 its own previous output
          2  where we are in the step cycle, as sin and cos
        """
        phase = (walk_time % GAIT_PERIOD) / GAIT_PERIOD
        obs = np.concatenate(
            [
                gyro * GYRO_SCALE,
                gravity_in_body(quat),
                np.asarray(cmd_vel) * CMD_SCALE,
                leg_q - POLICY_LEGS,
                leg_dq * JOINT_SPEED_SCALE,
                self.last_action,
                [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)],
            ]
        ).astype(np.float32)
        action, self.memory_h, self.memory_c = self.session.run(
            None, {"obs": obs[None], "h_in": self.memory_h, "c_in": self.memory_c}
        )
        self.last_action = action[0]
        return POLICY_LEGS + ACTION_SCALE * self.last_action


def both_feet_down(phase):
    """True when, in Unitree's training gait, both feet are on the ground.

    Each foot is on the ground for the first 55% of its own cycle, and the
    right foot's cycle starts half a cycle later. So both are down when the
    phase is 0.00-0.05 or 0.50-0.55.
    """
    return phase < 0.05 or 0.5 <= phase < 0.55


class Locomotion:
    def __init__(self, onnx_path):
        self.policy = WalkPolicy(onnx_path)
        self.mode = STAND
        self.walk_start = 0.0
        self.still_since = None
        self.stand_start = -1e9  # long ago: the robot starts already standing
        self.blend_from = robot.STAND_LEGS.copy()

    def is_standing(self, time):
        """Standing still and steady (ready for the arms to work)."""
        return self.mode == STAND and time - self.stand_start > BLEND_TIME + STEADY_TIME

    def update(self, time, cmd_vel, leg_q, leg_dq, quat, gyro):
        """Call every 20 ms. Returns (targets, kp, kd) for legs + waist (15 joints)."""
        moving = any(abs(v) > 1e-3 for v in cmd_vel)

        if self.mode == STAND and moving:
            self.mode = WALK
            self.policy.reset()
            self.walk_start = time
            self.still_since = None

        if self.mode == WALK:
            if moving:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = time
            phase = ((time - self.walk_start) % GAIT_PERIOD) / GAIT_PERIOD
            settled = (
                self.still_since is not None and time - self.still_since >= SETTLE_TIME
            )
            if settled and both_feet_down(phase):
                self.mode = STAND
                self.stand_start = time
                self.blend_from = np.array(leg_q, dtype=float)

        if self.mode == WALK:
            legs = self.policy.step(
                cmd_vel, leg_q, leg_dq, quat, gyro, time - self.walk_start
            )
            leg_kp, leg_kd = robot.LEG_KP, robot.LEG_KD
        else:
            # smooth blend: 0 -> 1 over BLEND_TIME, starting and ending slowly
            a = min(1.0, (time - self.stand_start) / BLEND_TIME)
            a = a * a * (3 - 2 * a)
            legs = (1 - a) * self.blend_from + a * robot.STAND_LEGS
            lean = gravity_in_body(quat)[0]  # forward lean, rad (small angles)
            lean_speed = gyro[1]  # rad/s, forward positive
            legs[ANKLE_PITCH] += BALANCE_KP * lean + BALANCE_KD * lean_speed
            leg_kp, leg_kd = robot.STAND_LEG_KP, robot.STAND_LEG_KD

        targets = np.concatenate([legs, np.zeros(3)])  # waist straight
        kp = np.concatenate([leg_kp, robot.WAIST_KP])
        kd = np.concatenate([leg_kd, robot.WAIST_KD])
        return targets, kp, kd
