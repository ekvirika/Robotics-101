"""The G1 robot in numbers: joint names, joint groups, poses and motor gains.

Every message that carries joint values uses the order of JOINTS below.
It is the order of the joints in robot/g1_with_hands.xml.
"""

import numpy as np

JOINTS = [
    # legs: Unitree's walking policy moves these 12
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    # waist
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # left arm
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    # left hand (Unitree Dex3-1: a thumb with 3 joints, two fingers with 2)
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    # right arm
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    # right hand
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
]
NUM_JOINTS = len(JOINTS)  # 43

# Groups, as positions in JOINTS
LEGS = slice(0, 12)
WAIST = slice(12, 15)
LEFT_ARM = slice(15, 22)
LEFT_HAND = slice(22, 29)
RIGHT_ARM = slice(29, 36)
RIGHT_HAND = slice(36, 43)
LOWER_BODY = slice(0, 15)  # legs + waist: the locomotion node commands these
UPPER_BODY = slice(15, 43)  # arms + hands: the arm node commands these

# ---- poses (rad) ----

# Standing still: hips -0.2, knees 0.4, ankles -0.2 (a small knee bend, feet flat).
# This puts the center of mass 2 cm in front of the ankles, over the middle of
# the foot (the foot reaches from 5 cm behind the ankle to 12 cm in front).
STAND_LEGS = np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0] * 2)

# Arms while walking (shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw).
# Unitree trained its walking policy on a G1 with fixed arms and light hands.
# Our G1 has Dex3 hands (2.3 kg more). With elbows at 1.2 the center of mass
# is where it was in training: 4.6 cm in front of the ankles. Then the robot
# stands in place when told to, instead of creeping forward. The hands also
# hang low (0.57 m), under any table top.
LEFT_ARM_WALK = np.array([0.0, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_ARM_WALK = np.array([0.0, -0.15, 0.0, 1.2, 0.0, 0.0, 0.0])

# Dex3 hand (thumb 0, 1, 2, middle 0, 1, index 0, 1). Right and left are mirror images.
# Open: fingers straight, thumb swung back toward the wrist (out of the way).
# Closed: fingers and thumb curled past where a 6 cm bottle stops them, so
# the motors keep squeezing.
RIGHT_HAND_OPEN = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
RIGHT_HAND_CLOSED = np.array([0.0, -0.3, -1.2, 1.3, 1.3, 1.3, 1.3])
LEFT_HAND_OPEN = -RIGHT_HAND_OPEN
LEFT_HAND_CLOSED = -RIGHT_HAND_CLOSED

# ---- motor gains ----
# Each motor: torque = KP * (target - angle) + KD * (0 - speed) + extra torque.
# Same as the G1's motor drivers (Unitree's LowCmd: q, kp, kd, tau).

# Walking: the gains Unitree trained the policy with (unitree_rl_gym g1.yaml).
LEG_KP = np.array([100, 100, 100, 150, 40, 40] * 2, dtype=float)
LEG_KD = np.array([2, 2, 2, 4, 2, 2] * 2, dtype=float)

# Standing: stiffer. The robot is an upside-down pendulum on its ankles:
# 34 kg x 9.81 x 0.65 m = 220 Nm per radian of lean pulls it over, so the two
# ankles together need more than that (2 x 250 = 500).
STAND_LEG_KP = np.array([150, 150, 150, 225, 250, 250] * 2, dtype=float)
STAND_LEG_KD = np.array([4, 4, 4, 8, 8, 8] * 2, dtype=float)

# Waist and arms: the gains Unitree uses on the real G1 with this policy
# (unitree_rl_gym deploy/deploy_real/configs/g1.yaml). One change: waist KD 6
# instead of 3. The real motor's gearbox adds damping that the simulation
# lacks; with 3 the torso twists at every step and the robot slowly turns.
WAIST_KP = np.array([300.0, 300.0, 300.0])
WAIST_KD = np.array([6.0, 6.0, 6.0])
ARM_KP = np.array([100, 100, 50, 50, 20, 20, 20], dtype=float)
ARM_KD = np.array([2, 2, 2, 2, 1, 1, 1], dtype=float)
# Hands: small finger motors (1.4 Nm at most). With KP 5, a target 0.3 rad past
# the bottle already asks for the full 1.4 Nm squeeze.
HAND_KP = np.array([5.0] * 7)
HAND_KD = np.array([0.1] * 7)
