"""The Unitree G1's joints, in Unitree's official order.

Body: the 29 joints in the order of Unitree's SDK (`G1JointIndex` in
unitree_sdk2_python example/g1/high_level/g1_arm7_sdk_dds_example.py),
the order of the robot's LowCmd/LowState messages. Hands: the 7 joints of
each Dex3-1 hand. The names are the joint names of Unitree's G1 model
(unitree_ros g1_29dof_with_hand_rev_1_0.urdf), which NVIDIA's Isaac Lab G1
(G1_29DOF_CFG) uses too.
"""

LEGS = [
    "left_hip_pitch_joint",  # 0
    "left_hip_roll_joint",  # 1
    "left_hip_yaw_joint",  # 2
    "left_knee_joint",  # 3
    "left_ankle_pitch_joint",  # 4
    "left_ankle_roll_joint",  # 5
    "right_hip_pitch_joint",  # 6
    "right_hip_roll_joint",  # 7
    "right_hip_yaw_joint",  # 8
    "right_knee_joint",  # 9
    "right_ankle_pitch_joint",  # 10
    "right_ankle_roll_joint",  # 11
]
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]  # 12, 13, 14
LEFT_ARM = [
    "left_shoulder_pitch_joint",  # 15
    "left_shoulder_roll_joint",  # 16
    "left_shoulder_yaw_joint",  # 17
    "left_elbow_joint",  # 18
    "left_wrist_roll_joint",  # 19
    "left_wrist_pitch_joint",  # 20
    "left_wrist_yaw_joint",  # 21
]
RIGHT_ARM = [name.replace("left_", "right_") for name in LEFT_ARM]  # 22..28
BODY = LEGS + WAIST + LEFT_ARM + RIGHT_ARM  # 29, Unitree's order

# The joints the arm SDK (rt/arm_sdk) commands: waist and both arms.
UPPER_BODY = WAIST + LEFT_ARM + RIGHT_ARM  # 17

# Dex3-1 hand: thumb (3 joints), index (2), middle (2), as in Unitree's model.
LEFT_HAND = [
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
]
RIGHT_HAND = [name.replace("left_", "right_") for name in LEFT_HAND]
HANDS = LEFT_HAND + RIGHT_HAND  # 14

ARM = {"left": LEFT_ARM, "right": RIGHT_ARM}
HAND = {"left": LEFT_HAND, "right": RIGHT_HAND}
