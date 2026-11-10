"""Inverse kinematics (IK) for one G1 arm.

Forward kinematics: 7 joint angles -> where the hand is. Easy, MuJoCo does it.
Inverse kinematics: where the hand should be -> 7 joint angles. That is what
we need, and there is no direct formula, so we search:

  1. Look where the hand is now and how far it is from the target (the error).
  2. Ask MuJoCo for the Jacobian: how much the hand moves when each joint
     turns a little.
  3. Turn each joint by the amount that shrinks the error the most
     (damped least squares, the standard method).
  4. Repeat until the error is below 1 mm.

Everything here is in the pelvis frame (origin in the pelvis, x forward,
y left, z up), with the waist straight. The arm node keeps it that way.
"""

import mujoco
import numpy as np

ARM_JOINTS = [
    "shoulder_pitch_joint",
    "shoulder_roll_joint",
    "shoulder_yaw_joint",
    "elbow_joint",
    "wrist_roll_joint",
    "wrist_pitch_joint",
    "wrist_yaw_joint",
]

DAMPING = 0.05  # keeps steps small near positions the arm can't reach
ROTATION_WEIGHT = (
    0.3  # 1 rad of hand rotation error counts like 30 cm of position error
)
POSITION_TOLERANCE = 0.001  # m
ROTATION_TOLERANCE = 0.01  # rad
MAX_STEP = 0.2  # rad per iteration, so the search doesn't jump around


class ArmIK:
    def __init__(self, model_path, side):
        """model_path: robot/g1_with_hands.xml. side: "left" or "right"."""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[0:7] = [0, 0, 0, 1, 0, 0, 0]  # pelvis at the origin, not rotated
        joint_ids = [self.model.joint(f"{side}_{name}").id for name in ARM_JOINTS]
        self.qpos_index = self.model.jnt_qposadr[joint_ids]
        self.dof_index = self.model.jnt_dofadr[joint_ids]
        self.lower = self.model.jnt_range[joint_ids, 0]
        self.upper = self.model.jnt_range[joint_ids, 1]
        self.site = self.model.site(f"{side}_hand_grasp").id

    def hand_pose(self, arm_q):
        """Forward kinematics: arm angles -> grasp point (position, 3x3 rotation)."""
        self.data.qpos[self.qpos_index] = arm_q
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[
            self.site
        ].reshape(3, 3).copy()

    def solve(self, position, rotation, arm_q, iterations=100):
        """Arm angles that put the grasp point at `position` with `rotation`.

        arm_q: where to start the search (use the current angles: the answer
        is then close to them, so the arm takes the short way).
        Returns (angles, reached). reached is False if the target is too far.
        """
        q = np.clip(np.array(arm_q, dtype=float), self.lower, self.upper)
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        for _ in range(iterations):
            hand_pos, hand_rot = self.hand_pose(q)
            pos_error = position - hand_pos
            # rotation error: the axis to turn around, times the angle (small angles)
            rot_error = 0.5 * sum(
                np.cross(hand_rot[:, i], rotation[:, i]) for i in range(3)
            )
            if (
                np.linalg.norm(pos_error) < POSITION_TOLERANCE
                and np.linalg.norm(rot_error) < ROTATION_TOLERANCE
            ):
                return q, True
            mujoco.mj_comPos(self.model, self.data)
            mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.site)
            jac = np.vstack(
                [
                    jac_pos[:, self.dof_index],
                    ROTATION_WEIGHT * jac_rot[:, self.dof_index],
                ]
            )
            error = np.concatenate([pos_error, ROTATION_WEIGHT * rot_error])
            # damped least squares: step = J^T (J J^T + damping^2 I)^-1 error
            step = jac.T @ np.linalg.solve(jac @ jac.T + DAMPING**2 * np.eye(6), error)
            step = np.clip(step, -MAX_STEP, MAX_STEP)
            q = np.clip(q + step, self.lower, self.upper)
        return q, False
