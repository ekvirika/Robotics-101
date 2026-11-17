"""What the G1 offers to the software that uses it: one small interface per job.

The split follows Unitree's own SDK for the G1 (unitree_sdk2):

  Locomotion    the walking controller: LocoClient.Move(vx, vy, vyaw),
                StopMove(), SetStandHeight(h). The robot moves its legs itself.
  UpperBody     the arm SDK (topic rt/arm_sdk): target angles for the waist
                and both arms, blended with the walking controller.
  Hands         the Dex3-1 hands (topics rt/dex3/left|right/cmd): finger targets.
  State         what the robot reports: time, joint angles (rt/lowstate) and
                where it is (odometry / localization).
  Lidar         the head lidar: one horizontal scan.
  Perception    where an object is (on the real robot: a camera and a detector).

Tasks and skills depend only on these interfaces, never on a simulator
(dependency inversion). A backend implements them: g1.isaac.IsaacG1 for
Isaac Lab; a real-robot backend on unitree_sdk2 would implement the same.
Each interface is small, so a part that only walks needs only Locomotion and
State (interface segregation).
"""

from abc import ABC, abstractmethod

import numpy as np


class Locomotion(ABC):
    @abstractmethod
    def move(self, vx: float, vy: float, vyaw: float) -> None:
        """Walk with this velocity: forward vx, left vy (m/s), turn vyaw (rad/s)."""

    def stop_move(self) -> None:
        """Stand still."""
        self.move(0.0, 0.0, 0.0)

    @abstractmethod
    def set_stand_height(self, height: float) -> None:
        """Pelvis height to stand and walk at (m)."""


class UpperBody(ABC):
    @abstractmethod
    def set_upper_body_targets(self, angles: np.ndarray) -> None:
        """Target angles (rad) of the 17 joints in g1.joints.UPPER_BODY (waist, left arm, right arm)."""


class Hands(ABC):
    @abstractmethod
    def set_hand_targets(self, side: str, angles: np.ndarray) -> None:
        """Target angles (rad) of one hand's 7 joints (g1.joints.HAND[side]); side is "left" or "right"."""


class State(ABC):
    @abstractmethod
    def time(self) -> float:
        """Robot time (s)."""

    @property
    @abstractmethod
    def dt(self) -> float:
        """Control period (s): how long one step() lasts."""

    @abstractmethod
    def joint_positions(self, names: list[str]) -> np.ndarray:
        """Current angles (rad) of the named joints, in that order."""

    @abstractmethod
    def pose(self) -> tuple[float, float, float]:
        """Where the robot is: (x, y, heading) in the world (m, m, rad)."""

    @abstractmethod
    def speed(self) -> float:
        """How fast the robot moves over the ground (m/s)."""

    @abstractmethod
    def pelvis_height(self) -> float:
        """Height of the pelvis above the floor (m)."""


class Lidar(ABC):
    @abstractmethod
    def scan(self) -> np.ndarray:
        """One horizontal turn: distances (m); ray k points k * 360/N degrees to
        the left of straight ahead. No hit = infinity (as in ROS)."""


class Perception(ABC):
    @abstractmethod
    def find(self, name: str) -> tuple[np.ndarray, float]:
        """Where an object is: position (x, y, z) in the world (m) and heading (rad)."""


class G1(Locomotion, UpperBody, Hands, State, Lidar, Perception, ABC):
    """The whole robot. Backends implement this; code that needs only part of
    the robot should ask for that part (e.g. Locomotion)."""

    @abstractmethod
    def step(self) -> None:
        """Let the robot run for one control period with the current commands."""
