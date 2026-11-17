"""Isaac Lab MDP terms for places with people: walk the crowd, detect a collision.

Only import inside a running Isaac Sim (after AppLauncher).
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
import torch

from isaaclab.managers import ManagerTermBase

from .crowd import Crowd

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv
    from isaaclab.managers import EventTermCfg


class MoveCrowd(ManagerTermBase):
    """Event term: walk the people (scene entities person_0 .. person_{n-1})
    with the crowd model; they step around each other and the robot.

    Use it in "interval" mode with the interval equal to the environment step,
    so it runs once per step. Each new episode gets the next crowd
    (seed, seed + 1, ...).

    Params: num_people, half_size (m: the area reaches from -half_size to
    +half_size), body_height (m: center height of a person), seed,
    keep_clear ((x, y, radius) where nobody starts).
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        params = cfg.params
        self.num_people = params["num_people"]
        self.half_size = params["half_size"]
        self.body_height = params["body_height"]
        self.seed = params["seed"]
        self.keep_clear = params["keep_clear"]
        self.people = [env.scene[f"person_{i}"] for i in range(self.num_people)]
        self.robot = env.scene["robot"]
        self.origin = env.scene.env_origins[0].cpu().numpy()
        self.episode = -1
        self.reset()

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self.episode += 1
        self.crowd_seed = self.seed + self.episode
        self.crowd = Crowd(self.num_people, self.half_size, self.crowd_seed, keep_clear=self.keep_clear)
        self._place()

    def __call__(self, env: ManagerBasedEnv, env_ids, num_people, half_size, body_height, seed, keep_clear):
        robot_xy = self.robot.data.root_pos_w.torch[0, :2].cpu().numpy() - self.origin[:2]
        self.crowd.step(env.step_dt, robot_xy)
        self._place()

    def positions(self) -> np.ndarray:
        return self.crowd.position.copy()

    def _place(self) -> None:
        for person, (x, y) in zip(self.people, self.crowd.position):
            # position, then upright orientation as a quaternion in Isaac Lab's order (x, y, z, w)
            pose = torch.tensor(
                [[self.origin[0] + x, self.origin[1] + y, self.body_height, 0.0, 0.0, 0.0, 1.0]],
                dtype=torch.float32,
                device=self.device,
            )
            person.write_root_pose_to_sim(pose)


def person_collision(
    env: ManagerBasedRLEnv, num_people: int, robot_radius: float, person_radius: float
) -> torch.Tensor:
    """Termination term: a person's body overlaps the robot's footprint circle
    (the collision test of the crowd-navigation literature, e.g. CrowdNav)."""
    robot_xy = env.scene["robot"].data.root_pos_w.torch[:, :2]
    hit = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    for i in range(num_people):
        person_xy = env.scene[f"person_{i}"].data.root_pos_w.torch[:, :2]
        hit |= torch.linalg.norm(person_xy - robot_xy, dim=1) < robot_radius + person_radius
    return hit
