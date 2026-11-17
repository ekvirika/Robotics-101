"""The square in MuJoCo: the G1, the people, and the robot's lidar.

CrowdWorld is lesson 3's Simulator (the G1 with its motor drivers and
sensors) plus two things:
  people  solid figures that the Crowd model (pedestrians.py) moves around,
  lidar   a laser scanner on the robot's head that measures the distance to
          whatever is around it, like the lidar in the real G1's head.
"""

import mujoco
import numpy as np

from crowd.pedestrians import RADIUS, Crowd
from g1.simulator import Simulator

HALF_SIZE = 6.0  # m: the square reaches from -6 to +6 (world/square.xml)
CROWD_DT = 0.02  # s: the people decide where to go 50 times per second

# A person as a shape: a 1.7 m tall figure (average adult), body + head.
BODY_TOP = 1.45  # m: shoulder height
HEAD_HEIGHT = 1.58  # m: middle of the head
HEAD_RADIUS = 0.11  # m
SHIRTS = [  # so you can tell people apart in the window
    (0.80, 0.25, 0.25, 1),
    (0.25, 0.45, 0.80, 1),
    (0.90, 0.70, 0.20, 1),
    (0.30, 0.65, 0.40, 1),
    (0.60, 0.35, 0.70, 1),
    (0.85, 0.50, 0.25, 1),
    (0.35, 0.65, 0.70, 1),
    (0.55, 0.55, 0.55, 1),
]

# ---- the lidar: numbers of a typical small 2D lidar (for example RPLIDAR A2) ----
LIDAR_RAYS = 360  # one ray per degree
LIDAR_RATE = 10.0  # turns per second
LIDAR_RANGE = 12.0  # m: beyond this it sees nothing
LIDAR_NOISE = 0.01  # m: each distance is off by about 1 cm (standard deviation)
# Where it sits: on top of the head, 0.50 m above the torso's origin, so the
# laser passes over the robot's own shoulders and arms.
LIDAR_MOUNT = np.array([0.0, 0.0, 0.50])


class CrowdWorld(Simulator):
    def __init__(self, scene_path, start, people, seed):
        """start: the robot's (x, y, heading). people: how many. seed: which
        random crowd (the same seed always gives the same crowd)."""
        self.people = people
        super().__init__(scene_path, start)
        self.crowd = Crowd(
            people, HALF_SIZE, seed, keep_clear=(start[0], start[1], 2.0)
        )
        self.mocap = np.array(
            [self.model.body(f"person_{i}").mocapid[0] for i in range(people)]
        )
        self.person_bodies = np.array(
            [self.model.body(f"person_{i}").id for i in range(people)]
        )
        self.torso = self.model.body("torso_link").id
        angles = np.arange(LIDAR_RAYS) * 2 * np.pi / LIDAR_RAYS
        self.ray_directions = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros(LIDAR_RAYS)], axis=1
        )
        self.noise = np.random.default_rng(seed + 1000)
        self.next_crowd_step = 0.0
        self.touched = False  # did a person ever bump into the robot?
        self._place_people()

    def load_model(self, scene_path):
        """The square from the file, plus one solid figure per person."""
        spec = mujoco.MjSpec.from_file(scene_path)
        for i in range(self.people):
            # "mocap" = a body that we move ourselves; physics can't push it
            body = spec.worldbody.add_body(name=f"person_{i}", mocap=True)
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                fromto=[0, 0, RADIUS, 0, 0, BODY_TOP - RADIUS],
                size=[RADIUS, 0, 0],
                rgba=SHIRTS[i % len(SHIRTS)],
            )
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                pos=[0, 0, HEAD_HEIGHT],
                size=[HEAD_RADIUS, 0, 0],
                rgba=(0.85, 0.7, 0.6, 1),
            )
        return spec.compile()

    def _place_people(self):
        self.data.mocap_pos[self.mocap, 0:2] = self.crowd.position
        self.data.mocap_pos[self.mocap, 2] = 0.0

    # ---- physics ----

    def step(self):
        """2 ms: the people move a little, then the robot's physics step."""
        if self.time >= self.next_crowd_step - 1e-9:
            self.next_crowd_step += CROWD_DT
            self.crowd.step(CROWD_DT, self.data.qpos[0:2])
            self._place_people()
        super().step()
        touching = self.model.geom_bodyid[self.data.contact.geom[: self.data.ncon]]
        if np.isin(touching, self.person_bodies).any():
            self.touched = True

    # ---- the lidar ----

    def scan(self):
        """One turn of the lidar: LIDAR_RAYS distances in meters.

        Ray k points k degrees to the left of the robot's forward direction.
        Rays that hit nothing within LIDAR_RANGE are infinity.
        """
        rotation = self.data.xmat[self.torso].reshape(3, 3)
        origin = self.data.xpos[self.torso] + rotation @ LIDAR_MOUNT
        directions = self.ray_directions @ rotation.T  # the lidar tilts with the body
        hit = np.zeros(LIDAR_RAYS, dtype=np.int32)
        ranges = np.zeros(LIDAR_RAYS)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            origin,
            directions.ravel(),
            None,  # all geom groups
            1,  # also static geoms
            self.torso,  # don't hit the body the lidar sits on
            hit,
            ranges,
            None,
            LIDAR_RAYS,
            LIDAR_RANGE,
        )
        ranges += self.noise.normal(0.0, LIDAR_NOISE, LIDAR_RAYS)
        ranges[hit < 0] = np.inf
        return ranges

    def people_truth(self):
        """Where the people really are, (people, 2). Only for tests and charts:
        the robot's software never gets this."""
        return self.crowd.position.copy()
