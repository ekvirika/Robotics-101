"""A stand-in G1 for tests: implements the robot's interfaces (g1.interfaces)
with simple, exact behavior, so the tasks' logic can be tested on any computer.

  walking   the body moves exactly with the commanded velocity
  joints    every joint is exactly at its target
  arm       real kinematics (Unitree's G1 model, g1.kinematics)
  lidar     a perfect 2D scan of round people and box-shaped tables
  objects   a bottle: held while the closed right hand's grasp point is on it,
            set down on whatever table is under it when the hand opens

What it does not test is physics (balance, contact, slipping): that is what
the Isaac Lab runs are for.
"""

import math

import numpy as np
from g1 import joints
from g1.interfaces import G1
from g1.kinematics import ArmIK, grasp_point, hand_pose

PERSON_RADIUS = 0.22  # m (area.crowd.RADIUS)
PELVIS_HEIGHT = 0.75  # m: NVIDIA's G1 standing height (G1_29DOF_CFG init_state)
WALK_LIMITS = np.array([[-0.5, -0.5, -1.0], [1.5, 0.5, 1.0]])  # as g1.isaac.WALK_LIMITS
GRASP_TOLERANCE = 0.02  # m: the closed hand holds the bottle if its grasp point is this close to its center


def scan_circles_and_boxes(origin, heading, circles, radius, boxes, rays=360, max_range=12.0):
    """Distances along `rays` horizontal rays to circles (people) and boxes (tables)."""
    angles = np.arange(rays) * 2 * np.pi / rays + heading
    dx_ray, dy_ray = np.cos(angles), np.sin(angles)
    ranges = np.full(rays, np.inf)
    for px, py in circles:
        dx, dy = px - origin[0], py - origin[1]
        along = dx * dx_ray + dy * dy_ray
        miss2 = dx * dx + dy * dy - along**2
        hit = (along > 0) & (miss2 < radius**2)
        ranges = np.where(hit, np.minimum(ranges, along - np.sqrt(np.maximum(radius**2 - miss2, 0))), ranges)
    for x0, y0, x1, y1 in boxes:  # slab method
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.stack([(x0 - origin[0]) / dx_ray, (x1 - origin[0]) / dx_ray])
            ty = np.stack([(y0 - origin[1]) / dy_ray, (y1 - origin[1]) / dy_ray])
        t_near = np.maximum(np.nanmin(tx, axis=0), np.nanmin(ty, axis=0))
        t_far = np.minimum(np.nanmax(tx, axis=0), np.nanmax(ty, axis=0))
        hit = (t_near <= t_far) & (t_far > 0)
        ranges = np.where(hit, np.minimum(ranges, np.maximum(t_near, 0.0)), ranges)
    ranges[ranges > max_range] = np.inf
    return ranges


class FakeG1(G1):
    def __init__(self, start=(0.0, 0.0, 0.0), people=None, tables=(), bottle=None, table_height=0.8):
        """people: a function returning the people's (n, 2) positions now.
        tables: footprints (x_min, y_min, x_max, y_max). bottle: its center (x, y, z)."""
        self.x, self.y, self.heading = start
        self.cmd = np.zeros(3)
        self.steps = 0
        self.people = people or (lambda: np.zeros((0, 2)))
        self.tables = list(tables)
        self.table_height = table_height
        self.angles = {name: 0.0 for name in joints.BODY + joints.HANDS}
        self.bottle = None if bottle is None else np.array(bottle, float)
        self.held = False
        self.ik = ArmIK("right")
        self.grasp = grasp_point("right")
        self.closed = hand_pose("right", closed=True)

    dt = 0.02  # s: control period, as the Isaac Lab environments (50 Hz)

    # Locomotion
    def move(self, vx, vy, vyaw):
        self.cmd = np.clip((vx, vy, vyaw), *WALK_LIMITS)

    def set_stand_height(self, height):
        pass

    # UpperBody, Hands
    def set_upper_body_targets(self, angles):
        self.angles.update(zip(joints.UPPER_BODY, angles))

    def set_hand_targets(self, side, angles):
        self.angles.update(zip(joints.HAND[side], angles))

    # State
    def time(self):
        return self.steps * self.dt

    def joint_positions(self, names):
        return np.array([self.angles[n] for n in names])

    def pose(self):
        return self.x, self.y, self.heading

    def speed(self):
        return float(np.hypot(*self.cmd[:2]))

    def pelvis_height(self):
        return PELVIS_HEIGHT

    # Lidar, Perception
    def scan(self):
        return scan_circles_and_boxes((self.x, self.y), self.heading, self.people(), PERSON_RADIUS, self.tables)

    def find(self, name):
        assert name == "bottle"
        return self.bottle.copy(), 0.0

    # the world moves on
    def step(self):
        forward, left, turn = self.cmd
        c, s = math.cos(self.heading), math.sin(self.heading)
        self.x += self.dt * (c * forward - s * left)
        self.y += self.dt * (s * forward + c * left)
        self.heading += self.dt * turn
        self.steps += 1
        if self.bottle is not None:
            self._bottle()

    def grasp_point_world(self):
        """Where the right hand's grasp point is, in the world."""
        palm, rotation = self.ik.palm_pose(self.joint_positions(joints.RIGHT_ARM), self.joint_positions(joints.WAIST))
        local = palm + rotation @ self.grasp
        c, s = math.cos(self.heading), math.sin(self.heading)
        return np.array(
            [self.x + c * local[0] - s * local[1], self.y + s * local[0] + c * local[1], PELVIS_HEIGHT + local[2]]
        )

    def _bottle(self):
        closed = np.allclose(self.joint_positions(joints.RIGHT_HAND), self.closed, atol=1e-6)
        point = self.grasp_point_world()
        if closed and not self.held and np.linalg.norm(point - self.bottle) < GRASP_TOLERANCE:
            self.held = True
        if self.held and not closed:
            self.held = False
            on_table = any(x0 <= point[0] <= x1 and y0 <= point[1] <= y1 for x0, y0, x1, y1 in self.tables)
            self.bottle[2] = self.table_height + 0.075 if on_table else 0.075  # set down (or dropped)
        if self.held:
            self.bottle = point
