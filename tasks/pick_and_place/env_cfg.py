"""The pick-and-place environment: two tables, a bottle on table A, a green
mark on table B, the G1 in front of them.

It ends when the bottle falls to the floor (failure), the robot falls
(failure) or time runs out (failure); success is checked by the mission's
runner (bottle on the mark, upright).
"""

from area import scene
from g1.isaac import G1TaskEnvCfg, lidar_cfg, robot_cfg

from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass

from .layout import BOTTLE_START, PLACE_MARK, START, TABLE_A, TABLE_B, TABLE_SIZE, TIME_LIMIT

MARK_COLOR = (0.2, 0.8, 0.3)
MARK_RADIUS = 0.05  # m: a 10 cm green disc on table B
FLOOR_LEVEL = 0.3  # m: a bottle center below this has fallen off the table


@configclass
class EventCfg:
    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform, mode="reset", params={"pose_range": {}, "velocity_range": {}}
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (1.0, 1.0), "velocity_range": (0.0, 0.0)},
    )
    reset_bottle = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {}, "velocity_range": {}, "asset_cfg": SceneEntityCfg("bottle")},
    )


@configclass
class PickPlaceEnvCfg(G1TaskEnvCfg):
    events: EventCfg = EventCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = TIME_LIMIT
        self.scene.robot = robot_cfg(START)
        self.scene.ground = scene.ground()
        self.scene.sky = scene.sky()
        self.scene.table_a = scene.table("TableA", TABLE_A, TABLE_SIZE)
        # table B stands turned by 90 degrees (the robot faces it from the side)
        self.scene.table_b = scene.table(
            "TableB", TABLE_B, (TABLE_SIZE[1], TABLE_SIZE[0], TABLE_SIZE[2]), (0.9, 0.9, 0.88)
        )
        self.scene.lidar = lidar_cfg(still=["{ENV_REGEX_NS}/TableA", "{ENV_REGEX_NS}/TableB"])
        self.scene.bottle = scene.bottle("Bottle", (BOTTLE_START[0], BOTTLE_START[1], TABLE_SIZE[2]))
        self.scene.place_mark = scene.mark(
            "PlaceMark", PLACE_MARK, MARK_COLOR, radius=MARK_RADIUS, height=TABLE_SIZE[2]
        )
        self.terminations.bottle_dropped = DoneTerm(
            func=mdp.root_height_below_minimum,
            params={"minimum_height": FLOOR_LEVEL, "asset_cfg": SceneEntityCfg("bottle")},
        )
