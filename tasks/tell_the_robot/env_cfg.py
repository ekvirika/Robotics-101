"""The tell-the-robot environment: the pick-and-place room (two tables, the
green mark on table B, the G1 at the start) with the objects of layout.OBJECTS
standing on the tables.

Scene names are the objects' names (blue_bottle, red_can, ...), so the sim
node reports them with OBJECTS=blue_bottle,red_can,... and Perception.find
finds them by the same name.

It ends when any object falls to the floor (failure), the robot falls
(failure) or time runs out. Whether a request was done right is for the
person who asked; scored runs check it (scripts/run_task.py does not run
this task: it has no fixed mission).
"""

from area import scene
from g1.isaac import G1TaskEnvCfg, lidar_cfg, robot_cfg

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass

from .layout import OBJECTS, PLACE_MARK, START, TABLE_A, TABLE_B, TABLE_SIZE, TIME_LIMIT

MARK_COLOR = (0.2, 0.8, 0.3)
MARK_RADIUS = 0.05  # m: the 10 cm green disc of pick and place
FLOOR_LEVEL = 0.3  # m: an object center below this has fallen off the table

COLORS = {
    "blue": (0.2, 0.55, 0.9),
    "red": (0.85, 0.2, 0.2),
    "yellow": (0.95, 0.8, 0.2),
    "green": (0.25, 0.65, 0.3),
}
CYLINDER = {"bottle": (0.03, 0.3), "can": (0.033, 0.35)}  # radius (m), mass (kg)
BOX_SIDE, BOX_MASS = 0.05, 0.2  # m, kg: its corners stay inside skills.OBJECT_RADIUS


def thing_cfg(thing) -> RigidObjectCfg:
    """One object of layout.OBJECTS, standing upright on its table."""
    color = sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS[thing.color])
    common = {
        "rigid_props": sim_utils.RigidBodyPropertiesCfg(),
        "collision_props": sim_utils.CollisionPropertiesCfg(),
        "physics_material": sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
        "visual_material": color,
    }
    if thing.kind == "box":
        spawn = sim_utils.CuboidCfg(
            size=(BOX_SIDE, BOX_SIDE, thing.height), mass_props=sim_utils.MassPropertiesCfg(mass=BOX_MASS), **common
        )
    else:
        radius, mass = CYLINDER[thing.kind]
        spawn = sim_utils.CylinderCfg(
            radius=radius,
            height=thing.height,
            axis="Z",
            mass_props=sim_utils.MassPropertiesCfg(mass=mass),
            **common,
        )
    position = (thing.xy[0], thing.xy[1], TABLE_SIZE[2] + thing.height / 2)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{thing.name}",
        spawn=spawn,
        init_state=RigidObjectCfg.InitialStateCfg(pos=position),
    )


def reset_term(name):
    return EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {}, "velocity_range": {}, "asset_cfg": SceneEntityCfg(name)},
    )


def dropped_term(name):
    return DoneTerm(
        func=mdp.root_height_below_minimum,
        params={"minimum_height": FLOOR_LEVEL, "asset_cfg": SceneEntityCfg(name)},
    )


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


@configclass
class TellTheRobotEnvCfg(G1TaskEnvCfg):
    events: EventCfg = EventCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = TIME_LIMIT
        self.scene.robot = robot_cfg(START)
        self.scene.ground = scene.ground()
        self.scene.sky = scene.sky()
        self.scene.table_a = scene.table("TableA", TABLE_A, TABLE_SIZE)
        self.scene.table_b = scene.table(
            "TableB", TABLE_B, (TABLE_SIZE[1], TABLE_SIZE[0], TABLE_SIZE[2]), (0.9, 0.9, 0.88)
        )
        self.scene.lidar = lidar_cfg(still=["{ENV_REGEX_NS}/TableA", "{ENV_REGEX_NS}/TableB"])
        self.scene.place_mark = scene.mark(
            "PlaceMark", PLACE_MARK, MARK_COLOR, radius=MARK_RADIUS, height=TABLE_SIZE[2]
        )
        for thing in OBJECTS:
            setattr(self.scene, thing.name, thing_cfg(thing))
            setattr(self.events, f"reset_{thing.name}", reset_term(thing.name))
            setattr(self.terminations, f"{thing.name}_dropped", dropped_term(thing.name))
