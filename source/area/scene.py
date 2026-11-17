"""Building blocks for the places tasks happen in (Isaac Lab scene configurations).

Each function returns one ready-made scene entity; a task's environment puts
together the ones it needs. Ground and sky are NVIDIA's own (the scene of
Isaac Lab's locomotion environments, velocity_env_cfg.py).

Only import inside a running Isaac Sim (after AppLauncher).
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

from .crowd import RADIUS as PERSON_RADIUS

PERSON_HEIGHT = 1.70  # m: an adult
SHIRTS = [
    (0.80, 0.25, 0.25),
    (0.25, 0.45, 0.80),
    (0.90, 0.70, 0.20),
    (0.30, 0.65, 0.40),
    (0.60, 0.35, 0.70),
    (0.85, 0.50, 0.25),
    (0.35, 0.65, 0.70),
    (0.55, 0.55, 0.55),
]


def ground() -> TerrainImporterCfg:
    """A flat floor with NVIDIA's tile material and friction 1.0 (as in Isaac Lab's
    locomotion environments)."""
    return TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
    )


def sky() -> AssetBaseCfg:
    """A photographed clear sky that lights the scene (NVIDIA's, as in Isaac Lab's locomotion environments)."""
    return AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


def person(i: int) -> RigidObjectCfg:
    """One walking person: a solid figure moved by code ("kinematic": physics
    can't push it, but it pushes the robot). Moved by terms.MoveCrowd."""
    return RigidObjectCfg(
        prim_path=f"/World/People/person_{i}",
        spawn=sim_utils.CapsuleCfg(
            radius=PERSON_RADIUS,
            height=PERSON_HEIGHT - 2 * PERSON_RADIUS,
            axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=SHIRTS[i % len(SHIRTS)]),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PERSON_HEIGHT / 2)),
    )


def mark(name: str, xy, color, radius=0.3, height=0.0) -> AssetBaseCfg:
    """A flat colored disc drawn at `height` (the floor by default), no collision."""
    return AssetBaseCfg(
        prim_path=f"/World/{name}",
        spawn=sim_utils.CylinderCfg(
            radius=radius, height=0.002, axis="Z", visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(xy[0], xy[1], height + 0.002)),
    )


def table(name: str, center_xy, size=(0.6, 0.8, 0.8), color=(0.55, 0.38, 0.22)) -> AssetBaseCfg:
    """A solid table: a box of size (depth, width, height) standing on the floor."""
    return AssetBaseCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(center_xy[0], center_xy[1], size[2] / 2)),
    )


def bottle(name: str, position, radius=0.03, height=0.15, mass=0.3) -> RigidObjectCfg:
    """A bottle that can be picked up: a cylinder (6 cm wide, 15 cm tall, 0.3 kg
    by default) standing upright at `position` (its bottom)."""
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/{name}",
        spawn=sim_utils.CylinderCfg(
            radius=radius,
            height=height,
            axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=mass),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.55, 0.9)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(position[0], position[1], position[2] + height / 2)),
    )
