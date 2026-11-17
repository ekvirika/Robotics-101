"""The G1 in NVIDIA Isaac Lab: robot configuration for environments, and IsaacG1,
which implements the robot's interfaces (g1.interfaces) on a running environment.

NVIDIA's parts, as in Isaac Lab's G1 loco-manipulation environment
(isaaclab_tasks/manager_based/locomanipulation/pick_place):

  robot      G1_29DOF_CFG: the G1 with two Dex3-1 hands (Isaac Lab asset)
  walking    the pre-trained AGILE lower-body policy (Policies/Agile/agile_locomotion.pt):
             command [vx, vy, wz, base height], moves hips, knees and ankles
  arms       joint position targets for the waist and arms (like Unitree's arm SDK)
  hands      joint position targets for the 14 finger joints (like Unitree's Dex3)
  lidar      360 horizontal rays from where the real G1 has its Livox MID-360
             (Unitree's model: 0.428 m above torso_link), with Isaac Lab's
             MultiMeshRayCaster, which follows moving objects (people)

Only import inside a running Isaac Sim (after AppLauncher).
"""

import math

import numpy as np
import torch

from isaaclab.envs import ManagerBasedRLEnvCfg, mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.ray_caster import MultiMeshRayCasterCfg, patterns
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.configclass import configclass

from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.action_cfg import AgileBasedLowerBodyActionCfg
from isaaclab_tasks.manager_based.locomanipulation.pick_place.configs.agile_locomotion_observation_cfg import (
    AgileTeacherPolicyObservationsCfg,
)

from isaaclab_assets.robots.unitree import G1_29DOF_CFG

from . import joints
from .interfaces import G1

# ---- robot configuration (for an environment's scene, actions, observations) ----

PRIM_PATH = "{ENV_REGEX_NS}/Robot"
# Base height sent with every walking command: NVIDIA's own value for this
# policy (isaaclab_mimic locomanipulation_sdg g1_locomanipulation_sdg_env.py).
STAND_HEIGHT = 0.8  # m
# The walking commands NVIDIA's AGILE G1 policy is trained for (WBC-AGILE,
# velocity_height_env_cfg.py): forward -0.5..1.5 m/s, sideways -0.5..0.5 m/s,
# turning -1..1 rad/s. Commands are clipped to them, as a robot's walking
# controller limits what it accepts.
WALK_LIMITS = np.array([[-0.5, -0.5, -1.0], [1.5, 0.5, 1.0]])
SIM_DT = 1 / 200  # s: physics at 200 Hz, as in NVIDIA's G1 loco-manipulation environment
DECIMATION = 4  # physics steps per control step: the walking policy runs at 50 Hz
# PhysX on the CPU: for one robot it is about 9x faster per step than on the GPU
# (measured on an L40S: 0.9 ms vs 8 ms per physics step); the GPU pays off when
# simulating thousands of robots at once (training). Scripts use it by default.
DEVICE = "cpu"


def robot_cfg(start=(0.0, 0.0, 0.0)):
    """NVIDIA's G1 with hands, standing at start = (x, y, heading)."""
    x, y, heading = start
    cfg = G1_29DOF_CFG.replace(prim_path=PRIM_PATH)
    cfg.init_state.pos = (x, y, cfg.init_state.pos[2])
    cfg.init_state.rot = (0.0, 0.0, math.sin(heading / 2), math.cos(heading / 2))  # (x, y, z, w)
    return cfg


@configclass
class G1ActionsCfg:
    """The robot's three command channels; the action vector is
    [walking (4), upper body (17), hands (14)]."""

    # The name is fixed: NVIDIA's AGILE observations read this term's last action.
    lower_body_joint_pos = AgileBasedLowerBodyActionCfg(
        asset_name="robot",
        joint_names=[".*_hip_.*_joint", ".*_knee_joint", ".*_ankle_.*_joint"],
        policy_output_scale=0.25,
        obs_group_name="lower_body_policy",
        policy_path=f"{ISAACLAB_NUCLEUS_DIR}/Policies/Agile/agile_locomotion.pt",
    )
    upper_body = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=joints.UPPER_BODY, preserve_order=True, scale=1.0, use_default_offset=False
    )
    hands = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=joints.HANDS, preserve_order=True, scale=1.0, use_default_offset=False
    )


@configclass
class G1ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """For a learning agent (not used by the robot's own software)."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, params={"asset_cfg": SceneEntityCfg("robot")})

    policy: PolicyCfg = PolicyCfg()
    # what NVIDIA's AGILE walking policy reads (the group name is set in G1ActionsCfg)
    lower_body_policy: AgileTeacherPolicyObservationsCfg = AgileTeacherPolicyObservationsCfg()


FALLEN_HEIGHT = 0.4  # m: pelvis below this = the robot is down (it stands at about 0.75 m)


@configclass
class G1SceneCfg(InteractiveSceneCfg):
    """One G1; a task adds its area (floor, sky, objects, people)."""

    robot = robot_cfg()


@configclass
class G1TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    fell = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={"minimum_height": FALLEN_HEIGHT, "asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class G1TaskEnvCfg(ManagerBasedRLEnvCfg):
    """What every G1 task shares: the robot, its command channels, the walking
    policy's observations, NVIDIA's timing, and "time out" / "fell" endings.
    A task subclasses it and adds its area, events, terminations and rewards.
    The robot software drives it; no reward is needed for that."""

    scene: G1SceneCfg = G1SceneCfg(num_envs=1, env_spacing=2.5)
    actions: G1ActionsCfg = G1ActionsCfg()
    observations: G1ObservationsCfg = G1ObservationsCfg()
    terminations: G1TerminationsCfg = G1TerminationsCfg()
    # as in NVIDIA's G1 loco-manipulation environment: no commands, rewards or curriculum
    commands = None
    rewards = None
    curriculum = None

    def __post_init__(self):
        self.sim.dt = SIM_DT
        self.decimation = DECIMATION
        # one picture per control step (50 Hz), as Isaac Lab's locomotion environments do
        self.sim.render_interval = DECIMATION


# ---- the robot's lidar ----

LIDAR_RAYS = 360  # one ray per degree, a 2D scan like ROS's pointcloud_to_laserscan makes from a 3D lidar
LIDAR_RANGE = 12.0  # m
LIDAR_NOISE = 0.01  # m: each distance is off by about 1 cm (standard deviation)
LIDAR_MOUNT = (0.0002835, 0.00003, 0.428434)  # m from torso_link: Unitree's G1 model, mid360_joint


def lidar_cfg(moving=(), still=()) -> MultiMeshRayCasterCfg:
    """The G1's lidar, seeing the prims in `moving` (followed every step, e.g.
    people) and `still` (e.g. tables). A task adds it as scene.lidar."""
    target = MultiMeshRayCasterCfg.RaycastTargetCfg
    return MultiMeshRayCasterCfg(
        prim_path=f"{PRIM_PATH}/torso_link",
        mesh_prim_paths=[target(prim_expr=p, track_mesh_transforms=True) for p in moving]
        + [target(prim_expr=p, track_mesh_transforms=False) for p in still],
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=LIDAR_MOUNT),
        ray_alignment="yaw",  # rays stay level when the body rocks while walking
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(0.0, 360.0),
            horizontal_res=360 / LIDAR_RAYS,
        ),
        max_distance=LIDAR_RANGE,
        update_period=0.0,  # every step; the robot software reads it every SCAN_DT
    )


# ---- the robot's interfaces, on a running environment ----


def yaw_of(quat) -> float:
    """Heading (rad) of a quaternion in Isaac Lab's order (x, y, z, w)."""
    x, y, z, w = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class IsaacG1(G1):
    """The G1 interfaces (Locomotion, UpperBody, Hands, State, Lidar,
    Perception) on an Isaac Lab environment whose actions are G1ActionsCfg."""

    def __init__(self, env, seed: int = 0):
        self.env = env
        self.robot = env.scene["robot"]
        self.origin = env.scene.env_origins[0].cpu().numpy()
        self.index = {name: i for i, name in enumerate(self.robot.joint_names)}
        self.noise = np.random.default_rng(seed + 1000)
        self.walk = np.array([0.0, 0.0, 0.0, STAND_HEIGHT])
        self.upper = self.joint_positions(joints.UPPER_BODY)
        self.hands = self.joint_positions(joints.HANDS)
        self.steps = 0
        self.last_step = None  # what env.step returned last time

    # Locomotion
    def move(self, vx, vy, vyaw):
        self.walk[:3] = np.clip((vx, vy, vyaw), *WALK_LIMITS)

    def set_stand_height(self, height):
        self.walk[3] = height

    # UpperBody
    def set_upper_body_targets(self, angles):
        self.upper = np.asarray(angles, float)

    # Hands
    def set_hand_targets(self, side, angles):
        start = 0 if side == "left" else len(joints.LEFT_HAND)
        self.hands[start : start + len(joints.HAND[side])] = angles

    # State
    def time(self):
        return self.steps * self.env.step_dt

    @property
    def dt(self):
        return self.env.step_dt

    def joint_positions(self, names):
        q = self.robot.data.joint_pos.torch[0].cpu().numpy()
        return np.array([q[self.index[n]] for n in names])

    def pose(self):
        x, y = self.robot.data.root_pos_w.torch[0, :2].cpu().numpy() - self.origin[:2]
        return float(x), float(y), yaw_of(self.robot.data.root_quat_w.torch[0].cpu().numpy())

    def speed(self):
        return float(torch.linalg.norm(self.robot.data.root_lin_vel_w.torch[0, :2]).item())

    def pelvis_height(self):
        return self.robot.data.root_pos_w.torch[0, 2].item()

    # Lidar
    def scan(self):
        """Distance along each ray (inf: nothing within range); ray k points k degrees
        to the left of the robot's forward direction."""
        lidar = self.env.scene["lidar"].data
        hits = lidar.ray_hits_w.torch[0].cpu().numpy()
        ranges = np.linalg.norm(hits - lidar.pos_w.torch[0].cpu().numpy(), axis=1)
        ranges[~np.isfinite(ranges) | (ranges > LIDAR_RANGE)] = np.inf
        return ranges + self.noise.normal(0.0, LIDAR_NOISE, LIDAR_RAYS)

    # Perception (the simulator knows where things are; a real robot would use a camera)
    def find(self, name):
        obj = self.env.scene[name]
        position = obj.data.root_pos_w.torch[0].cpu().numpy() - self.origin
        return position, yaw_of(obj.data.root_quat_w.torch[0].cpu().numpy())

    # running: as in Isaac Lab's own scripts (e.g. scripts/environments/zero_agent.py),
    # inside torch.inference_mode(): the walking policy runs without gradients
    @torch.inference_mode()
    def step(self):
        action = np.concatenate([self.walk, self.upper, self.hands])
        tensor = torch.tensor([action], dtype=torch.float32, device=self.env.device)
        self.last_step = self.env.step(tensor)
        self.steps += 1

    def reset(self):
        self.env.reset()
        self.steps = 0
        self.walk[:3] = 0.0
