"""The crowd-crossing environment: a 12 m x 12 m square, 16 walking people,
the G1 on the west side, the goal on the east side.

It ends when the robot touches a person (failure), falls (failure) or runs
out of time (failure); arriving is checked by the mission (tasks/crossing/mission.py).
"""

from area import scene, terms
from area.crowd import RADIUS as PERSON_RADIUS
from g1.isaac import G1TaskEnvCfg, lidar_cfg, robot_cfg
from g1.planner import ROBOT_RADIUS

from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass

from .layout import CROWD_SEED, GOAL, HALF_SIZE, NUM_PEOPLE, START, TIME_LIMIT

START_COLOR = (0.2, 0.4, 0.9)
GOAL_COLOR = (0.2, 0.8, 0.3)


@configclass
class EventCfg:
    # every episode: the robot back on the start mark, standing still, in its default pose
    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform, mode="reset", params={"pose_range": {}, "velocity_range": {}}
    )
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (1.0, 1.0), "velocity_range": (0.0, 0.0)},
    )
    # the people walk once per control step (the interval is set in __post_init__)
    move_crowd = EventTerm(
        func=terms.MoveCrowd,
        mode="interval",
        interval_range_s=(0.0, 0.0),
        is_global_time=True,
        params={
            "num_people": NUM_PEOPLE,
            "half_size": HALF_SIZE,
            "body_height": scene.PERSON_HEIGHT / 2,
            "seed": CROWD_SEED,
            "keep_clear": (START[0], START[1], 2.0),
        },
    )


@configclass
class CrossingEnvCfg(G1TaskEnvCfg):
    events: EventCfg = EventCfg()

    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = TIME_LIMIT
        # the area
        self.scene.robot = robot_cfg(START)
        self.scene.ground = scene.ground()
        self.scene.sky = scene.sky()
        self.scene.start_mark = scene.mark("StartMark", START[:2], START_COLOR)
        self.scene.goal_mark = scene.mark("GoalMark", GOAL, GOAL_COLOR)
        for i in range(NUM_PEOPLE):
            setattr(self.scene, f"person_{i}", scene.person(i))
        self.scene.lidar = lidar_cfg(moving=["/World/People/person_.*"])
        # the people walk once per control step
        step = self.decimation * self.sim.dt
        self.events.move_crowd.interval_range_s = (step, step)
        # touching a person ends the episode
        self.terminations.person_collision = DoneTerm(
            func=terms.person_collision,
            params={"num_people": NUM_PEOPLE, "robot_radius": ROBOT_RADIUS, "person_radius": PERSON_RADIUS},
        )
