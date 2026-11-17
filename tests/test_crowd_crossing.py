"""The crowd-crossing task's logic, with the stand-in robot (tests/fakes.py),
in many crowds of the environment's crowd model (area.crowd).

Scored like the crowd-navigation literature (e.g. CrowdNav, SocNavBench):
success rate, collisions, time to goal, smallest free space to a person.
"""

import numpy as np
from area.crowd import Crowd
from fakes import PERSON_RADIUS, FakeG1
from g1.planner import ROBOT_RADIUS

from tasks.crossing.layout import HALF_SIZE, NUM_PEOPLE, START, TIME_LIMIT
from tasks.crossing.mission import Crossing

CROWDS = range(20)


def cross(seed):
    crowd = Crowd(NUM_PEOPLE, HALF_SIZE, seed, keep_clear=(START[0], START[1], 2.0))
    robot = FakeG1(start=START, people=lambda: crowd.position)
    mission = Crossing(robot)
    closest = np.inf
    while not mission.done and robot.time() < TIME_LIMIT:
        mission.update()
        robot.step()
        crowd.step(robot.dt, np.array(robot.pose()[:2]))
        gap = np.linalg.norm(crowd.position - robot.pose()[:2], axis=1).min() - PERSON_RADIUS - ROBOT_RADIUS
        closest = min(closest, gap)
    return {"arrived": mission.done, "time": robot.time(), "closest": closest, "collided": closest < 0.0}


def test_no_collisions_and_every_crowd_crossed():
    results = [cross(seed) for seed in CROWDS]
    times = [r["time"] for r in results if r["arrived"]]
    print(
        f"\n{len(CROWDS)} crowds: success {len(times)}/{len(CROWDS)}, "
        f"collisions {sum(r['collided'] for r in results)}, "
        f"time to goal mean {np.mean(times):.1f} s (min {min(times):.1f}, max {max(times):.1f}), "
        f"smallest free space {100 * min(r['closest'] for r in results):.0f} cm"
    )
    assert not [r for r in results if r["collided"]]
    assert len(times) == len(CROWDS)
