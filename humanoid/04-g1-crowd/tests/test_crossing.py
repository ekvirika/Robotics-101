"""Run the crossing without dora and without a window, and check the result.

The same parts as the dora graph (CrowdWorld, Locomotion, ArmController,
Tracker, Planner), wired together in one loop instead of exchanging messages.

  uv run python tests/test_crossing.py           (5 different crowds, PASS or FAIL)
  uv run python tests/test_crossing.py 3         (only the crowd with seed 3)
  uv run --with pytest pytest tests/             (the same, as pytest tests)
"""

import sys
from pathlib import Path

import numpy as np

LESSON = Path(__file__).resolve().parents[1]
G1 = LESSON.parent / "03-g1-pick-place"  # the robot software and model from lesson 3
sys.path[:0] = [str(LESSON), str(G1)]

from crowd.pedestrians import RADIUS  # noqa: E402
from crowd.planner import ROBOT_RADIUS, Planner  # noqa: E402
from crowd.tracker import SCAN_DT, Tracker  # noqa: E402
from crowd.world import CrowdWorld  # noqa: E402
from g1 import robot  # noqa: E402
from g1.arm import ArmController  # noqa: E402
from g1.locomotion import POLICY_DT, Locomotion  # noqa: E402
from g1.simulator import yaw_of  # noqa: E402

# The task, as in dataflow.yml
START = (-5.0, 0.0, 0.0)
GOAL = (5.0, 0.0)
PEOPLE = 16
TIME_LIMIT = 90.0  # s of robot time
SEEDS = (0, 1, 2, 3, 4)


def run_crossing(seed, verbose=True, record=None, stop_at=None):
    """One crossing. Returns the results, as a dictionary.

    record: an optional list that gets (time, robot xy, people xy, tracks)
    at every lidar scan. stop_at: stop at this time and return the world
    instead (for pictures).
    """
    world = CrowdWorld(str(LESSON / "world/square.xml"), START, PEOPLE, seed)
    locomotion = Locomotion(str(G1 / "policies/g1_walk.onnx"))
    arm = ArmController(str(G1 / "robot/g1_with_hands.xml"))
    tracker = Tracker()
    planner = Planner(GOAL)

    tracks = np.zeros((0, 4))
    next_control = next_scan = 0.0
    closest = np.inf
    fell = False
    while world.time < (stop_at or TIME_LIMIT) and not fell:
        if world.time >= next_control - 1e-9:  # the 50 Hz part
            next_control += POLICY_DT
            t = world.time
            q, dq = world.joint_state()
            quat, gyro = world.imu()
            position, base_quat = world.base_pose()
            heading = yaw_of(base_quat)
            if t >= next_scan - 1e-9:  # the 10 Hz part: lidar, tracker, planner
                next_scan += SCAN_DT
                pose = (position[0], position[1], heading)
                tracks = tracker.update(world.scan(), pose)
                planner.plan(position, tracks)
                if record is not None:
                    record.append(
                        (t, position[:2].copy(), world.people_truth(), tracks)
                    )
            cmd_vel = planner.to_cmd_vel(heading, POLICY_DT)
            world.set_command(
                robot.LOWER_BODY,
                *locomotion.update(
                    t, cmd_vel, q[robot.LEGS], dq[robot.LEGS], quat, gyro
                ),
            )
            world.set_command(robot.UPPER_BODY, *arm.update(t, q[robot.UPPER_BODY]))
            gaps = np.linalg.norm(world.people_truth() - position[:2], axis=1)
            closest = min(closest, gaps.min() - RADIUS - ROBOT_RADIUS)
            if planner.arrived and locomotion.is_standing(t):
                break
        world.step()
        fell = world.data.qpos[2] < 0.5

    if stop_at:
        return world
    position, _ = world.base_pose()
    result = {
        "seed": seed,
        "arrived": planner.arrived,
        "fell": fell,
        "touched": world.touched,
        "time": world.time,
        "closest_cm": 100 * closest,
        "off_goal_cm": 100 * float(np.hypot(*(position[:2] - GOAL))),
    }
    if verbose:
        outcome = "arrived" if planner.arrived else "did NOT arrive"
        problems = (", FELL" if fell else "") + (
            ", TOUCHED a person" if world.touched else ""
        )
        print(
            f"crowd {seed}: {outcome} at t={world.time:5.1f}s{problems}, "
            f"closest free space to a person {100 * closest:4.0f} cm, "
            f"stopped {result['off_goal_cm']:.0f} cm from the goal"
        )
    return result


def passed(r):
    return r["arrived"] and not r["fell"] and not r["touched"]


def test_crossing():
    for seed in SEEDS:
        r = run_crossing(seed, verbose=False)
        assert passed(r), r


if __name__ == "__main__":
    seeds = [int(a) for a in sys.argv[1:]] or SEEDS
    ok = all([passed(run_crossing(seed)) for seed in seeds])
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
