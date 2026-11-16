"""Run the whole mission without dora and without a window, and check the result.

The same parts as the dora graph (Simulator, Locomotion, ArmController,
Mission), wired together in one loop instead of exchanging messages. It runs
about 20 times faster than real time (about 2 s), so you can check every change.

  uv run python tests/test_mission.py      (prints the steps, PASS or FAIL)
  uv run --with pytest pytest tests/       (the same, as a pytest test)
"""

import math
import sys
from pathlib import Path

import numpy as np

LESSON = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LESSON))

from g1 import messages, robot  # noqa: E402
from g1.arm import ArmController  # noqa: E402
from g1.locomotion import POLICY_DT, Locomotion  # noqa: E402
from g1.mission import Mission  # noqa: E402
from g1.simulator import Simulator  # noqa: E402

# The room, as in robot/scene.xml and dataflow.yml
PLACE_POINT = (1.0, -1.52)
PLACE_HEADING = math.radians(-90)
PICK_HEADING = 0.0
TABLE_HEIGHT = 0.80
TIME_LIMIT = 150.0  # s of robot time


def run_mission(verbose=True):
    sim = Simulator(str(LESSON / "robot/scene.xml"))
    locomotion = Locomotion(str(LESSON / "policies/g1_walk.onnx"))
    arm = ArmController(str(LESSON / "robot/g1_with_hands.xml"))
    mission = Mission(PICK_HEADING, PLACE_POINT, PLACE_HEADING, TABLE_HEIGHT)

    arm_status = None
    arm_command_id = None
    label = ""
    next_control = 0.0
    lowest_pelvis = 1.0
    while sim.time < TIME_LIMIT and not mission.finished and not mission.failed:
        if sim.time >= next_control - 1e-9:  # the 50 Hz part: task, legs, arm
            next_control += POLICY_DT
            t = sim.time
            q, dq = sim.joint_state()
            quat, gyro = sim.imu()
            base_position, base_quat = sim.base_pose()
            bottle_position, _ = sim.bottle_pose()

            cmd_vel, command = mission.update(
                t,
                base_position,
                base_quat,
                bottle_position,
                locomotion.is_standing(t),
                arm_status,
            )
            if mission.label != label:
                label = mission.label
                if verbose and label:
                    print(f"t={t:6.2f}s  {label}")
            if command is not None:
                arm_command_id, kind, position, yaw = messages.unpack_arm_command(
                    command
                )
                if not arm.execute(t, kind, position, yaw):
                    arm_status = (arm_command_id, False)
                    arm_command_id = None

            sim.set_command(
                robot.LOWER_BODY,
                *locomotion.update(
                    t, cmd_vel, q[robot.LEGS], dq[robot.LEGS], quat, gyro
                ),
            )
            sim.set_command(robot.UPPER_BODY, *arm.update(t, q[robot.UPPER_BODY]))
            if arm_command_id is not None and not arm.busy():
                arm_status = (arm_command_id, True)
                arm_command_id = None
        sim.step()
        lowest_pelvis = min(lowest_pelvis, sim.data.qpos[2])
        if lowest_pelvis < 0.5:
            mission.failed = "the robot fell"

    bottle, bottle_quat = sim.bottle_pose()
    # tilt: angle between the bottle's axis and vertical (from its quaternion)
    w, x, y, z = bottle_quat
    tilt = math.degrees(math.acos(min(1.0, 1 - 2 * (x * x + y * y))))
    miss = float(np.hypot(bottle[0] - PLACE_POINT[0], bottle[1] - PLACE_POINT[1]))
    on_table = abs(bottle[2] - (TABLE_HEIGHT + 0.075)) < 0.01
    result = {
        "finished": mission.finished,
        "failed": mission.failed,
        "time": sim.time,
        "miss_cm": 100 * miss,
        "tilt_deg": tilt,
        "on_table": on_table,
    }
    if verbose:
        outcome = "finished" if mission.finished else f"FAILED: {mission.failed}"
        where = "standing on table B" if on_table else "NOT on table B"
        print(f"\nmission {outcome} at t={sim.time:.1f}s")
        print(
            f"bottle: {100 * miss:.1f} cm from the mark, tilt {tilt:.1f} deg, {where}"
        )
    return result


def test_mission():
    r = run_mission(verbose=False)
    assert r["finished"], r["failed"]
    assert r["on_table"] and r["miss_cm"] < 3.0 and r["tilt_deg"] < 10.0, r


if __name__ == "__main__":
    r = run_mission()
    ok = r["finished"] and r["on_table"] and r["miss_cm"] < 3.0 and r["tilt_deg"] < 10.0
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
