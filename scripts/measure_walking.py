"""Measure the G1's real walking speed and braking in Isaac Lab.

Two numbers the robot software relies on, measured instead of assumed:
  top speed  does the walking policy reach the commanded speed? (g1.planner
             MAX_SPEED is set from the AGILE policy's training range, 1.5 m/s)
  braking    how fast it stops when the command drops to zero; ISO/TS 15066
             takes stopping time and distance from ISO 10218-1, which asks for
             them to be measured. Sets g1.planner.MAX_DECEL for the safety layer.

    $ISAACLAB/isaaclab.sh -p scripts/measure_walking.py --headless

It uses the crowd-crossing square with the people kept 50 m away.
"""

import argparse
import inspect

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Measure the G1's walking speed and braking.")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")  # g1.isaac.DEVICE: one robot simulates faster on the CPU
args = parser.parse_args()
app = AppLauncher(args).app

# ---- from here on Isaac Sim is running ----
import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
from g1.isaac import IsaacG1  # noqa: E402
from g1.planner import MAX_ACCEL, MAX_DECEL  # noqa: E402

from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import tasks  # noqa: E402, F401

SPEEDS = (0.5, 1.0, 1.5)  # m/s commanded: up to the AGILE policy's training range
HOLD = 3.0  # s at the commanded speed, after speeding up
STILL = 0.05  # m/s


def step(env, robot, forward):
    robot.move(forward, 0.0, 0.0)
    robot.step()
    term = env.event_manager.get_term_cfg("move_crowd").func
    if not inspect.isclass(term):  # the crowd term exists from the first step on
        term.crowd.position[:] = 50.0


def main():
    env = gym.make(
        "G1-Crowd-Crossing-v0", cfg=parse_env_cfg("G1-Crowd-Crossing-v0", device=args.device, num_envs=1)
    ).unwrapped
    env.reset()
    robot = IsaacG1(env)
    decels = []
    for target in SPEEDS:
        robot.reset()
        command, speeds = 0.0, []
        while command < target or robot.time() < target / MAX_ACCEL + HOLD:  # speed up gently, then hold
            command = min(target, command + MAX_ACCEL * robot.dt)
            step(env, robot, command)
            if command >= target:
                speeds.append(robot.speed())
        reached = float(np.mean(speeds[-int(1.0 / robot.dt) :]))  # the last second
        start, t0 = np.array(robot.pose()[:2]), robot.time()
        while robot.speed() > STILL and robot.time() - t0 < 10.0:  # brake: command zero at once
            step(env, robot, 0.0)
        t_stop = robot.time() - t0
        distance = float(np.linalg.norm(np.array(robot.pose()[:2]) - start))
        decel = reached / t_stop if t_stop > 0 else float("inf")
        decels.append(decel)
        print(
            f"commanded {target:.1f} m/s: walks at {reached:.2f} m/s; "
            f"stops in {t_stop:.2f} s over {distance:.2f} m (mean {decel:.2f} m/s^2)",
            flush=True,
        )
    print(f"smallest measured deceleration {min(decels):.2f} m/s^2 (g1.planner.MAX_DECEL is {MAX_DECEL})")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
