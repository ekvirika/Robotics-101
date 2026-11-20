"""Load the tell-the-robot scene in Isaac Lab, let it stand for a few seconds
with the robot still, and report where every object is. PASS = every object
still stands on its table, within 1 cm of where layout.py puts it.

    $ISAACLAB/isaaclab.sh -p tasks/tell_the_robot/check_scene.py --headless
"""

import argparse
import math
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Check the tell-the-robot scene.")
parser.add_argument("--seconds", type=float, default=3.0)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")  # g1.isaac.DEVICE: one robot simulates faster on the CPU
args = parser.parse_args()
app = AppLauncher(args).app

# ---- from here on Isaac Sim is running ----
import gymnasium as gym  # noqa: E402
from g1.isaac import IsaacG1  # noqa: E402

from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

from tasks.tell_the_robot import ENV_ID  # noqa: E402
from tasks.tell_the_robot.layout import OBJECTS, TABLE_SIZE, table_under  # noqa: E402

DRIFT = 0.01  # m: further from its start than this, an object moved on its own


def main() -> bool:
    cfg = parse_env_cfg(ENV_ID, device=args.device, num_envs=1)
    env = gym.make(ENV_ID, cfg=cfg).unwrapped
    env.reset()
    robot = IsaacG1(env, 0)
    while robot.time() < args.seconds and app.is_running():
        robot.step()
    ok = True
    print(f"after {robot.time():.1f} s, robot at {tuple(round(v, 2) for v in robot.pose())}")
    for thing in OBJECTS:
        position, _ = robot.find(thing.name)
        drift = math.hypot(position[0] - thing.xy[0], position[1] - thing.xy[1])
        bottom = position[2] - thing.height / 2
        standing = abs(bottom - TABLE_SIZE[2]) < DRIFT and table_under(position) is not None
        good = standing and drift < DRIFT
        ok &= good
        print(
            f"{thing.name:<14} at ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}), "
            f"moved {100 * drift:.1f} cm, bottom {100 * (bottom - TABLE_SIZE[2]):+.1f} cm from the top "
            f"{'ok' if good else 'MOVED OR FELL'}"
        )
    print("PASS" if ok else "FAIL", flush=True)
    env.close()
    return ok


if __name__ == "__main__":
    ok = main()
    app.close()
    sys.exit(0 if ok else 1)
