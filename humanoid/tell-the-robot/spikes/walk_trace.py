"""Where the robot walks during look around, pick and place, every 0.5 s: for walks that overshoot.

The original room (or a seed), the live demo's sequence. Prints per sample: time, step,
pelvis (x, y, heading), the waypoint it walks to, the walk command, and the distance left.

    humanoid/tell-the-robot/light.sh uv run --no-sync python humanoid/tell-the-robot/spikes/walk_trace.py [seed]

FIND="" surveys every viewpoint (look_around without find), the default stops at the blue bottle.
"""

import os
import sys
from pathlib import Path

import numpy as np

LESSON = Path(__file__).resolve().parents[2] / "03-g1-pick-place"
sys.path.insert(0, str(LESSON / "tests"))
sys.path.insert(0, str(LESSON))

from harness import Bench  # noqa: E402
from g1 import scenes, skills  # noqa: E402

EVERY = 0.5  # s


def traced(skill):
    update, last = skill.update, [-1.0]

    def update_and_print(r):
        cmd_vel, command = update(r)
        if r.time - last[0] >= EVERY and 0 <= skill.index < len(skill.steps) and skill.steps[skill.index][0] == "walk":
            last[0] = r.time
            goal = skill.route[0] if skill.route else None
            left = "" if goal is None else f"  to ({goal[0]:+.2f}, {goal[1]:+.2f}, {np.degrees(goal[2]):+4.0f}°){' stop' if len(goal) == 4 else ''}  left {100 * np.hypot(r.position[0] - goal[0], r.position[1] - goal[1]):5.1f} cm"
            print(f"    {r.time:6.1f}  {skill.label:28.28}  at ({r.position[0]:+.2f}, {r.position[1]:+.2f}, {np.degrees(r.heading):+4.0f}°)  cmd ({cmd_vel[0]:+.2f}, {cmd_vel[1]:+.2f}, {cmd_vel[2]:+.2f}){left}", flush=True)
        return cmd_vel, command

    skill.update = update_and_print
    return skill


seed = int(sys.argv[1]) if len(sys.argv) > 1 else None
bench = Bench(seed)
print(bench.run(traced(skills.LookAround(os.environ.get("FIND", "blue bottle")))).message)
bottle = bench.find("blue", "bottle")
print("bottle in memory:", np.round(bench.robot.object(bottle)["position"], 3).tolist(), " truth:", np.round(bench.truth("bottle"), 3).tolist())
pick = traced(skills.Pick(bottle))
result = bench.run(pick)
if hasattr(pick, "option"):
    print("stand pose:", np.round(pick.option.stand, 3).tolist())
if result.ok:
    result = bench.run(traced(skills.Place("mark")))
    true = bench.truth("bottle")
    print(f"truth: the bottle stands {100 * np.hypot(*(true[:2] - np.array(scenes.PLACE_MARK))):.1f} cm from the mark, at height {true[2]:.3f} m")
