"""look_around(find) skips table B: does placing on the mark still avoid what it never surveyed?

Rooms with an object 15 to 20 cm from the mark. Look around for the blue bottle (stops at
table A), pick it, put it on the mark. place looks at the spot on arrival. Checked against
the simulator: where the bottle stands, and whether any other object moved.

    humanoid/tell-the-robot/light.sh uv run --no-sync python humanoid/tell-the-robot/spikes/find_eval.py [seed ...]
"""

import sys
from pathlib import Path

import numpy as np

LESSON = Path(__file__).resolve().parents[2] / "03-g1-pick-place"
sys.path.insert(0, str(LESSON / "tests"))
sys.path.insert(0, str(LESSON))

from harness import Bench  # noqa: E402
from g1 import scenes, skills  # noqa: E402

SEEDS = [int(s) for s in sys.argv[1:]] or [5, 10, 18, 20, 36]
MOVED = 0.01  # m


def room(seed):
    bench = Bench(seed)
    names = [o.name for o in bench.scene.objects if o.name != "bottle"]
    print(f"\n== room {seed}: " + ", ".join(o.label for o in bench.scene.objects), flush=True)
    start = bench.sim.time
    look = bench.run(skills.LookAround("blue bottle"), verbose=False)
    print(f"   look_around: {look.message} ({bench.sim.time - start:.0f} s)", flush=True)
    pick = bench.run(skills.Pick(bench.find("blue", "bottle")), verbose=False)
    if not pick.ok:
        print(f"   pick FAILED: {pick.message}", flush=True)
        return False
    before = {n: bench.truth(n) for n in names}
    place = bench.run(skills.Place("mark"), verbose=False)
    print(f"   place: {'OK' if place.ok else 'FAILED'}: {place.message}", flush=True)
    moved = [f"{n} {100 * np.hypot(*(bench.truth(n)[:2] - p[:2])):.0f} cm" for n, p in before.items() if np.hypot(*(bench.truth(n)[:2] - p[:2])) > MOVED]
    true = bench.truth("bottle")
    print(f"   truth: bottle {100 * np.hypot(*(true[:2] - np.array(scenes.PLACE_MARK))):.1f} cm from the mark; moved: {', '.join(moved) or 'nothing'}; {bench.sim.time - start:.0f} s in all", flush=True)
    return place.ok and not moved


ok = [room(s) for s in SEEDS]
print(f"\n{sum(ok)}/{len(ok)} rooms: placed on the mark, nothing else moved")
