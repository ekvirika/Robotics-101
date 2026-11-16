"""Draws media/square.jpg: one moment of a crossing, seen from above the square.

  uv run python tools/make_picture.py

It runs the crossing with crowd 0 (like the test) and takes the picture 12 s
in. The red rings are the robot's tracks: where it thinks people are.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

LESSON = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(LESSON), str(LESSON / "tests")]

import test_crossing  # noqa: E402

PICTURE_TIME = 12.0  # s
RING = np.array([0.9, 0.1, 0.1, 0.6])


def main():
    record = []
    world = test_crossing.run_crossing(
        0, verbose=False, record=record, stop_at=PICTURE_TIME
    )
    tracks = record[-1][3]

    renderer = mujoco.Renderer(world.model, 720, 1280)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.0, 0.0, 0.5]
    camera.distance, camera.azimuth, camera.elevation = 15, 90, -45
    renderer.update_scene(world.data, camera)
    scene = renderer.scene
    for x, y, _, _ in tracks:
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [0.35, 0.004, 0],
            [x, y, 0.006],
            np.eye(3).ravel(),
            RING,
        )
        scene.ngeom += 1
    (LESSON / "media").mkdir(exist_ok=True)
    Image.fromarray(renderer.render()).save(LESSON / "media/square.jpg", quality=88)
    print(
        f"wrote media/square.jpg (t = {world.time:.1f} s, {len(tracks)} people tracked)"
    )


if __name__ == "__main__":
    main()
