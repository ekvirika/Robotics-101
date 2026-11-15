"""The memory's rules (g1/world.py), without a simulator.

  uv run --with pytest pytest tests/test_world.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1.objects import Seen  # noqa: E402
from g1.world import MISSES_TO_FORGET, World  # noqa: E402


def seen(x, y, color="blue", kind="bottle"):
    return Seen(kind, color, np.array([x, y, 0.875]), 500, (0, 0, 1, 1))


def always(_):
    return True


def never(_):
    return False


def test_new_objects_get_new_ids():
    world = World()
    world.update(0.0, [seen(1.4, 0.0), seen(1.6, 0.2, "red")], always)
    assert sorted(o["id"] for o in world.as_list()) == [1, 2]


def test_same_object_keeps_its_id():
    world = World()
    world.update(0.0, [seen(1.4, 0.0)], always)
    world.update(0.3, [seen(1.42, 0.01)], always)
    assert [o["id"] for o in world.as_list()] == [1]
    assert world.objects[1].sightings == 2


def test_different_color_is_a_different_object():
    world = World()
    world.update(0.0, [seen(1.4, 0.0, "blue")], always)
    world.update(0.3, [seen(1.41, 0.0, "red")], always)
    assert len(world.objects) == 2


def test_closest_pair_wins():
    world = World()
    world.update(0.0, [seen(1.40, 0.0), seen(1.47, 0.0)], always)
    world.update(0.3, [seen(1.46, 0.0), seen(1.41, 0.0)], always)
    assert np.isclose(world.objects[1].position[0], 1.40 + 0.3 * 0.01)
    assert np.isclose(world.objects[2].position[0], 1.47 - 0.3 * 0.01)


def test_kind_is_a_vote():
    world = World()
    for t, kind in enumerate(["bottle", "can", "bottle", None]):
        world.update(float(t), [seen(1.4, 0.0, kind=kind)], always)
    assert world.objects[1].kind == "bottle"


def test_removed_object_is_forgotten_when_it_should_be_seen():
    world = World()
    world.update(0.0, [seen(1.4, 0.0)], always)
    for t in range(1, MISSES_TO_FORGET + 1):
        world.update(float(t), [], always)
    assert world.objects == {}


def test_out_of_view_or_hidden_object_is_kept():
    world = World()
    world.update(0.0, [seen(1.4, 0.0)], always)
    for t in range(1, 20):
        world.update(float(t), [], never)
    assert list(world.objects) == [1]
