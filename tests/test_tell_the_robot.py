"""The tell-the-robot skills, with the stand-in robot (tests/fakes.py: FakeRoom).

Each test is what a language model would ask for, and checks the result it
would read: done, or refused with the reason. No physics: that is for the
Isaac Lab runs.
"""

import math

import numpy as np
import pytest
from fakes import FakeRoom

from tasks.tell_the_robot import layout, skills
from tasks.tell_the_robot.layout import OBJECTS, PLACE_MARK, TABLE_A, TABLE_SIZE, Thing
from tasks.tell_the_robot.memory import Memory, in_view

TABLE_A_EDGE = TABLE_A[0] - TABLE_SIZE[0] / 2


def room(things=OBJECTS):
    robot = FakeRoom(things, [layout.box_of(t) for t in layout.TABLES], TABLE_SIZE[2], start=layout.START)
    body = skills.Body(robot, Memory(things))
    return robot, body


def run(robot, skill, limit=200.0):
    start = robot.time()
    while not skill.done and robot.time() - start < limit:
        skill.update()
        robot.step()
    assert skill.done, f"{skill.name} did not finish in {limit:.0f} s (at: {skill.label})"
    print(f"\n{skill.name}: {skill.result.message}")
    return skill.result


def off(robot, name, xy):
    p = robot.objects[name]
    return math.hypot(p[0] - xy[0], p[1] - xy[1])


def test_the_spoken_example_puts_the_bottle_on_the_mark_and_skips_table_b():
    robot, body = room()
    look = run(robot, skills.LookAround(body, "blue bottle"))
    assert look.ok and "not checked: table_b" in look.message
    assert "green" not in look.message  # the green box is on table B
    bottle = body.memory.find("blue bottle")[0]
    assert run(robot, skills.Pick(body, bottle.id)).ok
    placed = run(robot, skills.Place(body, "mark"))
    assert placed.ok
    assert off(robot, "blue_bottle", PLACE_MARK) < skills.ON_SPOT
    assert robot.objects["blue_bottle"][2] == pytest.approx(TABLE_SIZE[2] + 0.075)
    assert robot.holding is None and body.memory.held is None


def test_looking_around_without_find_sees_both_tables():
    robot, body = room()
    look = run(robot, skills.LookAround(body))
    assert look.ok and "not checked" not in look.message
    assert len(body.memory.objects) == len(OBJECTS)
    assert body.memory.unseen == set()


def test_an_object_out_of_reach_is_refused_with_the_distances():
    deep = Thing("blue_bottle", "blue", "bottle", (TABLE_A_EDGE + 0.30, 0.0))
    robot, body = room((deep,))
    run(robot, skills.LookAround(body))
    before = robot.pose()
    result = run(robot, skills.Pick(body, 1))
    assert not result.ok
    assert "can't reach #1 blue bottle: 30 cm from the edge of table_a" in result.message
    assert f"the hand reaches {100 * body.max_depth:.0f} cm" in result.message
    assert robot.pose() == before  # refused before walking


def test_the_reach_limit_comes_from_the_arm_model():
    _, body = room()
    assert 0.15 < body.max_depth < 0.30
    assert body.clearance == pytest.approx(body.reach[0] - skills.STANCE_DEPTH)


def test_an_object_where_the_hand_comes_in_is_named():
    target = Thing("blue_bottle", "blue", "bottle", (TABLE_A_EDGE + 0.16, 0.0))
    in_front = Thing("red_can", "red", "can", (TABLE_A_EDGE + 0.06, 0.0))
    robot, body = room((target, in_front))
    run(robot, skills.LookAround(body))
    bottle = body.memory.find("blue bottle")[0]
    result = run(robot, skills.Pick(body, bottle.id))
    assert not result.ok and "red can is where the hand comes in" in result.message


def test_a_spot_on_a_table_the_survey_skipped_is_checked_on_arrival():
    on_the_mark = Thing("red_can", "red", "can", PLACE_MARK)
    robot, body = room((OBJECTS[0], on_the_mark))
    run(robot, skills.LookAround(body, "blue bottle"))
    assert "can" not in " ".join(o.kind for o in body.memory.objects.values())  # not seen yet
    run(robot, skills.Pick(body, 1))
    result = run(robot, skills.Place(body, "mark"))
    assert not result.ok
    assert "the spot is not free: #2 red can" in result.message
    assert result.message.endswith("I'm still holding #1 blue bottle")
    assert robot.holding == "blue_bottle"
    assert off(robot, "red_can", PLACE_MARK) == 0.0  # nothing was put on it


def test_next_to_keeps_its_distance_and_stays_on_the_table():
    robot, body = room()
    run(robot, skills.LookAround(body))
    run(robot, skills.Pick(body, body.memory.find("blue bottle")[0].id))
    can = body.memory.find("red can")[0]
    place = skills.Place(body, ("next_to", can.id))
    result = run(robot, place)
    assert result.ok and "next to #2 red can" in result.message
    assert off(robot, "blue_bottle", place.spot) < skills.ON_SPOT
    assert off(robot, "blue_bottle", robot.objects["red_can"][:2]) >= skills.NEXT_TO[0] - 1e-6
    assert layout.table_under(robot.objects["blue_bottle"]) == "table_a"


def test_an_object_moved_after_looking_is_noticed():
    robot, body = room()
    run(robot, skills.LookAround(body, "blue bottle"))
    robot.objects["blue_bottle"][1] += 0.10  # someone moved it
    result = run(robot, skills.Pick(body, 1))
    assert not result.ok and "#1 blue bottle is not where I saw it: it moved 10 cm" in result.message
    assert robot.holding is None


def test_asking_for_the_impossible_says_why():
    robot, body = room()
    assert run(robot, skills.Pick(body, 9)).message == "I don't know object #9; look around first"
    assert run(robot, skills.Place(body, "mark")).message == "my hand is empty; pick something first"
    assert "unknown place 'kitchen'" in run(robot, skills.GoTo(body, "kitchen")).message


def test_go_to_stands_at_the_place_facing_its_table():
    robot, body = room()
    assert run(robot, skills.GoTo(body, "table_b")).ok
    x, y, heading = robot.pose()
    gx, gy, gh = layout.PLACES["table_b"]
    assert math.hypot(x - gx, y - gy) < 0.05
    assert abs(math.atan2(math.sin(heading - gh), math.cos(heading - gh))) < 0.05


def test_the_camera_sees_ahead_not_behind():
    pose = (0.0, 0.0, 0.0)
    assert in_view(pose, (1.0, 0.3))
    assert not in_view(pose, (-1.0, 0.0))
    assert not in_view(pose, (0.5, 1.0))  # 63 degrees to the left
    assert not in_view(pose, (3.0, 0.0))  # too far for the detector
    assert in_view((0.6, -0.7, -np.pi / 2), (0.6, -1.78))  # table B's viewpoint sees the mark
