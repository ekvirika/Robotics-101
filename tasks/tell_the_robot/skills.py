"""What the robot can do when it is told in words: four skills a language model
calls as tools, each ending in an honest result it can read.

    look_around(find)   walk to each table and look; with find ("blue bottle"),
                        stop as soon as a matching object is in the memory
    go_to(place)        "start", "table_a", "table_b", "mark"
    pick(object_id)     walk to the object's table, grasp it, lift it
    place(target)       "table_a", "table_b", "mark", or ("next_to", object_id)

Walking and the arm are the shared skills, used as tasks/pick_and_place/mission.py
uses them: g1.navigator walks to 0.8 m before a spot and docks; g1.manipulation
moves the arm by inverse kinematics on Unitree's model; where to stand comes
from the hand's geometry (g1.kinematics). What these skills add:

  before   checks that say why not, in words: the object isn't known, it is
           out of the hand's reach (measured on the arm model, not guessed),
           another object is where the hand comes in, no free spot
  during   the object is still there when the robot arrives; the hand holds it
           after the lift; a spot on a table the last look skipped is looked at
           before anything is put on it
  after    the object stands where it was put; neighbors that moved are named

A skill is a list of steps (label, start, finished), like the pick-and-place
mission. start() returns None, or the reason the skill fails.

    body = Body(robot, memory)          # one per robot: walking, arm, hands
    skill = Pick(body, 1)
    while not skill.done:
        skill.update(); robot.step()
    skill.result                        # Result(ok, message)
"""

import math
from dataclasses import dataclass

import numpy as np
from g1.kinematics import ArmIK, grasp_point
from g1.manipulation import Manipulator
from g1.navigator import Navigator
from g1.tracker import SCAN_DT

from .layout import PLACE_MARK, PLACES, TABLE_SIZE, TABLES, VIEWPOINTS, box_of, depth_on, table_under
from .memory import matches

SIDE = "right"  # the hand that grasps
APPROACH = 0.8  # m: walk to this far before a working spot, then dock slowly (as pick and place)
PRE_GRASP = 0.12  # m: the open hand waits this far behind the object, then moves in
LIFT = 0.10  # m: lift the object this high off the table
WALK_ABOVE = 0.05  # m: while walking, empty hands wait this far above grasp height
PLACE_ABOVE = 0.01  # m: let go this far above the table
STANCE_DEPTH = 0.08  # m: pick and place's stance, an object this far in: the pelvis never goes nearer the edge
LOOK_TIME = 1.0  # s: stand still and look
STEP_TIMEOUT = 60.0  # s
MOVED = 0.05  # m: an object this far from where it was remembered has been moved
ON_SPOT = 0.03  # m: put down within this of the spot counts as there
DISTURBED = 0.03  # m: a neighbor that moved more than this was pushed
NEIGHBORHOOD = 0.3  # m: objects this close to a spot are watched for being pushed
PLACE_DEPTH = 0.08  # m from the working edge, where objects are put on a table
PLACE_GAP = 0.13  # m: a put-down object keeps this distance to every other object
NEXT_TO = (0.13, 0.16)  # m between centers for "next to": two rings
EDGE_MARGIN = 0.05  # m: spots stay this far from a table's side edges
OBJECT_RADIUS = 0.035  # m: the widest object
HAND_WAY = ((-PRE_GRASP - 0.04, 0.05), (-0.07, 0.07))  # m around the object, (ahead, left) in the robot's
# frame: where the hand passes coming in from behind and closing. A first estimate from PRE_GRASP and the
# Dex3's width; to be measured in Isaac Lab (in MuJoCo the fingers reached 10 cm past the grasp point).
ATTEMPTS = 3  # places try up to this many spots before giving up


@dataclass
class Result:
    ok: bool
    message: str


def to_frame(pose, point):
    """World (x, y) -> (ahead, left) of a robot standing at pose (x, y, heading)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    return c * dx + s * dy, -s * dx + c * dy


def in_hand_way(stance, spot, others):
    """The remembered object in the way of the hand working at `spot` from `stance`, or None."""
    (ahead_min, ahead_max), (left_min, left_max) = HAND_WAY
    spot_ahead, spot_left = to_frame(stance, spot)
    for other in others:
        ahead, left = to_frame(stance, other.position)
        ahead, left = ahead - spot_ahead, left - spot_left
        if (
            ahead_min - OBJECT_RADIUS < ahead < ahead_max + OBJECT_RADIUS
            and left_min - OBJECT_RADIUS < left < left_max + OBJECT_RADIUS
        ):
            return other
    return None


def free(spot, others, gap=PLACE_GAP):
    return all(math.hypot(o.position[0] - spot[0], o.position[1] - spot[1]) >= gap for o in others)


class Body:
    """The robot's walking, arm and hands, shared by every skill (one per robot)."""

    def __init__(self, robot, memory):
        self.robot, self.memory = robot, memory
        self.navigator = Navigator(static_map=[box_of(t) for t in TABLES])
        self.hands = Manipulator(robot)
        self.ik = ArmIK(SIDE)
        palm, self.palm_rotation = self.ik.palm_pose(np.zeros(7))
        self.grasp_offset = grasp_point(SIDE)
        self.reach = palm + self.palm_rotation @ self.grasp_offset  # the object's center, pelvis frame, arm relaxed
        self.clearance = self.reach[0] - STANCE_DEPTH  # pelvis to the working edge, never less
        self.walk_pose = {}
        for side in ("left", "right"):
            side_palm, rotation = ArmIK(side).palm_pose(np.zeros(7))
            self.walk_pose[side] = (side_palm + np.array([-PRE_GRASP, 0.0, WALK_ABOVE]), rotation)
        self.next_scan = robot.time()
        self.walking = None
        self.dock_spot = None
        self.max_depth = self._deepest_reach()

    # ---- every control step ----

    def update(self):
        self._drive()
        self.hands.update()

    # ---- walking ----

    def walk_to(self, spot, approach=APPROACH):
        x, y, h = spot
        self.dock_spot = spot
        self.navigator.set_goal((x - approach * math.cos(h), y - approach * math.sin(h)))
        self.walking = "approach"

    def walked(self):
        return self.walking is None and self.robot.speed() < 0.05

    def _drive(self):
        if self.walking is None:
            self.robot.move(0.0, 0.0, 0.0)
            return
        pose = self.robot.pose()
        scan = None
        if self.robot.time() >= self.next_scan - 1e-9:
            self.next_scan += SCAN_DT
            scan = self.robot.scan()
        if self.walking == "approach":
            cmd = self.navigator.update(pose, scan, self.robot.dt)
            if self.navigator.arrived:
                self.walking = "dock"
        else:
            if scan is not None:
                self.navigator.update(pose, scan, self.robot.dt)
            cmd, docked = self.navigator.dock(pose, self.dock_spot, self.robot.dt)
            if docked:
                self.walking = None
                cmd = (0.0, 0.0, 0.0)
        self.robot.move(*cmd)

    # ---- where to stand, what the hand reaches ----

    def stance(self, xy, table):
        """Where the pelvis stands to work at (x, y) on `table`: the object where the
        relaxed arm holds it, but never nearer the edge than the pick-and-place stance."""
        _, heading = TABLES[table]
        ahead = max(self.reach[0], self.clearance + depth_on(table, xy))
        c, s = math.cos(heading), math.sin(heading)
        left = self.reach[1]
        return (xy[0] - (c * ahead - s * left), xy[1] - (s * ahead + c * left), heading)

    def reachable(self, center, stance):
        """The hand gets behind, around and above `center` (world) from `stance` (arm model)."""
        for back, up in ((PRE_GRASP, 0.0), (0.0, 0.0), (0.0, LIFT)):
            palm = self._palm_for(center, stance, back, up)
            if not self.ik.solve(palm, self.palm_rotation)[1]:
                return False
        return True

    def hand_to(self, center, back=0.0, up=0.0):
        """Move the grasp point to `center` (world), `back` behind it, `up` above it."""
        return self.hands.reach(SIDE, self._palm_for(center, self.robot.pose(), back, up), self.palm_rotation)

    def raise_hands(self):
        """Hands up for walking; a hand holding something stays where it is."""
        goals = {side: pose for side, pose in self.walk_pose.items() if not (side == SIDE and self.memory.held)}
        return self.hands.reach_all(goals)

    def still(self):
        return not self.hands.busy()

    def _palm_for(self, center, pose, back, up):
        ahead, left = to_frame(pose, center)
        point = np.array([ahead - back, left, center[2] - self.robot.pelvis_height() + up])
        return point - self.palm_rotation @ self.grasp_offset

    def _deepest_reach(self):
        """How far from the working edge the hand reaches (m), probed on the arm model."""
        center_x, _ = TABLES["table_a"][0]
        deepest = 0.0
        for depth in np.arange(0.02, 0.40, 0.01):
            xy = (center_x - TABLE_SIZE[0] / 2 + depth, TABLES["table_a"][0][1])
            center = (*xy, TABLE_SIZE[2] + 0.075)
            if not self.reachable(center, self.stance(xy, "table_a")):
                break
            deepest = float(depth)
        return deepest


class Skill:
    name = "skill"

    def __init__(self, body):
        self.body, self.robot, self.memory = body, body.robot, body.memory
        self.steps = None
        self.index = 0
        self.started = False
        self.step_start = 0.0
        self.label = ""
        self.result = None
        self.failure = None  # set while backing away after a failure

    @property
    def done(self):
        return self.result is not None

    def plan(self):
        """The steps, or a Result if the skill can't even start."""
        raise NotImplementedError

    def success(self):
        return Result(True, "done")

    def try_other(self, reason):
        """New steps to try another way after `reason`, or None to give up."""
        return

    def retreat_steps(self):
        """Steps that back away safely before a failure is reported (hand open, hand back)."""
        return []

    def failure_message(self, reason):
        return reason

    def update(self):
        """One control step."""
        if self.done:
            return
        if self.steps is None:
            planned = self.plan()
            if isinstance(planned, Result):
                self.result = planned
                return
            self.steps = planned
        self.body.update()
        if self.index >= len(self.steps):
            self._finish()
            return
        label, start, finished = self.steps[self.index]
        if not self.started:
            self.label, self.step_start, self.started = label, self.robot.time(), True
            reason = start()
            if reason:
                self._fail(reason)
                return
        if self.robot.time() - self.step_start > STEP_TIMEOUT:
            self._fail(f"{label}: took longer than {STEP_TIMEOUT:.0f} s")
            return
        if finished():
            self.index += 1
            self.started = False
            if self.index == len(self.steps):
                self._finish()

    def _finish(self):
        self.result = Result(False, self.failure_message(self.failure)) if self.failure else self.success()

    def _fail(self, reason):
        if self.failure is None:
            other = self.try_other(reason)
            if other is not None:
                self.steps, self.index, self.started = other, 0, False
                return
            back = self.retreat_steps()
            if back:
                self.failure = reason
                self.steps, self.index, self.started = back, 0, False
                return
        self.body.walking = None
        self.result = Result(False, self.failure_message(self.failure or reason))

    # ---- building blocks for steps ----

    def check(self, label, reason_fn):
        return (label, reason_fn, lambda: True)

    def do(self, label, fn):
        return (label, lambda: fn() and None, lambda: True)

    def walk(self, label, spot_fn, approach=APPROACH):
        return (label, lambda: self.body.walk_to(spot_fn(), approach), self.body.walked)

    def arm(self, label, move_fn):
        return (
            label,
            lambda: None if move_fn() is not False else f"{label}: the hand can't reach that",
            self.body.still,
        )

    def look(self, label="look"):
        def finished():
            if self.robot.time() - self.step_start < LOOK_TIME:
                return False
            self.memory.look(self.robot)
            return True

        return (label, lambda: None, finished)


class LookAround(Skill):
    name = "look_around"

    def __init__(self, body, find=""):
        super().__init__(body)
        self.find = find.strip()
        self.found = None
        self.skipped = False

    def plan(self):
        self.memory.unseen = set(VIEWPOINTS)
        steps = [self.arm("raise the hands", self.body.raise_hands)]
        for table, viewpoint in VIEWPOINTS.items():
            steps += [
                self.walk(f"walk to look at {table}", lambda v=viewpoint: v, approach=0.0),
                self.look(f"look at {table}"),
                self.do("remember the table", lambda t=table: self._looked_at(t)),
            ]
        return steps

    def _looked_at(self, table):
        self.memory.unseen.discard(table)
        if not self.find:
            return
        self.found = next((o for o in self.memory.objects.values() if matches(o, self.find)), None)
        if self.found is not None:
            self.skipped = self.index + 1 < len(self.steps)
            del self.steps[self.index + 1 :]

    def success(self):
        known = sorted(self.memory.objects.values(), key=lambda o: o.id)
        count = f"{len(known)} object{'s' if len(known) != 1 else ''}"
        seen = f"I see {count}: " + ", ".join(o.label for o in known) if known else "I see no objects"
        if self.skipped:
            return Result(
                True,
                f"{seen} (found {self.find}, stopped looking; not checked: {', '.join(sorted(self.memory.unseen))})",
            )
        if self.find and self.found is None:
            return Result(True, f"{seen}; no {self.find} among them")
        return Result(True, seen)


class GoTo(Skill):
    name = "go_to"

    def __init__(self, body, place):
        super().__init__(body)
        self.place = place

    def plan(self):
        if self.place not in PLACES:
            return Result(False, f"unknown place {self.place!r}, I know {', '.join(PLACES)}")
        return [
            self.arm("raise the hands", self.body.raise_hands),
            self.walk(f"walk to {self.place}", lambda: PLACES[self.place], approach=0.0),
        ]

    def success(self):
        return Result(True, f"at {self.place}")


class Pick(Skill):
    name = "pick"

    def __init__(self, body, object_id):
        super().__init__(body)
        self.object_id = object_id

    def plan(self):
        if self.memory.held is not None:
            held = self.memory.objects[self.memory.held]
            return Result(False, f"my hand already holds {held.label}; place it first")
        obj = self.memory.by_id(self.object_id)
        if obj is None:
            return Result(False, f"I don't know object #{self.object_id}; look around first")
        self.obj = obj
        table = table_under(obj.position)
        if table is None:
            return Result(False, f"can't pick {obj.label}: it is not on a table")
        self.start_position = np.array(obj.position)
        self.stance = self.body.stance(obj.position[:2], table)
        if not self.body.reachable(obj.position, self.stance):
            depth, deepest = 100 * depth_on(table, obj.position), 100 * self.body.max_depth
            return Result(
                False,
                f"can't reach {obj.label}: {depth:.0f} cm from the edge of {table}, the hand reaches {deepest:.0f} cm",
            )
        blocker = in_hand_way(self.stance, obj.position, self.memory.others(but=obj.name))
        if blocker is not None:
            return Result(False, f"can't pick {obj.label}: {blocker.label} is where the hand comes in")
        center = lambda: np.array(self.memory.objects[obj.name].position)  # noqa: E731
        return [
            self.arm("raise the hands", self.body.raise_hands),
            self.walk(f"walk to {obj.label}", lambda: self.stance),
            self.look("look at it up close"),
            self.check("check it is still there", self._still_there),
            self.arm("open the hand", lambda: self.body.hands.grip(SIDE, closed=False)),
            self.arm("hand behind it", lambda: self.body.hand_to(center(), back=PRE_GRASP)),
            self.arm("hand around it", lambda: self.body.hand_to(center())),
            self.arm("close the hand", lambda: self.body.hands.grip(SIDE, closed=True)),
            self.arm("lift it", lambda: self.body.hand_to(self.start_position, up=LIFT)),
            self.check("check the hand holds it", self._holding),
        ]

    def _still_there(self):
        now = self.memory.objects[self.obj.name]
        moved = math.hypot(now.position[0] - self.start_position[0], now.position[1] - self.start_position[1])
        if moved > MOVED:
            return f"{self.obj.label} is not where I saw it: it moved {100 * moved:.0f} cm"
        self.start_position = np.array(now.position)
        return None

    def _holding(self):
        position, _ = self.robot.find(self.obj.name)
        if position[2] < self.start_position[2] + LIFT / 2:
            return f"my hand closed on nothing: {self.obj.label} is still on the table"
        self.memory.held = self.obj.name
        self.memory.look(self.robot)
        return None

    def retreat_steps(self):
        """The hand was at the table: open it and take it back and up, away from the objects."""
        if self.index < 4 or self.memory.held is not None:
            return []
        return [
            self.arm("open the hand", lambda: self.body.hands.grip(SIDE, closed=False)),
            self.arm("hand back", lambda: self.body.hand_to(self.start_position, back=PRE_GRASP, up=LIFT)),
            self.arm("raise the hands", self.body.raise_hands),
        ]

    def success(self):
        return Result(True, f"holding {self.obj.label}")


class Place(Skill):
    name = "place"

    def __init__(self, body, target):
        super().__init__(body)
        self.target = target  # "table_a" | "table_b" | "mark" | ("next_to", object_id)
        self.attempt = 0

    def plan(self):
        if self.memory.held is None:
            return Result(False, "my hand is empty; pick something first")
        self.held = self.memory.objects[self.memory.held]
        candidates, self.where, reason = self._candidates()
        if reason:
            return Result(False, reason)
        others = self.memory.others()
        self.known = {o.name: o.position for o in others}
        options = []
        for spot, table in candidates:
            if table_under(spot, EDGE_MARGIN) != table or not free(spot, others):
                continue
            stance = self.body.stance(spot, table)
            center = (*spot, TABLE_SIZE[2] + self.held_height() / 2 + PLACE_ABOVE)
            if self.body.reachable(center, stance) and in_hand_way(stance, spot, others) is None:
                options.append((spot, table, stance))
        if not options:
            return Result(False, f"can't place {self.held.label}: no free spot within reach {self.where}")
        here = self.robot.pose()
        self.options = sorted(options, key=lambda o: math.hypot(o[2][0] - here[0], o[2][1] - here[1]))[:ATTEMPTS]
        return self.steps_for(self.options[0])

    def held_height(self):
        return self.memory.things[self.held.name].height

    def _candidates(self):
        """Spots to try (x, y, table), the words for where, and the reason if there are none."""
        if self.target == "mark":
            return [(PLACE_MARK, "table_b")], "on the mark", None
        if self.target in TABLES:
            center, heading = TABLES[self.target]
            c, s = math.cos(heading), math.sin(heading)
            edge = (center[0] - c * TABLE_SIZE[0] / 2, center[1] - s * TABLE_SIZE[0] / 2)
            spots = []
            for offset in np.arange(-0.35, 0.36, 0.05):
                spots.append(
                    ((edge[0] + c * PLACE_DEPTH - s * offset, edge[1] + s * PLACE_DEPTH + c * offset), self.target)
                )
            return spots, f"on {self.target}", None
        if isinstance(self.target, tuple) and self.target[0] == "next_to":
            anchor = self.memory.by_id(self.target[1])
            if anchor is None:
                return [], "", f"I don't know object #{self.target[1]}; look around first"
            table = table_under(anchor.position)
            if table is None:
                return [], "", f"{anchor.label} is not on a table"
            spots = []
            for radius in NEXT_TO:
                for angle in np.radians(np.arange(0, 360, 30)):
                    spots.append(
                        (
                            (
                                anchor.position[0] + radius * math.cos(angle),
                                anchor.position[1] + radius * math.sin(angle),
                            ),
                            table,
                        )
                    )
            return spots, f"next to {anchor.label}", None
        return [], "", f"unknown place {self.target!r}"

    def steps_for(self, option):
        spot, table, stance = option
        self.spot, self.table = spot, table
        center = np.array([*spot, TABLE_SIZE[2] + self.held_height() / 2 + PLACE_ABOVE])
        self.neighbors = {
            o.name: o.position
            for o in self.memory.others()
            if math.hypot(o.position[0] - spot[0], o.position[1] - spot[1]) < NEIGHBORHOOD
        }
        steps = [self.walk(f"carry it, to put it {self.where}", lambda: stance)]
        if table in self.memory.unseen:
            steps += [
                self.look("look at the spot"),
                self.do("remember the table", lambda: self.memory.unseen.discard(table)),
                self.check("check the spot is free", self._spot_free),
            ]
        return steps + [
            self.arm("hold it over the spot", lambda: self.body.hand_to(center, up=LIFT)),
            self.arm("put it down", lambda: self.body.hand_to(center)),
            self.arm("open the hand", lambda: self.body.hands.grip(SIDE, closed=False)),
            self.do("let go", self._let_go),
            self.arm("pull the hand back", lambda: self.body.hand_to(center, back=PRE_GRASP)),
            self.arm("raise the hands", self.body.raise_hands),
            self.look("look where I put it"),
            self.check("check it stands there", self._there),
        ]

    def _spot_free(self):
        """The spot was chosen before this table was seen: only what the memory didn't know then counts."""
        stance = self.options[self.attempt][2]
        for other in self.memory.others():
            before = self.known.get(other.name)
            if (
                before is not None
                and math.hypot(other.position[0] - before[0], other.position[1] - before[1]) <= DISTURBED
            ):
                continue
            if not free(self.spot, [other]):
                return f"the spot is not free: {other.label} stands there"
            if in_hand_way(stance, self.spot, [other]) is not None:
                return f"the spot is not free: {other.label} is in my hand's way"
        return None

    def _let_go(self):
        self.memory.held = None
        self.held.position = (*self.spot, TABLE_SIZE[2] + self.held_height() / 2)

    def _there(self):
        position, _ = self.robot.find(self.held.name)
        off = math.hypot(position[0] - self.spot[0], position[1] - self.spot[1])
        standing_height = TABLE_SIZE[2] + self.held_height() / 2
        if position[2] < TABLE_SIZE[2]:
            return f"{self.held.label} fell off the table"
        if position[2] < standing_height - 0.02:
            return f"{self.held.label} fell over: it lies on its side where I put it down"
        if off > ON_SPOT:
            return f"{self.held.label} is {100 * off:.0f} cm from where I put it"
        return None

    def try_other(self, reason):
        if self.memory.held is None or "not free" not in reason or self.attempt + 1 >= len(self.options):
            return None
        self.attempt += 1
        return self.steps_for(self.options[self.attempt])

    def failure_message(self, reason):
        tries = f" (tried {self.attempt + 1} spots)" if self.attempt else ""
        still = f"; I'm still holding {self.held.label}" if self.memory.held else ""
        return f"{reason}{tries}{still}"

    def success(self):
        moved = []
        for name, before in self.neighbors.items():
            now = self.memory.objects[name]
            shift = math.hypot(now.position[0] - before[0], now.position[1] - before[1])
            if shift > DISTURBED:
                moved.append(f"{now.label} moved {100 * shift:.0f} cm")
        message = f"put {self.held.label} {self.where}"
        return Result(True, message + (", but " + " and ".join(moved) if moved else ""))
