"""The robot's memory of objects: what it has seen, where, and when.

Until the head camera and the detector are ported, sightings come from
Perception.find (the simulator knows where everything is), but only for objects
the head camera could see from where the robot stands: in front of it, inside
its horizontal field of view, closer than MAX_RANGE. So looking around still
matters, and the language model only knows what the robot has seen.

Ids are given in the order objects are first seen and never change, so "#2"
means the same object to the skills and to the language model.

The object in the hand moves with the hand: every look updates it, wherever it is.
"""

import math
from dataclasses import dataclass

CAMERA_HFOV = math.radians(69)  # the G1's head camera (RealSense D435i, color): 69 x 42 degrees
MAX_RANGE = 2.5  # m: further away, the detector would miss small objects


@dataclass
class Remembered:
    id: int
    name: str
    color: str
    kind: str
    position: tuple
    last_seen: float

    @property
    def label(self):
        return f"#{self.id} {self.color} {self.kind}"

    def as_dict(self):
        return {
            "id": self.id,
            "color": self.color,
            "kind": self.kind,
            "position": [round(v, 3) for v in self.position],
            "last_seen": round(self.last_seen, 1),
        }


def in_view(pose, point, hfov=CAMERA_HFOV, max_range=MAX_RANGE):
    """True if the head camera at pose (x, y, heading) sees point (x, y, ...)."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance > max_range or distance < 1e-6:
        return False
    off = math.atan2(dy, dx) - pose[2]
    off = math.atan2(math.sin(off), math.cos(off))
    return abs(off) <= hfov / 2


def matches(remembered, text):
    """Every word of `text` ("blue bottle", "#3", "can") names this object."""
    names = (remembered.color, remembered.kind, f"#{remembered.id}", str(remembered.id))
    return all(word in names for word in text.lower().split())


class Memory:
    def __init__(self, things):
        """things: what the simulator reports, layout.Thing (name, color, kind)."""
        self.things = {t.name: t for t in things}
        self.objects = {}  # name -> Remembered
        self.held = None  # name of the object in the hand
        self.unseen = set()  # tables the last look_around skipped

    def look(self, robot):
        """One look with the head camera: remember or update every object in view."""
        pose, now = robot.pose(), robot.time()
        seen = []
        for name, thing in self.things.items():
            position, _ = robot.find(name)
            if name != self.held and not in_view(pose, position):
                continue
            known = self.objects.get(name)
            if known is None:
                known = Remembered(len(self.objects) + 1, name, thing.color, thing.kind, tuple(position), now)
                self.objects[name] = known
            known.position, known.last_seen = tuple(float(v) for v in position), now
            seen.append(known)
        return seen

    def by_id(self, object_id):
        return next((o for o in self.objects.values() if o.id == object_id), None)

    def find(self, text):
        return [o for o in self.objects.values() if matches(o, text)]

    def others(self, but=None):
        """Every remembered object except `but` (a name) and the one in the hand."""
        return [o for o in self.objects.values() if o.name not in (but, self.held)]

    def as_list(self):
        return [
            o.as_dict() | {"in_hand": o.name == self.held} for o in sorted(self.objects.values(), key=lambda o: o.id)
        ]
