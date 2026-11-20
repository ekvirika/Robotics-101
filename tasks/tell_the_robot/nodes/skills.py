"""dora node: the tell-the-robot skills (tasks/tell_the_robot/skills.py) on the
robot behind the sim node's messages (g1.dora.DoraG1), one skill at a time.

Inputs:  base_pose, robot_state, scan, object_pose (every object)   from sim
         command   JSON text, one skill, from agent:
                   {"skill": "look_around", "find": "blue bottle"}   ("" looks at everything)
                   {"skill": "go_to", "place": "table_a"}
                   {"skill": "pick", "object": 1}                    (or "label": "blue bottle")
                   {"skill": "place", "target": "mark"}              (or {"next_to": 2})
                   {"skill": "stop"}                                 stops at once
Outputs: cmd_vel, upper_body, hands                                  to sim
         result    JSON text when a skill ends: {"skill", "ok", "message"}
         objects   JSON text, the memory: id, color, kind, position, table, in_hand, and
                   can_pick ("" or why not, from the same checks pick makes)
         held      [object id], or [-1] when the hand is empty
         status    the step running now, e.g. "pick: hand around it"
"""

import json

import pyarrow as pa
from dora import Node
from g1.dora import DoraG1

from tasks.tell_the_robot import skills
from tasks.tell_the_robot.layout import OBJECTS, table_under
from tasks.tell_the_robot.memory import Memory

robot = DoraG1(Node(), objects=[t.name for t in OBJECTS])
memory = Memory(OBJECTS)
body = skills.Body(robot, memory)
skill, label, sent = None, "", None


def send(output, value):
    robot.node.send_output(output, pa.array(value if isinstance(value, list) else [value]))


def report_memory():
    """Send the memory and the hand, when they changed."""
    global sent
    objects = []
    for o in memory.as_list():
        remembered = memory.by_id(o["id"])
        problem = None if o["in_hand"] else skills.pick_problem(body, remembered)
        objects.append(o | {"table": table_under(remembered.position), "can_pick": problem or ""})
    held = memory.objects[memory.held].id if memory.held else -1
    now = (json.dumps(objects), held)
    if now != sent:
        send("objects", now[0])
        send("held", [float(held)])
        sent = now


def finish(name, ok, message):
    robot.node.log_info(f"{name} {'OK' if ok else 'FAILED'}: {message}")
    send("result", json.dumps({"skill": name, "ok": ok, "message": message}))
    report_memory()


def make(command):
    """A JSON command -> a skill, or the reason it can't start."""
    name = command.get("skill")
    if name == "look_around":
        return skills.LookAround(body, command.get("find", ""))
    if name == "go_to":
        return skills.GoTo(body, command.get("place", ""))
    if name == "pick":
        object_id = command.get("object")
        if object_id is None and "label" in command:
            found = memory.find(command["label"])
            if not found:
                return f"I don't remember a {command['label']}; look around first"
            object_id = found[0].id
        return skills.Pick(body, int(object_id))
    if name == "place":
        target = command.get("target")
        if isinstance(target, dict) and "next_to" in target:
            target = ("next_to", int(target["next_to"]))
        return skills.Place(body, target)
    return f"unknown skill {name!r}"


def hear(command):
    global skill
    name = command.get("skill", "?")
    if name == "stop":
        if skill is not None:
            finish(skill.name, False, f"stopped by a person during: {skill.label}")
        skill, body.walking = None, None
        return
    if skill is not None:
        finish(name, False, f"busy with {skill.name}; wait for its result or send stop")
        return
    made = make(command)
    if isinstance(made, str):
        finish(name, False, made)
    else:
        skill = made


memory.look(robot)
report_memory()
while True:
    for event in robot.inbox:
        if event["id"] == "command":
            hear(json.loads(event["value"][0].as_py()))
    robot.inbox.clear()
    if skill is None:
        body.update()
    else:
        skill.update()
        if skill.label != label:
            label = skill.label
            send("status", f"{skill.name}: {label}")
            robot.node.log_info(f"{skill.name}: {label}  (robot at {tuple(round(v, 2) for v in robot.pose())})")
        if skill.done:
            finish(skill.name, skill.result.ok, skill.result.message)
            skill, label = None, ""
    robot.step()
