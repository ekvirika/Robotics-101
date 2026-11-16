"""Draw the README's charts from the project's own code and one real simulation run.

  uv run python tools/make_charts.py

It runs the whole mission once without a window (a few seconds), then writes
the charts to media/*.svg and prints the numbers the README quotes. Every
number in the charts comes from the code or from that run. After you change a
number in the code, run this again and the charts match the code again.
"""

import math
import sys
from pathlib import Path
from xml.sax.saxutils import escape

import mujoco
import numpy as np

LESSON = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LESSON))

import tests.test_mission as room  # noqa: E402  (where the mark is, which way to face)
from g1 import messages, navigation, robot  # noqa: E402
from g1.arm import HAND_ACCEL, HAND_SPEED, ArmController, Trapezoid, upright_hand  # noqa: E402
from g1.arm_ik import ArmIK  # noqa: E402
from g1.locomotion import POLICY_DT, Locomotion  # noqa: E402
from g1.mission import APPROACH, GRASP_SPOT, WALK, Mission  # noqa: E402
from g1.simulator import Simulator  # noqa: E402

MEDIA = LESSON / "media"
# Colours: blue / orange / teal stay apart for colour-blind readers too.
BG, INK, BODY, MUTED, GRID = "#F7F6F2", "#11151B", "#3B4351", "#5C6473", "#DCD9D1"
BLUE, ORANGE, TEAL, GREEN = "#2457C5", "#C9641A", "#2A9D8F", "#3FA34D"
GRAVITY = 9.81


class Svg:
    """A tiny helper that writes an SVG picture piece by piece."""

    def __init__(self, width, height, title):
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}" font-family="Helvetica, Arial, sans-serif">',
            f"<title>{escape(title)}</title>",
            "<defs>"
            + "".join(
                f'<marker id="h{c[1:]}" orient="auto" markerWidth="5" markerHeight="5" refX="2" refY="2" '
                f'overflow="visible"><path d="M0 0 L4 2 L0 4 Z" fill="{c}"/></marker>'
                for c in (INK, MUTED, BLUE, ORANGE, TEAL)
            )
            + "</defs>",
            f'<rect width="{width}" height="{height}" fill="{BG}"/>',
        ]

    def text(self, x, y, s, size=16, color=BODY, anchor="start", weight="normal"):
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}" font-weight="{weight}">{escape(s)}</text>'
        )

    def line(self, x1, y1, x2, y2, color=MUTED, width=2, dash=None, arrow=False):
        extra = f' stroke-dasharray="{dash}"' if dash else ""
        extra += f' marker-end="url(#h{color[1:]})"' if arrow else ""
        self.parts.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" '
            f'stroke-width="{width}" stroke-linecap="round"{extra}/>'
        )

    def polyline(self, points, color, width=3, dash=None):
        pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
        extra = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linejoin="round" stroke-linecap="round"{extra}/>'
        )

    def rect(self, x, y, w, h, fill, stroke=None, rx=4, width=2):
        extra = f' stroke="{stroke}" stroke-width="{width}"' if stroke else ""
        self.parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}"{extra}/>'
        )

    def circle(self, cx, cy, r, fill, stroke=None, width=2):
        extra = f' stroke="{stroke}" stroke-width="{width}"' if stroke else ""
        self.parts.append(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}"{extra}/>'
        )

    def raw(self, element):
        self.parts.append(element)

    def save(self, name):
        (MEDIA / name).write_text("\n".join(self.parts + ["</svg>"]) + "\n")
        print("wrote media/" + name)


# ---------------------------------------------------------------- the run


def run_mission():
    """One whole mission without a window, recording what the charts need."""
    sim = Simulator(str(LESSON / "robot/scene.xml"))
    locomotion = Locomotion(str(LESSON / "policies/g1_walk.onnx"))
    arm = ArmController(str(LESSON / "robot/g1_with_hands.xml"))
    mission = Mission(
        room.PICK_HEADING, room.PLACE_POINT, room.PLACE_HEADING, room.TABLE_HEIGHT
    )
    shoulder = robot.JOINTS.index("right_shoulder_pitch_joint")
    model, gdata = arm.ik.model, arm.gravity_data
    joint_id = model.joint("right_shoulder_pitch_joint").id
    body_id = model.body("right_shoulder_pitch_link").id

    rec = {
        "path": [],
        "steps": [],
        "shoulder": [],
        "bottle_start": sim.bottle_pose()[0],
    }
    arm_status = arm_id = None
    next_control = 0.0
    while sim.time < room.TIME_LIMIT and not mission.finished and not mission.failed:
        if sim.time >= next_control - 1e-9:
            next_control += POLICY_DT
            t = sim.time
            q, dq = sim.joint_state()
            quat, gyro = sim.imu()
            base, base_quat = sim.base_pose()
            bottle, _ = sim.bottle_pose()
            cmd_vel, command = mission.update(
                t, base, base_quat, bottle, locomotion.is_standing(t), arm_status
            )
            if 0 <= mission.index < len(mission.steps) and (
                not rec["steps"] or rec["steps"][-1][1] != mission.index
            ):
                rec["steps"].append(
                    (t, mission.index, mission.label, mission.steps[mission.index].kind)
                )
            if command is not None:
                arm_id, kind, position, yaw = messages.unpack_arm_command(command)
                if not arm.execute(t, kind, position, yaw):
                    arm_status, arm_id = (arm_id, False), None
            sim.set_command(
                robot.LOWER_BODY,
                *locomotion.update(
                    t, cmd_vel, q[robot.LEGS], dq[robot.LEGS], quat, gyro
                ),
            )
            sim.set_command(robot.UPPER_BODY, *arm.update(t, q[robot.UPPER_BODY]))
            if arm_id is not None and not arm.busy():
                arm_status, arm_id = (arm_id, True), None
            rec["path"].append((t, base[0], base[1], mission.index))
            # the right shoulder's motor command right now, and where the arm's weight is
            r = gdata.subtree_com[body_id] - gdata.xanchor[joint_id]
            rec["shoulder"].append(
                {
                    "t": t,
                    "label": mission.label,
                    "target": sim.target[shoulder],
                    "angle": q[shoulder],
                    "speed": dq[shoulder],
                    "kp": sim.kp[shoulder],
                    "kd": sim.kd[shoulder],
                    "extra": sim.tau[shoulder],
                    "forward": r[0],
                    "tilt": math.degrees(math.asin(abs(gdata.xaxis[joint_id][2]))),
                    "mass": model.body_subtreemass[body_id],
                }
            )
        sim.step()
    rec["end_time"] = sim.time
    rec["bottle_end"] = sim.bottle_pose()[0]
    rec["finished"] = mission.finished
    return rec, sim


# ---------------------------------------------------------------- charts


def room_map(rec, sim):
    """Top view of the room with the path the robot really walked."""
    s, left, top = 200.0, 40.0, 70.0  # 200 px per meter
    y_left, x_top = (
        0.6,
        2.1,
    )  # world edges shown: y from 0.6 (left) to -2.2, x from 2.1 (top) to -0.3

    def px(x, y):
        return left + (y_left - y) * s, top + (x_top - x) * s

    svg = Svg(840, 640, "The room from above, with the path the robot walked")
    svg.text(
        40,
        40,
        "The room from above, with the path the robot really walked",
        20,
        INK,
        weight="bold",
    )
    model, data = sim.model, sim.data
    for name, fill, stroke, label in (
        ("table_a_top", "#EFD5BA", "#A8521A", "Table A"),
        ("table_b_top", "#E6E3DC", "#8C8577", "Table B"),
    ):
        g = model.geom(name).id
        cx, cy = data.geom_xpos[g][:2]
        hx, hy = model.geom_size[g][:2]
        x0, y0 = px(cx + hx, cy + hy)
        x1, y1 = px(cx - hx, cy - hy)
        svg.rect(x0, y0, x1 - x0, y1 - y0, fill, stroke, rx=6)
        svg.text(x0 + 10, y0 + 24, label, 16, INK, weight="bold")
    bx, by = px(*rec["bottle_start"][:2])
    svg.circle(bx, by, 8, BLUE)
    svg.text(bx + 14, by + 5, "bottle", 15, INK)
    mark = data.site_xpos[model.site("place_target").id]
    mx, my = px(mark[0], mark[1])
    svg.circle(mx, my, 10, GREEN)
    svg.text(mx - 14, my + 5, "mark", 15, INK, anchor="end")
    # the path, in three parts: before the grasp, carrying, after letting go
    close = next(i for _, i, label, _ in rec["steps"] if label == "close the hand")
    release = next(i for _, i, label, _ in rec["steps"] if label == "open the hand")
    for color, keep in (
        (BLUE, lambda i: i <= close),
        (ORANGE, lambda i: close <= i <= release),
        (TEAL, lambda i: i >= release),
    ):
        svg.polyline([px(x, y) for _, x, y, i in rec["path"] if keep(i)], color, 4)
    sx, sy = px(*rec["path"][0][1:3])
    svg.circle(sx, sy, 9, INK)
    svg.text(sx + 16, sy + 5, "start", 15, INK)
    # scale bar and legend
    svg.line(560, 600, 560 + s, 600, MUTED, 3)
    svg.line(560, 592, 560, 608, MUTED, 3)
    svg.line(560 + s, 592, 560 + s, 608, MUTED, 3)
    svg.text(560 + s / 2, 588, "1 m", 15, MUTED, anchor="middle")
    for k, (color, label) in enumerate(
        (
            (BLUE, "walking to the bottle"),
            (ORANGE, "carrying the bottle"),
            (TEAL, "after letting go"),
        )
    ):
        y = 470 + 30 * k
        svg.line(560, y, 600, y, color, 4)
        svg.text(612, y + 5, label, 15, BODY)
    svg.save("room-map.svg")


def mission_timeline(rec):
    """When each of the 16 steps started and ended, in the run."""
    steps = rec["steps"]
    rows = len(steps)
    left, top, width, row_h = 300, 90, 500, 28
    t_max = math.ceil(rec["end_time"] / 5) * 5
    svg = Svg(860, top + rows * row_h + 60, "The 16 mission steps over time")
    svg.text(
        40,
        40,
        f"The 16 steps over time (one run, {rec['end_time']:.1f} s)",
        20,
        INK,
        weight="bold",
    )
    colors = {
        WALK: (BLUE, "walk"),
        messages.OPEN: (ORANGE, "hand"),
        messages.CLOSE: (ORANGE, "hand"),
    }
    for k, (color, label) in enumerate(
        ((BLUE, "walk step"), (TEAL, "arm step"), (ORANGE, "hand step"))
    ):
        svg.rect(left + 150 * k, 56, 18, 18, color, rx=3)
        svg.text(left + 150 * k + 26, 71, label, 15, BODY)
    for t in range(0, t_max + 1, 5):
        x = left + width * t / t_max
        svg.line(x, top - 6, x, top + rows * row_h, GRID, 1)
        svg.text(x, top + rows * row_h + 22, f"{t} s", 14, MUTED, anchor="middle")
    for k, (t0, index, label, kind) in enumerate(steps):
        t1 = steps[k + 1][0] if k + 1 < rows else rec["end_time"]
        color = colors.get(kind, (TEAL, "arm"))[0]
        y = top + k * row_h
        svg.text(left - 12, y + 19, f"{index + 1}. {label}", 15, BODY, anchor="end")
        svg.rect(
            left + width * t0 / t_max,
            y + 5,
            max(3, width * (t1 - t0) / t_max),
            row_h - 10,
            color,
            rx=3,
        )
    svg.save("mission-timeline.svg")


def reach_grid():
    """Where the right hand can grasp an upright bottle, seen from above."""
    ik = ArmIK(str(LESSON / "robot/g1_with_hands.xml"), "right")
    xs = [round(0.42 - 0.04 * i, 2) for i in range(9)]  # rows: far ahead at the top
    ys = [round(-0.04 * j, 2) for j in range(10)]  # columns: robot's middle on the left
    pitch, cell, left, top = 50, 44, 110, 70
    svg = Svg(960, 690, "Where the right hand can grasp, at 0.88 m height")
    svg.text(
        40,
        40,
        "Where the right hand can grasp (0.88 m high, fingers forward)",
        20,
        INK,
        weight="bold",
    )
    reachable = 0
    for r, x in enumerate(xs):
        for c, y in enumerate(ys):
            _, ok = ik.solve(
                np.array([x, y, 0.10]), upright_hand(0.0), robot.RIGHT_ARM_WALK
            )
            reachable += ok
            fill, stroke = ("#BBCDF2", BLUE) if ok else ("#E2DFD8", "#C9C5BB")
            if abs(x - GRASP_SPOT[0]) < 1e-6 and abs(y - GRASP_SPOT[1]) < 1e-6:
                fill, stroke = ORANGE, "#8F4512"
            svg.rect(left + c * pitch, top + r * pitch, cell, cell, fill, stroke, rx=6)
    pc = ys.index(round(GRASP_SPOT[1] - APPROACH, 2))
    pr = xs.index(round(GRASP_SPOT[0], 2))
    svg.raw(
        f'<rect x="{left + pc * pitch - 4}" y="{top + pr * pitch - 4}" width="{cell + 8}" height="{cell + 8}" '
        f'rx="9" fill="none" stroke="{INK}" stroke-width="3" stroke-dasharray="7 5"/>'
    )
    for r in (0, 3, 8):
        svg.text(
            left - 12,
            top + r * pitch + 28,
            f"{xs[r] * 100:.0f} cm",
            15,
            MUTED,
            anchor="end",
        )
    svg.text(
        left - 12, top + 5 * pitch + 28, "ahead", 15, BODY, anchor="end", weight="bold"
    )
    for c in (0, 3, 9):
        svg.text(
            left + c * pitch + cell / 2,
            top + 9 * pitch + 22,
            f"{-ys[c] * 100:.0f}",
            15,
            MUTED,
            anchor="middle",
        )
    svg.text(
        left + 5 * pitch,
        top + 9 * pitch + 48,
        "cm to the robot’s right →",
        15,
        BODY,
        anchor="middle",
        weight="bold",
    )
    rx, ry = left + cell / 2, top + 8 * pitch + cell / 2 + 2.5 * pitch
    svg.circle(rx, ry, 16, INK)
    svg.text(rx + 26, ry + 5, "robot (pelvis), facing up", 15, INK)
    legend = (
        ("#BBCDF2", BLUE, "the hand gets there"),
        ("#E2DFD8", "#C9C5BB", "out of reach"),
        (ORANGE, "#8F4512", "the bottle when the robot stops"),
    )
    for k, (fill, stroke, label) in enumerate(legend):
        y = 120 + 40 * k
        svg.rect(640, y, 26, 26, fill, stroke, rx=5)
        svg.text(676, y + 19, label, 15, BODY)
    svg.raw(
        f'<rect x="637" y="237" width="32" height="32" rx="7" fill="none" stroke="{INK}" stroke-width="3" stroke-dasharray="6 4"/>'
    )
    svg.text(676, 259, "where the open hand starts", 15, BODY)
    svg.save("reach.svg")
    return reachable, len(xs) * len(ys)


def nav_speed():
    """Walking speed the navigation asks for, by distance to the goal."""

    def speed(d):
        if d < navigation.POSITION_TOLERANCE:
            return 0.0
        return min(
            navigation.MAX_FORWARD, max(navigation.MIN_SPEED, navigation.GAIN * d)
        )

    left, bottom, w, h = 110, 400, 680, 300  # plot area
    d_max, v_max = 0.8, 0.45
    svg = Svg(900, 480, "Walking speed by distance to the goal")
    svg.text(
        40,
        40,
        "Walking speed by distance to the goal (forward)",
        20,
        INK,
        weight="bold",
    )

    def px(d, v):
        return left + w * d / d_max, bottom - h * v / v_max

    for v in (0.1, 0.2, 0.3, 0.4):
        x0, y = px(0, v)
        svg.line(x0, y, left + w, y, GRID, 1)
        svg.text(left - 12, y + 5, f"{v:.1f}", 14, MUTED, anchor="end")
    for d in (0, 0.2, 0.4, 0.6, 0.8):
        x, y0 = px(d, 0)
        svg.text(x, bottom + 24, f"{d * 100:.0f} cm", 14, MUTED, anchor="middle")
    svg.line(left, bottom, left + w, bottom, MUTED, 2)
    svg.line(left, bottom, left, bottom - h - 10, MUTED, 2)
    svg.text(left - 12, bottom - h - 24, "m/s", 14, MUTED, anchor="end")
    svg.text(
        left + w / 2, bottom + 52, "distance still to go", 15, BODY, anchor="middle"
    )
    ds = np.linspace(0, d_max, 801)
    svg.polyline([px(d, speed(d)) for d in ds], BLUE, 3)
    x, y = px(0.5, 0.4)
    svg.text(x + 40, y - 12, f"never faster than {navigation.MAX_FORWARD} m/s", 15, INK)
    x, y = px(0.3, speed(0.3))
    svg.text(x + 16, y + 22, f"speed = {navigation.GAIN} × distance", 15, INK)
    x, y = px(0.115, 0.05)
    svg.text(x, y, f"never slower than {navigation.MIN_SPEED} m/s", 15, INK)
    x, y = px(0.05, 0.008)
    svg.text(
        x,
        y,
        f"stop when closer than {navigation.POSITION_TOLERANCE * 100:.0f} cm",
        15,
        INK,
    )
    svg.save("nav-speed.svg")


def speed_profile():
    """Hand speed during one 25 cm straight-line move."""
    move = Trapezoid(0.25, HAND_SPEED, HAND_ACCEL)
    left, bottom, w, h = 110, 360, 680, 260
    t_max, v_max = 2.5, 0.18
    svg = Svg(900, 450, "Hand speed during a 25 cm move")
    svg.text(
        40,
        40,
        f"Hand speed during a 25 cm move ({move.duration:.2f} s)",
        20,
        INK,
        weight="bold",
    )

    def px(t, v):
        return left + w * t / t_max, bottom - h * v / v_max

    for v in (0.05, 0.10, 0.15):
        x0, y = px(0, v)
        svg.line(x0, y, left + w, y, GRID, 1)
        svg.text(left - 12, y + 5, f"{v:.2f}", 14, MUTED, anchor="end")
    for t in (0, 0.5, 1.0, 1.5, 2.0, 2.5):
        x, _ = px(t, 0)
        svg.text(x, bottom + 24, f"{t:.1f} s", 14, MUTED, anchor="middle")
    svg.line(left, bottom, left + w, bottom, MUTED, 2)
    svg.line(left, bottom, left, bottom - h - 10, MUTED, 2)
    svg.text(left - 12, bottom - h - 24, "m/s", 14, MUTED, anchor="end")
    ts = np.linspace(0, move.duration, 600)
    dt = ts[1] - ts[0]
    progress = np.array([move.progress(t) for t in ts]) * move.distance
    speeds = np.gradient(progress, dt)
    svg.polyline([px(t, v) for t, v in zip(ts, speeds)], BLUE, 3)
    ramp = move.ramp_time
    x, y = px(ramp / 2, HAND_SPEED / 2)
    svg.text(x + 12, y + 6, "speed up", 15, INK)
    x, y = px(move.duration / 2, HAND_SPEED)
    svg.text(x, y - 14, f"cruise at {HAND_SPEED} m/s", 15, INK, anchor="middle")
    x, y = px(move.duration - ramp / 2, HAND_SPEED / 2)
    svg.text(x - 12, y + 6, "slow down", 15, INK, anchor="end")
    svg.text(
        left + w / 2,
        bottom + 52,
        f"speeding up and slowing down take {ramp:.1f} s each ({HAND_SPEED} ÷ {HAND_ACCEL} m/s²)",
        15,
        BODY,
        anchor="middle",
    )
    svg.save("speed-profile.svg")


def standing():
    """The standing robot as an upside-down pendulum, with numbers from the model."""
    model = mujoco.MjModel.from_xml_path(str(LESSON / "robot/g1_with_hands.xml"))
    data = mujoco.MjData(model)
    data.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
    index = model.jnt_qposadr[[model.joint(n).id for n in robot.JOINTS]]
    data.qpos[index[robot.LEGS]] = robot.STAND_LEGS
    data.qpos[index[robot.LEFT_ARM]] = robot.LEFT_ARM_WALK
    data.qpos[index[robot.RIGHT_ARM]] = robot.RIGHT_ARM_WALK
    mujoco.mj_forward(model, data)
    ankle = data.xpos[model.body("left_ankle_roll_link").id]
    com = data.subtree_com[1]
    mass = model.body_subtreemass[1]
    height = com[2] - ankle[2]
    ahead = com[0] - ankle[0]
    pull = mass * GRAVITY * height
    push = 2 * robot.STAND_LEG_KP[4]

    svg = Svg(960, 560, "The standing robot as an upside-down pendulum")
    svg.text(40, 40, "Standing still: an upside-down pendulum", 20, INK, weight="bold")
    ax, ay, length, lean = 260, 450, 330, math.radians(10)
    cx, cy = ax + length * math.sin(lean), ay - length * math.cos(lean)
    svg.line(60, ay + 34, 520, ay + 34, MUTED, 3)
    scale = 1500  # px per meter, for the foot only
    svg.rect(ax - 0.05 * scale, ay + 14, 0.17 * scale, 20, "#C9C5BB", rx=6)
    svg.line(ax, ay, ax, ay - length - 30, MUTED, 2, dash="8 6")
    svg.line(ax, ay, cx, cy, INK, 9)
    svg.circle(cx, cy, 34, INK)
    svg.circle(ax, ay, 13, BG, INK, 4)
    svg.line(cx, cy + 42, cx, cy + 160, ORANGE, 5, arrow=True)
    svg.text(cx + 46, cy + 6, f"the robot’s weight: {mass:.1f} kg", 15, INK)
    svg.text(cx + 18, cy + 110, "pulls it over when it leans", 15, ORANGE)
    svg.text(ax - 14, ay - length / 2, f"{height:.2f} m", 15, BODY, anchor="end")
    svg.raw(
        f'<path d="M {ax + 46} {ay - 26} A 52 52 0 0 0 {ax - 46} {ay - 26}" fill="none" stroke="{BLUE}" '
        f'stroke-width="5" marker-end="url(#h{BLUE[1:]})"/>'
    )
    svg.text(ax - 60, ay - 34, "ankles push back", 15, BLUE, anchor="end")
    svg.text(
        ax - 0.05 * scale,
        ay + 58,
        "foot: 5 cm behind the ankle, 12 cm in front",
        14,
        MUTED,
    )
    x = 560
    lines = (
        ("Weight sits", f"{ahead * 100:.1f} cm in front of the ankle"),
        ("and", f"{height:.2f} m above it"),
        (
            "Leaning pulls it over with",
            f"{mass:.1f} × 9.81 × {height:.2f} = {pull:.0f} Nm per radian",
        ),
        (
            "Two ankles push back with",
            f"2 × {robot.STAND_LEG_KP[4]:.0f} = {push:.0f} Nm per radian",
        ),
        ("", f"{push:.0f} > {pull:.0f}, so it stays up"),
    )
    for k, (a, b) in enumerate(lines):
        svg.text(x, 120 + 62 * k, a, 15, MUTED)
        svg.text(x, 142 + 62 * k, b, 16, INK, weight="bold")
    svg.save("standing.svg")
    return mass, height, ahead, pull, push


def low_state_bar():
    """The 94 numbers of one low_state message."""
    n = robot.NUM_JOINTS
    parts = (
        ("time", 1, BLUE, "#F7F6F2"),
        ("joint angles", n, "#7FA3E3", INK),
        ("joint speeds", n, "#B5CAF0", INK),
        ("orientation", 4, "#E8935A", INK),
        ("turn rates", 3, "#F3C39A", INK),
    )
    total = sum(p[1] for p in parts)
    svg = Svg(940, 200, "The 94 numbers in one low_state message")
    svg.text(
        40,
        40,
        f"One low_state message: {total} numbers, always in this order",
        20,
        INK,
        weight="bold",
    )
    widths = [110, 260, 260, 120, 110]
    x, first = 40, 0
    for (label, count, fill, color), w in zip(parts, widths):
        svg.rect(x, 64, w - 4, 70, fill, rx=6)
        svg.text(x + 12, 94, label, 15, color, weight="bold")
        svg.text(x + 12, 118, str(count), 15, color)
        last = first + count - 1
        svg.text(
            x + 4,
            158,
            f"position {first}" if count == 1 else f"positions {first}–{last}",
            13,
            MUTED,
        )
        x += w
        first += count
    svg.text(40, 186, "(widths not to scale)", 13, MUTED)
    svg.save("low-state.svg")


def shoulder_lever(sample):
    """Why the shoulder needs its extra torque: weight times lever."""
    lever = sample["extra"] / (sample["mass"] * GRAVITY)
    svg = Svg(900, 520, "Where the shoulder's extra torque comes from")
    svg.text(
        40,
        40,
        f"The right shoulder at {sample['t']:.1f} s: torque = weight × lever",
        20,
        INK,
        weight="bold",
    )
    sx, sy = 330, 110
    svg.line(sx, 80, sx, 470, MUTED, 2, dash="8 6")
    svg.line(sx, sy, 240, 250, "#B9B4A8", 18)
    svg.line(240, 250, 300, 350, "#B9B4A8", 14)
    svg.circle(305, 360, 16, "#B9B4A8")
    svg.circle(sx, sy, 16, BG, INK, 5)
    cx, cy = 262, 266
    svg.circle(cx, cy, 18, INK, BG, 4)
    svg.line(cx, cy + 24, cx, cy + 150, ORANGE, 5, arrow=True)
    svg.line(cx, 470, sx, 470, BLUE, 4)
    svg.line(cx, 460, cx, 480, BLUE, 4)
    svg.line(sx, 460, sx, 480, BLUE, 4)
    svg.text(sx + 26, sy + 6, "shoulder axis", 15, INK)
    svg.text(cx - 28, cy - 2, "center of mass", 15, INK, anchor="end")
    svg.text(cx - 28, cy + 18, "of the whole arm", 15, INK, anchor="end")
    svg.text(
        cx - 16,
        cy + 110,
        f"weight {sample['mass'] * GRAVITY:.1f} N",
        15,
        ORANGE,
        anchor="end",
        weight="bold",
    )
    svg.text(
        (cx + sx) / 2,
        500,
        f"lever {abs(lever) * 100:.2f} cm",
        15,
        BLUE,
        anchor="middle",
        weight="bold",
    )
    svg.text(400, 300, "front →", 15, MUTED)
    svg.text(40, 500, "(not to scale)", 13, MUTED)
    x = 520
    rows = (
        (
            "Weight",
            f"{sample['mass']:.3f} kg × 9.81 = {sample['mass'] * GRAVITY:.1f} N",
        ),
        (
            "Arm’s center of mass",
            f"{abs(sample['forward']) * 100:.1f} cm {'behind' if sample['forward'] < 0 else 'in front of'} the shoulder",
        ),
        (
            "Shoulder axis tilt",
            f"{sample['tilt']:.0f}°, so the lever is {abs(lever) * 100:.2f} cm",
        ),
        (
            "Torque",
            f"{sample['mass'] * GRAVITY:.1f} N × {abs(lever):.4f} m = {abs(sample['extra']):.2f} Nm",
        ),
    )
    for k, (a, b) in enumerate(rows):
        svg.text(x, 120 + 70 * k, a, 15, MUTED)
        svg.text(x, 144 + 70 * k, b, 16, INK, weight="bold")
    svg.save("shoulder-lever.svg")


# ---------------------------------------------------------------- main


def main():
    MEDIA.mkdir(exist_ok=True)
    rec, sim = run_mission()
    room_map(rec, sim)
    mission_timeline(rec)
    reachable, tried = reach_grid()
    nav_speed()
    speed_profile()
    mass, height, ahead, pull, push = standing()
    low_state_bar()

    # the shoulder example: moments during "raise the hand", counted from its start
    raise_samples = [s for s in rec["shoulder"] if s["label"] == "raise the hand"]
    t0 = raise_samples[0]["t"]
    picks = {}
    for offset in (0.04, 0.74, 1.24, 2.14):
        picks[offset] = min(raise_samples, key=lambda s: abs(s["t"] - t0 - offset))
    shoulder_lever(picks[1.24])

    print(
        f"\nrun: {'finished' if rec['finished'] else 'NOT finished'} at {rec['end_time']:.1f} s"
    )
    print("steps:", ", ".join(f"{i + 1}:{t:.1f}s" for t, i, _, _ in rec["steps"]))
    miss = np.hypot(
        rec["bottle_end"][0] - room.PLACE_POINT[0],
        rec["bottle_end"][1] - room.PLACE_POINT[1],
    )
    print(f"bottle: {100 * miss:.1f} cm from the mark")
    print(f"reach: {reachable} of {tried} hand positions reachable")
    print(
        f"standing: mass {mass:.2f} kg, CoM {ahead * 100:.1f} cm ahead, {height:.3f} m up, pull {pull:.0f}, push {push:.0f} Nm/rad"
    )
    print("shoulder samples (t, target, angle, speed, kp, kd, extra, CoM forward cm):")
    for s in picks.values():
        spring = s["kp"] * (s["target"] - s["angle"])
        brake = -s["kd"] * s["speed"]
        print(
            f"  {s['t']:.2f}s  {s['target']:.3f} {s['angle']:.3f} {s['speed']:.3f}  {s['kp']:.0f} {s['kd']:.0f}  "
            f"{s['extra']:+.2f}  {s['forward'] * 100:+.1f}  -> spring {spring:+.2f} brake {brake:+.2f} total {spring + brake + s['extra']:+.2f}"
        )


if __name__ == "__main__":
    main()
