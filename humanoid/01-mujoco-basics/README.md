# Lesson 1: a robot arm with one motor

**What we simulate:** a robot's shoulder. You give a target angle, and the motor moves the arm there and holds it. A humanoid works the same way with about 29 motors.

## Run it

```bash
cd humanoid/01-mujoco-basics
uv sync --extra sim
uv run mjpython watch.py
```

(macOS needs `mjpython` instead of `python` to open the 3D window.)

## What you see

| In the window | What it is |
| --- | --- |
| grey post | the robot's body, fixed |
| yellow cylinder | the shoulder joint, with the motor inside |
| blue arm + hand | the part the motor moves |
| green line | the target angle |
| top-left numbers | arm angle, target, motor torque, and the torque the arm's weight needs |

**Try:** right panel → Control → drag **target_angle_deg**. Double-click the arm, then Ctrl + right-drag to push it. Backspace resets.

## Smooth motion (`trajectory.py` + `controller.py`)

Like a real robot, in two parts:

1. **`trajectory.py`** plans the motion: speed up, cruise at max 86 deg/s, slow down, stop exactly on the target.
2. **`controller.py`** makes the motor follow that plan, 500 times per second:

```python
torque = weight_torque                 # hold the arm's weight
       + inertia * ref_accel           # push needed to speed up / slow down as planned
       + KP * (ref_angle - angle)      # fix position error
       + KD * (ref_speed - speed)      # fix speed error
```

## Without the window

```bash
uv run python swing.py
uv run python swing.py --target 45
```

## Words

- **joint**: where two parts connect and rotate. **hinge** = one axis
- **motor / actuator**: turns a joint. Strength = **torque**, in newton-metres (Nm)
- **radian**: angle unit. 1 rad = 57.3°, 90° = 1.57 rad
- **MuJoCo**: the physics engine. It computes how everything moves
