# Arm simulation

A 2-joint planar arm (links 0.5 m and 0.4 m) follows a target that moves in a spiral.

```bash
./run.sh arm
```

```
target ── target ──→ ik ── joint_targets ──→ arm ── joint_state ──→ viewer
```

| Node | Does | Rate |
| --- | --- | --- |
| `target` | moving point to reach | 20 Hz |
| `ik` | inverse kinematics: target position into two joint angles, elbow up | per target |
| `arm` | joint dynamics with a PD controller plus velocity feedforward, speed limit 3 rad/s | 100 Hz |
| `viewer` | draws the arm and target, plots both joint angles, logs the tracking error | per state |

- **Inverse kinematics** has two answers, elbow up and elbow down. Set `ELBOW: down` in `dataflow.yml` to see the other one
- **Out of reach:** when the target leaves the 0.9 m reach, `ik` logs a warning and the arm holds its last targets
- **Feedforward matters:** without it the arm trails about 10 cm behind the moving target. With it, about 1 cm. Delete `target_velocity[i]` in `nodes/arm.py` to see the difference
