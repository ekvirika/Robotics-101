# Mobile robot simulation

A wheeled robot in a 10 × 10 m arena with six round obstacles. It has a simulated 36-beam lidar and drives to five goals in turn.

```bash
./run.sh mobile-robot
```

```
simulator ── pose, scan ──→ planner ── cmd_vel ──→ simulator
     └─────── pose, scan ──→ viewer ←── goal ─────── planner
```

| Node | Does | Rate |
| --- | --- | --- |
| `simulator` | moves the robot from speed commands, blocks it on collision, casts lidar beams | 50 Hz |
| `planner` | steers towards the goal and away from close lidar hits | every scan |
| `viewer` | draws obstacles, robot, trail, lidar hits and goal in Rerun | every message |

- **The maths is separate from dora:** `world.py` (motion, collisions, lidar) and `controller.py` (steering) are plain Python, easy to test
- **Measured here:** all 5 goals reached in under 60 s, no collisions
- **Try:** move an obstacle in `world.py` onto the straight line between two goals and watch it steer around
- **Try:** set `SAFE_DISTANCE = 0.4` in `controller.py`, the robot cuts much closer to obstacles
