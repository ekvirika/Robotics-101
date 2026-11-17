# Task: crowd crossing

The G1 gets across a 12 m × 12 m square, from (−5, 0) to (5, 0), **as fast as is safe**, while 16 people walk across the square in all directions. It speeds up when the way is clear, slows down or stops when people come close, and must never touch anyone. It only knows what its lidar sees.

| | |
|---|---|
| **Success** | at the goal (within 0.3 m) and standing still, within 90 s |
| **Failure** | a person's body inside the robot's 30 cm circle, the robot fell, out of time |
| **Better** | less time to the goal |

## Files

| File | What it is |
|---|---|
| `layout.py` | the world as numbers: square size, start, goal, number of people, time limit |
| `env_cfg.py` | the Isaac Lab environment: floor, sky, marks, 16 people (`area`), the G1 (`g1.isaac`), how an episode ends |
| `mission.py` | the logic: the walking skill (`g1.navigator`) picks every command; arrived = at the goal and still |
| `dataflow.yml` | the dora graph: sim → tracker → planner → safety → sim |

Everything else comes from the shared packages: the robot and its walking software (`source/g1`), the people (`source/area`).

## How the robot decides

| Part | How | Standard it follows |
|---|---|---|
| tracker | lidar scan → clusters → people → alpha-beta filter (position and speed of each person) | ROS leg/people trackers |
| planner | tries walking velocities (1.5, 1.0, 0.5 m/s in 13 directions, and standing still), looks 4 s ahead, keeps the fastest one that stays clear of people; smooths speeding up and braking separately | Dynamic Window Approach (Fox, Burgard, Thrun 1997); Nav2 velocity smoother |
| safety | between the planner and the robot, can only slow down or stop: at most the speed from which the robot can still stop before reaching a person, given their measured speed toward it | speed and separation monitoring, ISO/TS 15066; placed like Nav2's Collision Monitor |

The safety rule: the free space to a person must stay at least S = v_h (T_r + T_s) + v_r T_r + B + Z, with v_h the person's speed toward the robot, v_r the robot's, T_r = 0.2 s reaction time, T_s and B the robot's stopping time and distance at its braking rate (`g1.planner.MAX_DECEL`), Z = 6 cm uncertainty. Example: a person 2 m ahead walking toward the robot at 1.3 m/s → the robot may approach at no more than 0.37 m/s.

## Run

On the Isaac Sim machine, from the repository folder (setup: main README):

```bash
$ISAACLAB/isaaclab.sh -p scripts/run_task.py crossing --headless --episodes 20     # 20 crowds, scored
$ISAACLAB/isaaclab.sh -p scripts/run_task.py crossing --livestream 2 --viz kit     # watch at <link>/viewer/
cd tasks/crossing && dora run dataflow.yml                                         # the dora graph
```

## Results

| Run | Result |
|---|---|
| Isaac Lab, NVIDIA's G1 + AGILE walking policy, 20 crowds (`run_task.py`) | **success 20/20, 0 collisions, 0 falls, 18.2 s on average (12.3 to 27.7 s)** |
| Isaac Lab, `dataflow.yml` (sim, tracker, planner, safety as dora nodes) | arrived at 19.9 s |
| stand-in robot, 20 crowds (`tests/test_crowd_crossing.py`, no physics) | success 20/20, 0 collisions, 20.4 s on average, smallest free space 16 cm |
