# Lesson 2: the arm as dora nodes

Same arm, same physics, same plan and controller as lesson 1. The difference: each job is now its own program (a **node**), and they talk by sending messages. `dataflow.yml` says who sends what to whom.

**Why split it?** On a real robot these jobs run as separate programs, often on different computers, written by different people. If the charts crash, the robot keeps moving. If you swap the simulator for the real motor driver, nothing else changes.

## Run it

```bash
./run.sh arm-nodes
```

You get the MuJoCo 3D window and a Rerun window with live charts. The arm goes to 90°, 45°, −30°, 120°, 0°, one goal every 3 seconds, then the list starts over. Ctrl-C stops everything.

The 3D window only **shows** the arm. Its Backspace (reset) and sliders don't control the robot: the next message from `sim` overwrites them.

## The nodes

| Node | File | Gets | Sends | Job |
| --- | --- | --- | --- | --- |
| `sim` | `nodes/sim.py` | timer 2 ms, `torque` | `joint_state` | the robot (MuJoCo). On hardware: the motor driver |
| `goals` | `nodes/goals.py` | timer 3 s | `goal` | the task: next angle to go to |
| `arm.planner` | `nodes/planner.py` | `goal`, `joint_state` | `reference` | smooth plan (lesson 1's `trajectory.py`) |
| `arm.controller` | `nodes/joint_controller.py` | `reference`, `joint_state`, `estop` | `torque` | motor push (lesson 1's `controller.py`) |
| `safety` | `nodes/safety.py` | `joint_state` | `estop` | stops the arm if it's out of range, too fast, or silent |
| `plots` | `nodes/plots.py` | timer 20 ms, state, plan, goal | – | live charts (Rerun) |
| `viewer3d` | `nodes/viewer3d.py` | timer 16 ms, state, goal | – | 3D window (MuJoCo) |

`arm.planner` and `arm.controller` come from the module `modules/joint_control.yml`: one joint's control, written once. A humanoid would use it once per joint.

## The loop, 500 times a second

`sim` sends `joint_state` → `planner` sends `reference` → `controller` sends `torque` → `sim` applies it on the next tick.

## dora features used here

| In `dataflow.yml` | Why |
| --- | --- |
| `dora/timer/millis/2`, `dora/timer/secs/3` | built-in clocks: physics every 2 ms, a new goal every 3 s |
| top-level `env` | settings every node needs: the model file and where lesson 1's code is |
| node `env` | settings for one node: the goals list, the safety limits |
| `module` | planner + controller packed as one reusable "joint control" block |
| `output_types` + `strict_types` | dora checks that both ends of a connection agree on the data type |
| `input_timeout` | `safety` notices if the robot goes silent for 0.5 s |
| `queue_size` | `viewer3d` has room for 0.5 s of states while its window opens |
| `restart_policy`, `max_restarts` | if the charts crash, dora restarts them (up to 3 times) |
| `name`, `description` | notes for humans; `dora graph` shows them |

## Try

- Change `GOALS_DEG` in `dataflow.yml` and run again.
- Set the safety `MAX_SPEED` to `1.0`: the arm starts moving, safety stops it, and the controller only brakes.
- `dora graph dataflow.yml --open` draws the graph.
- `dora expand dataflow.yml` shows the YAML with the module unpacked.
