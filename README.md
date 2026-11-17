# Unitree G1 tasks: Isaac Lab + dora

A Unitree **G1** humanoid that does several tasks, simulated in [NVIDIA Isaac Lab](https://isaac-sim.github.io/IsaacLab/) with NVIDIA's own G1 and walking policy, and connected by [dora](https://github.com/dora-rs/dora) like the programs on a real robot.

| Task | What the robot does |
|---|---|
| [`tasks/crossing`](tasks/crossing/README.md) | gets across a square full of walking people as fast as is safe, never touching anyone |
| [`tasks/pick_and_place`](tasks/pick_and_place/README.md) | walks to a table, picks up a bottle, carries it to another table, puts it on a mark |

## Contents

1. [The repository](#1-the-repository)
2. [How it is built, and why](#2-how-it-is-built-and-why)
3. [Set up the cloud machine (NVIDIA Brev)](#3-set-up-the-cloud-machine-nvidia-brev)
4. [Install](#4-install)
5. [Run](#5-run)
6. [Tests](#6-tests)
7. [Add a task](#7-add-a-task)
8. [Cost and troubleshooting](#8-cost-and-troubleshooting)
9. [Status](#9-status)
10. [Sources](#10-sources)

## 1. The repository

```
source/
  g1/            THE ROBOT: everything any task needs from the G1
    interfaces.py    what the robot offers (Locomotion, UpperBody, Hands, State, Lidar, Perception)
    joints.py        joint names in Unitree's official order
    kinematics.py    arm inverse kinematics (Pinocchio on Unitree's G1 model)
    manipulation.py  arm and hand skill
    navigator.py     walking skill (tracker.py + planner.py + safety.py)
    isaac.py         the robot in Isaac Lab (implements the interfaces)
    dora.py          the robot behind dora messages (implements the interfaces)
    nodes/           shared dora nodes: sim, tracker, planner, safety
    assets/          Unitree's G1 model (URDF)
  area/          THE PLACES: crowd model, scene pieces (floor, sky, people, tables, bottles), crowd terms
tasks/
  crossing/      layout.py, env_cfg.py, mission.py, dataflow.yml, README.md
  pick_and_place/  layout.py, env_cfg.py, mission.py, nodes/, dataflow.yml, README.md
scripts/         run_task.py (any task, scored), measure_walking.py, list_envs.py
tests/           the robot software and both tasks, on any computer
```

**Each task has everything of its own** in its folder: its world as numbers (`layout.py`), its Isaac Lab environment (`env_cfg.py`), its logic (`mission.py`), its dora graph (`dataflow.yml`) and its README. **Everything about the robot and the places is shared**, in `source/`.

## 2. How it is built, and why

**Like a real G1 is programmed.** Unitree's SDK splits the robot into separate interfaces: walking (`LocoClient.Move(vx, vy, vyaw)`: the robot moves its own legs), arms and waist (the arm SDK, joint targets on `rt/arm_sdk`), hands (Dex3, `rt/dex3/left|right/cmd`), and state (`rt/lowstate`). `g1/interfaces.py` mirrors that split. Arm inverse kinematics is the application's job (the SDK takes joint angles); `g1/kinematics.py` does it with Pinocchio on Unitree's G1 model, as Unitree's own teleoperation software does (`xr_teleoperate`).

**SOLID:**

| Principle | Where |
|---|---|
| single responsibility | one job per module: the tracker finds people, the planner chooses a velocity, the safety layer only slows down, the kinematics only computes angles, a mission only sequences skills |
| open/closed | a new task is a new folder; the robot, the skills, `scripts/run_task.py` and the shared dora nodes don't change |
| Liskov substitution | `IsaacG1` (Isaac Lab), `DoraG1` (dora messages) and the tests' `FakeG1` are interchangeable: every task runs on each of them unchanged |
| interface segregation | small interfaces (`Locomotion`, `UpperBody`, `Hands`, `State`, `Lidar`, `Perception`); the crossing needs only walking, state and lidar |
| dependency inversion | tasks and skills depend on the interfaces, never on Isaac Lab; a real-robot backend on Unitree's SDK would implement the same interfaces |

**NVIDIA's robot in Isaac Lab** (`g1/isaac.py`), as in Isaac Lab's G1 loco-manipulation environment:

| Part | NVIDIA's piece |
|---|---|
| robot | `G1_29DOF_CFG`: the G1 with two Dex3-1 hands |
| walking | the pre-trained **AGILE** lower-body policy (`Policies/Agile/agile_locomotion.pt`): command [forward, sideways, turn, base height]; base height 0.8 m as NVIDIA sends it |
| arms, hands | joint position targets (like Unitree's arm SDK and Dex3) |
| timing | physics 200 Hz, control 50 Hz, one picture per control step |
| physics device | **CPU** (scripts default to `--device cpu`): for one robot PhysX is about 9× faster per step on the CPU (0.9 ms) than on the GPU (8 ms), measured on the L40S; the GPU pays off for thousands of robots (training) |
| floor, sky | as in Isaac Lab's locomotion environments |

**Walking speed.** NVIDIA's AGILE training config uses forward commands from −0.5 to 1.5 m/s, sideways ±0.5 m/s, turning ±1 rad/s; commands are clipped to that, and the planner's top speed is 1.5 m/s. Measured in Isaac Lab (`scripts/measure_walking.py`):

| Commanded | Walks at | Stops in | Over |
|---|---|---|---|
| 0.5 m/s | 0.48 m/s | 0.44 s | 0.06 m |
| 1.0 m/s | 0.94 m/s | 0.62 s | 0.17 m |
| 1.5 m/s | 1.42 m/s | 1.50 s | 0.21 m |

**Safety.** The walking skill's safety layer applies ISO/TS 15066 speed and separation monitoring: the robot never walks faster than it can stop before reaching a person, given their measured speed toward it. The braking rate it relies on is 0.5 m/s²; the measured one is at least 0.95 m/s² (average over the stop, table above), so the assumption is on the safe side.

**The robot's lidar** sits where the real G1 has its Livox MID-360: 0.428 m above `torso_link` (Unitree's model, `mid360_joint`), about 1.2 m above the ground. It is Isaac Lab's `MultiMeshRayCaster`, which follows moving objects (the plain `RayCaster` "currently supports only a single static mesh"): 360 level rays, 12 m, 1 cm noise. Each task says what it should see (`g1.isaac.lidar_cfg`): the people in the crossing, the tables in pick and place.

## 3. Set up the cloud machine (NVIDIA Brev)

Isaac Sim needs an NVIDIA RTX GPU, so it runs on a rented cloud machine used from the browser.

1. Account at [brev.nvidia.com](https://brev.nvidia.com), credits under **Billing**.
2. NVIDIA's **Isaac Launchable** ([guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_advanced_cloud_setup_launchable.html)): Isaac Sim 6.0.1 + Isaac Lab 3.0.0-beta2 + VS Code in the browser. We used `g6e.xlarge` (1 × L40S, 4 CPU threads), $2.79/hr.
3. **Deploy Launchable**, wait for **Running** and **Built** (about 10 min the first time).
4. Instance page → **Using Secure Links** → open the **Shareable URL** (`https://isaac-xxxxxxxx.brevlab.com`): VS Code on the machine. **Terminal → New Terminal**.

From your own computer (optional): `brew install brevdev/homebrew-brev/brev`, `brev login`, `brev ls`, `brev start|stop|shell <name>`. The code runs in the container `vscode`.

## 4. Install

On the machine (`$ISAACLAB` = `/workspace/isaaclab` on the Isaac Launchable):

```bash
export ISAACLAB=/workspace/isaaclab
cd /workspace && git clone https://github.com/tchavtchavadzetornike/Robotics-101.git g1 && cd g1
$ISAACLAB/isaaclab.sh -p -m pip install -e .        # g1, area, tasks + Pinocchio, dora
$ISAACLAB/isaaclab.sh -p -m pip install dora-rs-cli==1.1.0   # the dora command
export PATH=$HOME/.local/bin:$PATH                  # where pip puts it on the Isaac Launchable
$ISAACLAB/isaaclab.sh -p scripts/list_envs.py       # shows both tasks
```

On any computer, for the tests: `pip install -e ".[dev]"`.

## 5. Run

```bash
$ISAACLAB/isaaclab.sh -p scripts/measure_walking.py --headless                     # once: real top speed and braking
$ISAACLAB/isaaclab.sh -p scripts/run_task.py crossing --headless --episodes 20     # scored
$ISAACLAB/isaaclab.sh -p scripts/run_task.py pick_and_place --headless --episodes 3
$ISAACLAB/isaaclab.sh -p scripts/run_task.py crossing --livestream 2 --viz kit     # watch it
cd tasks/crossing && dora run dataflow.yml                                         # a task's dora graph
```

**Watch live:** start with `--livestream 2 --viz kit` and open the VS Code address with `/viewer/` at the end. One viewer tab at a time. Teammates: invite them under **Team**, then **Edit Access** next to the link; never make the link public (it is VS Code with a terminal on your machine).

## 6. Tests

On any computer, no GPU: `pytest -s`

| Test | Checks |
|---|---|
| `test_navigation.py` | tracker follows a walking person; planner reaches a goal around a person; safety stops, slows and lets go exactly by the ISO/TS 15066 rule; message layouts |
| `test_crowd_crossing.py` | the crossing task in 20 crowds: success 20/20, collisions 0, mean 20.4 s, closest 16 cm (stand-in robot) |
| `test_pick_and_place.py` | the pick-and-place task: every step reachable, bottle 0.1 cm from the mark (stand-in robot) |

The tasks run on `tests/fakes.py:FakeG1`, a stand-in robot (exact walking and joint targets, real arm kinematics, no physics). `tests/fake_sim_node.py` replaces the sim node to check a task's dora graph without a GPU; both graphs pass with it.

## 7. Add a task

1. `tasks/<name>/layout.py`: the world as numbers.
2. `env_cfg.py`: subclass `g1.isaac.G1TaskEnvCfg`, add pieces from `area.scene` and the episode endings.
3. `mission.py`: the logic on the robot's interfaces, using the skills (`g1.navigator`, `g1.manipulation`); `update()`, `done`, `failed`, and `succeeded(robot, mission)`.
4. `__init__.py`: `ENV_ID`, `Mission`, `succeeded`, and `gym.register(...)`; add it to `tasks/__init__.py`.
5. `dataflow.yml` (shared nodes from `g1/nodes`, your own in `nodes/`), `README.md`, a test with `FakeG1`.

`scripts/run_task.py <name>` then runs and scores it.

## 8. Cost and troubleshooting

| | |
|---|---|
| running | $2.79/hr (`g6e.xlarge`), also when idle: **Stop** it when nobody uses it |
| stopped | only the disk; everything installed stays |
| link "can't be reached" after Start | wait a few minutes; Chrome: `chrome://net-internals/#dns` → Clear host cache |
| `/viewer/` "Waiting for stream…" | started with `--headless`, or still starting: refresh after the first lines |
| `No module named g1` | section 4 |
| simulation slow in the viewer | measured on `g6e.xlarge`: about 0.7× real time with the picture, 1.1× without; rendering the picture costs about 11 ms of each 20 ms step on its 4 CPU threads. Check `--device cpu` (the default) |

## 9. Status

Run on the cloud machine (Isaac Sim 6.0.1, Isaac Lab 3.0.0-beta2, NVIDIA's G1 and AGILE walking policy), 2026-11-17:

| | Result |
|---|---|
| crossing, `run_task.py --episodes 20` | **PASS: 20/20 crowds, 0 collisions, 0 falls, 18.2 s on average (12.3 to 27.7 s)** |
| pick and place, `run_task.py --episodes 3` | **PASS: 3/3, 21.5 s on average**; the Dex3 hand holds the bottle in physics |
| crossing `dataflow.yml` (4 dora nodes) | arrived at the goal at 19.9 s |
| pick and place `dataflow.yml` (2 dora nodes) | all 12 steps, mission finished |
| `measure_walking.py` | table in section 2 |
| tests (`pytest`), on the machine and on a Mac | 6 passed |

**What the walking policy can't do** (measured): it barely follows commands below 0.2 m/s (0.10 m/s → 0.04 m/s) and doesn't turn on the spot (0.5 rad/s → 0.01 rad/s); it turns fine while walking. Docking therefore walks at 0.2 m/s and stops within 5 cm, and the arm makes up the rest.

**Next:** NVIDIA's animated human characters instead of colored figures.

## 10. Sources

- **Unitree** (BSD-3-Clause): `unitree_sdk2_python` (G1 `LocoClient`, arm SDK example with `G1JointIndex`), `unitree_ros` G1 model `g1_29dof_with_hand_rev_1_0.urdf` (in `source/g1/assets`, with its license), `xr_teleoperate` (Pinocchio arm IK).
- **NVIDIA Isaac Lab** (BSD-3-Clause): G1 loco-manipulation environment (`G1_29DOF_CFG`, `AgileBasedLowerBodyAction`, AGILE observations), locomotion environments (floor, sky), MDP terms. **NVIDIA WBC-AGILE**: G1 velocity-height command ranges. Isaac Sim 6.0.1 under the NVIDIA Isaac Sim license.
- **ROS 2 Nav2**: Collision Monitor, velocity smoother, costmap static layer.
- **ISO/TS 15066:2016** speed and separation monitoring (from ISO 13855; stopping per ISO 10218-1); J. Marvel, R. Norcross, NIST, 2017.
- D. Helbing, P. Molnár, *Social force model for pedestrian dynamics*, 1995 (crowd). D. Fox, W. Burgard, S. Thrun, *The Dynamic Window Approach*, 1997 (planner). S. Buss, *Introduction to Inverse Kinematics*, 2004 (damped least squares). CrowdNav, SocNavBench (metrics).
- **dora-rs** 1.1.0 (Apache-2.0).
