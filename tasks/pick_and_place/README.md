# Task: pick and place

The G1 walks to table A, picks up a bottle with its right hand, walks to table B and puts the bottle on the green mark.

| | |
|---|---|
| **Success** | the bottle stands on table B within 3 cm of the mark, and the hand has let go, within 120 s |
| **Failure** | the bottle fell to the floor, the robot fell, a step can't be reached, out of time |

## Files

| File | What it is |
|---|---|
| `layout.py` | the room as numbers: tables (0.8 m high), where the bottle starts, where the mark is, which way the robot faces at each table |
| `env_cfg.py` | the Isaac Lab environment: floor, sky, two tables, the bottle, the mark (`area`), the G1 (`g1.isaac`), how an episode ends |
| `mission.py` | the logic: 12 steps, each using a shared skill |
| `nodes/mission.py` | the dora node that runs `mission.py` (through `g1.dora.DoraG1`) |
| `dataflow.yml` | the dora graph: sim ⇄ mission |

## The steps

1. raise both hands for walking: 12 cm back and 5 cm up from where they work, so they can't catch a table's edge or knock the bottle (NVIDIA's default G1 pose holds the forearms forward at table height)
2. walk to table A: the walking skill (`g1.navigator`) to 0.8 m before the spot, then docks at 0.2 m/s to within 5 cm
3. open the hand
4. move the open hand next to the bottle (12 cm behind it)
5. move the hand around the bottle
6. close the hand
7. lift the bottle 10 cm
8. walk to table B (turning while walking: the walking policy doesn't turn on the spot)
9. move the bottle over the mark
10. put it down
11. open the hand
12. pull the hand back

**Where the robot stands** at a table is computed, not picked: the hand's grasp point (the middle of the gap between thumb and fingers, 7.3 cm ahead of the palm, from Unitree's G1 model) in the arm's natural pose sets where the bottle should be relative to the pelvis, and the robot stands so it is there. Then the arm only needs small moves.

**The arm** moves with the shared arm skill (`g1.manipulation`): inverse kinematics on Unitree's G1 model with Pinocchio, as in Unitree's own teleoperation software, then a smooth joint-space move. **The hand** opens to all joints at 0 and closes to each joint's limit in its closing direction (from Unitree's model); a held bottle stops the fingers.

**The tables are on the map** of the walking skill (like the static layer of a Nav2 costmap): the head lidar scans at about 1.2 m and passes over a 0.8 m table, as on the real G1.

**Docking** walks at 0.2 m/s and stops within 5 cm: NVIDIA's walking policy barely follows slower commands (measured: 0.10 m/s → 0.04 m/s). The arm makes up the rest, because every hand target is computed from the robot's measured pose and the bottle's measured position.

## Run

On the Isaac Sim machine, from the repository folder (setup: main README):

```bash
$ISAACLAB/isaaclab.sh -p scripts/run_task.py pick_and_place --headless --episodes 3
$ISAACLAB/isaaclab.sh -p scripts/run_task.py pick_and_place --livestream 2 --viz kit    # watch at <link>/viewer/
cd tasks/pick_and_place && dora run dataflow.yml                                        # the dora graph
```

## Results

| Run | Result |
|---|---|
| Isaac Lab, NVIDIA's G1 with Dex3 hands, real contact physics (`run_task.py --episodes 3`) | **success 3/3, 21.5 s on average**; one logged run put the bottle 2.6 cm from the mark's center, standing |
| Isaac Lab, `dataflow.yml` (sim and mission as dora nodes) | all 12 steps, mission finished |
| stand-in robot (`tests/test_pick_and_place.py`, no physics) | done in 21.7 s, bottle 0.1 cm from the mark |

The bottle lands 2–3 cm from the mark's center in physics (the limit is 3 cm): docking stops within 5 cm and the robot sways a little while standing. Tighter placement would need the arm to correct with the bottle's measured position just before letting go.
