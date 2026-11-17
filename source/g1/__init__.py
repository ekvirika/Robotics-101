"""The Unitree G1: everything any task needs from the robot, independent of the task.

  interfaces.py    what the robot offers, one small interface per job (as Unitree's SDK):
                   Locomotion, UpperBody, Hands, State, Lidar, Perception
  joints.py        joint names in Unitree's official order
  kinematics.py    arm inverse kinematics on Unitree's G1 model (Pinocchio)
  manipulation.py  arm and hand skill: reach a spot smoothly, open or close a hand
  navigator.py     walking skill: get to a spot safely (tracker + planner + safety), dock, turn
  tracker.py, planner.py, safety.py   the parts of the walking skill
  messages.py      the layout of the dora messages
  isaac.py         the robot in NVIDIA Isaac Lab (IsaacG1 implements the interfaces)
  dora.py          the robot behind dora messages (DoraG1 implements the interfaces)
  nodes/           dora nodes: sim (Isaac Lab), tracker, planner, safety
  assets/          Unitree's G1 model (URDF, BSD-3-Clause)

Only isaac.py and nodes/sim.py need Isaac Sim; everything else runs anywhere.
"""
