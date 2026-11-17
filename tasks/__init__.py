"""The tasks the G1 does. Each task is one folder with everything of its own:

  env_cfg.py    its Isaac Lab environment (area, objects, start, how it ends)
  mission.py    its logic, on the robot's interfaces only (g1.interfaces)
  dataflow.yml  its dora graph
  README.md     what it is and how to run it

Importing this package registers every task's environment with Isaac Lab.
"""

from . import crossing, pick_and_place  # noqa: F401
