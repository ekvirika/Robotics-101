"""The places the robot works in, as reusable pieces.

  crowd.py   walking people: the social force model (plain NumPy)
  scene.py   Isaac Lab scene entities: ground, sky, people, tables, bottles, marks
  terms.py   Isaac Lab MDP terms: MoveCrowd (walk the people), person_collision

A task's environment (tasks/<task>/env_cfg.py) puts together the pieces it needs.
scene.py and terms.py need a running Isaac Sim; crowd.py runs anywhere.
"""
