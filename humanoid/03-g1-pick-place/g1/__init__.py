"""The G1 robot software, without dora: plain Python you can test on its own.

  robot.py       joint names and groups, poses, motor gains
  messages.py    the layout of every message between the nodes
  simulator.py   MuJoCo world + the G1's motor drivers and sensors
  locomotion.py  walking (Unitree's trained policy) and standing (balance)
  navigation.py  "go to this spot" -> walking velocity
  arm_ik.py      inverse kinematics: hand position -> 7 arm joint angles
  arm.py         smooth arm moves, gravity compensation, the grip
  mission.py     the pick-and-place steps

The dora nodes in nodes/ only pass messages to and from these classes.
"""
