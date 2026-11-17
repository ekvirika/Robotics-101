"""The crowd-crossing task's world, as plain numbers (used by the Isaac Lab
environment, the mission and the tests)."""

HALF_SIZE = 6.0  # m: the square reaches from -6 to +6 (12 m x 12 m)
START = (-5.0, 0.0, 0.0)  # x, y, heading: the west side of the square
GOAL = (5.0, 0.0)  # the east side, 10 m away
NUM_PEOPLE = 16
CROWD_SEED = 0  # the first crowd; each new episode gets the next one
TIME_LIMIT = 90.0  # s
