"""The robot's joint controller. Used by both watch.py and swing.py.

Same idea as on a real robot: trajectory.py says where the arm should be
right now and how fast it should move. This controller makes the motor
follow that, 500 times per second.
"""

KP = 20.0  # Nm per radian of position error
KD = 2.0  # Nm per rad/s of speed error


def joint_torque(ref_angle, ref_speed, ref_accel, angle, speed, inertia, weight_torque):
    """
    ref_angle      where the arm should be right now (radians), from trajectory.py
    ref_speed      how fast it should be moving right now (rad/s), from trajectory.py
    ref_accel      how hard it should be speeding up right now (rad/s^2), from trajectory.py
    angle          where the arm is now (radians), from the joint encoder
    speed          how fast it is moving (rad/s)
    inertia        how hard the arm is to spin up (kg m^2): torque = inertia x acceleration
    weight_torque  torque needed to hold the arm's weight at this angle (Nm)
    returns        the torque to send to the motor (Nm)
    """
    return (
        weight_torque  # hold the arm's weight
        + inertia * ref_accel  # torque needed to speed up / slow down as planned
        + KP * (ref_angle - angle)  # correct position error
        + KD * (ref_speed - speed)  # correct speed error
    )
