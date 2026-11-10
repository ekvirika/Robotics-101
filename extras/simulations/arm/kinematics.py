import math

L1 = 0.5
L2 = 0.4


def forward(q1: float, q2: float) -> tuple[tuple[float, float], tuple[float, float]]:
    elbow = (L1 * math.cos(q1), L1 * math.sin(q1))
    hand = (elbow[0] + L2 * math.cos(q1 + q2), elbow[1] + L2 * math.sin(q1 + q2))
    return elbow, hand


def inverse(x: float, y: float, elbow_up: bool = True) -> tuple[float, float] | None:
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1.0:
        return None
    s2 = math.sqrt(1 - c2 * c2) * (-1 if elbow_up else 1)
    q2 = math.atan2(s2, c2)
    q1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)
    return q1, q2
