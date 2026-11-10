import numpy as np

OBJECTS = [
    {"label": "red", "colour": (220, 40, 40), "radius": 40, "speed": (3.1, 2.3)},
    {"label": "blue", "colour": (40, 80, 220), "radius": 30, "speed": (-2.4, 3.7)},
    {"label": "green", "colour": (40, 180, 60), "radius": 50, "speed": (1.7, -2.9)},
]


def render_scene(frame_index: int, width: int, height: int) -> np.ndarray:
    image = np.full((height, width, 3), 235, dtype=np.uint8)
    image[height * 2 // 3 :, :] = (200, 190, 170)
    ys, xs = np.mgrid[0:height, 0:width]
    for item in OBJECTS:
        r = item["radius"]
        cx = bounce(frame_index * item["speed"][0] + width / 3, r, width - r)
        cy = bounce(frame_index * item["speed"][1] + height / 3, r, height - r)
        mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
        image[mask] = item["colour"]
    return image


def bounce(position: float, low: float, high: float) -> float:
    span = high - low
    phase = (position - low) % (2 * span)
    return low + (phase if phase <= span else 2 * span - phase)
