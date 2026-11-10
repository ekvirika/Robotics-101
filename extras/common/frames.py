import numpy as np
import pyarrow as pa


def encode_image(image: np.ndarray) -> tuple[pa.Array, dict]:
    height, width = image.shape[:2]
    return pa.array(image.ravel()), {
        "width": width,
        "height": height,
        "encoding": "rgb8",
    }


def decode_image(value: pa.Array, metadata: dict) -> np.ndarray:
    width = int(metadata["width"])
    height = int(metadata["height"])
    return value.to_numpy(zero_copy_only=False).reshape(height, width, 3)


def boxes_to_arrow(boxes: list[dict]) -> pa.Array:
    return pa.array(
        boxes,
        type=pa.struct(
            [
                ("label", pa.string()),
                ("confidence", pa.float32()),
                ("x1", pa.float32()),
                ("y1", pa.float32()),
                ("x2", pa.float32()),
                ("y2", pa.float32()),
            ]
        ),
    )


def boxes_from_arrow(value: pa.Array) -> list[dict]:
    return value.to_pylist()
