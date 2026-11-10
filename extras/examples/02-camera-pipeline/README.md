# 02 · camera pipeline

The demo from the training: camera into detector into viewer.

```
camera ── image ──┬──→ detector ── boxes ──→ viewer
                  └──────────────────────────→ viewer
```

```bash
./run.sh camera                 # Rerun window: moving shapes with boxes
./run.sh camera --headless --stop-after 15s
```

By default it needs no hardware:

- **Synthetic camera:** draws three moving coloured circles, `CAMERA_SOURCE: synthetic`
- **Colour detector:** finds the red, green and blue circle, `DETECTOR: colour`

To use a real webcam and YOLO:

```bash
uv sync --extra vision          # opencv and ultralytics, about 2 GB with torch
```

Then in `dataflow.yml` set `CAMERA_SOURCE: webcam` and `DETECTOR: yolo`. macOS asks for camera permission for the terminal on the first frame.

- **Frames are flat arrays plus metadata:** `common/frames.py` sends the image as a flat Arrow array with `width` and `height` in the metadata, the receiver rebuilds it
- **The detector never falls behind:** its input has `queue_size: 1` and `queue_policy: drop_oldest`, so a slow detector always works on the newest frame
- **Example:** with YOLO on a CPU the detector may manage 10 frames a second while the camera sends 30. The viewer still shows smooth video, boxes update 10 times a second
