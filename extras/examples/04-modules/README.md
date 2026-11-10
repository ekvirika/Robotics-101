# 04 · modules

A module is a group of nodes in its own YAML file. A dataflow uses the whole group as if it were one node.

```bash
./run.sh modules --stop-after 15s
dora expand examples/04-modules/dataflow.yml    # see the unpacked graph
```

- **`modules/perception.yml`** holds two nodes: the colour detector and a box counter. It declares one input, `camera_feed`, and one output, `detections`
- **`_mod/camera_feed`** inside the module means "whatever the parent connects to `camera_feed`"
- **In `dataflow.yml`** the group is one node called `vision`. `dora expand` shows it becomes `vision.detector` and `vision.counter`
- **Example:** the same perception group can go into a warehouse robot and an arm cell, each with its own camera, without copying YAML

**1.1.0 quirk:** inside a module, `path: uv` fails with "Could not find source path modules/uv". The module uses `modules/pyrun`, a two-line launcher that calls `uv run`.
