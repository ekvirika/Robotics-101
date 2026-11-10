# extras: more dora examples and two simulations

Runnable examples for [dora-rs](https://github.com/dora-rs/dora) 1.1.0, from a three-node "hello" graph up to two small robot simulations. Everything runs on a laptop: no webcam, no GPU, no ROS 2 needed (except example 10).

dora connects the programs that make up a robot. Each program is a **node**. One **YAML file** lists the nodes and says who sends what to whom. dora starts the nodes, passes data between them, and can restart them when they crash.

## Quick start

```bash
# needs uv: https://docs.astral.sh/uv/
./run.sh list            # show everything
./run.sh hello           # three nodes passing numbers
./run.sh mobile-robot    # a robot driving around obstacles, in a Rerun window
./run.sh arm --headless --stop-after 20s   # no window, recording saved to out/arm.rrd
```

`run.sh` installs the dora command (`dora-rs-cli` 1.1.0) and the Python environment on first use.

## What is in here

| Name | Folder | What it shows | Tested here |
| --- | --- | --- | --- |
| `hello` | [examples/01-hello-graph](examples/01-hello-graph) | the smallest graph: timer, counter, doubler, printer | yes |
| `camera` | [examples/02-camera-pipeline](examples/02-camera-pipeline) | camera into detector into viewer, synthetic camera by default, webcam and YOLO optional | yes, synthetic camera and colour detector |
| `types` | [examples/03-types](examples/03-types) | typed ports, `dora validate` catching wrong wiring | yes |
| `modules` | [examples/04-modules](examples/04-modules) | a group of nodes reused as one node | yes |
| `services` | [examples/05-services-actions](examples/05-services-actions) | request and reply, goals with progress and cancel | yes |
| `faults` | [examples/06-fault-tolerance](examples/06-fault-tolerance) | automatic restarts, dropping stale frames, input timeouts | yes |
| | [examples/07-record-replay](examples/07-record-replay) | record a run, play it back into a new detector | **no**, needs a Rust tool, see its README |
| | [examples/08-live-changes](examples/08-live-changes) | add and swap nodes while the graph runs | yes, `live.sh` |
| | [examples/09-distributed](examples/09-distributed) | one graph over several computers, by name and by label | yes, two daemons on one laptop, `local-cluster.sh` |
| | [examples/10-ros2-bridge](examples/10-ros2-bridge) | talk to a running ROS 2 system | **no**, needs ROS 2 and the bridge binary |
| `mobile-robot` | [simulations/mobile-robot](simulations/mobile-robot) | wheeled robot, simulated lidar, drives to 5 goals around obstacles | yes, all goals reached, no collisions |
| `arm` | [simulations/arm](simulations/arm) | 2-joint arm, inverse kinematics, PD control following a moving target | yes, about 1 cm tracking error |

## Layout

```
common/              shared helpers: image encoding, synthetic camera, Rerun setup
examples/NN-name/    dataflow.yml + nodes/ + README.md
simulations/name/    dataflow.yml + nodes/ + the pure maths (world.py, kinematics.py)
```

Run them from the repository root with `./run.sh <name>`.

## Things that bit us, so they do not bite you

- **`pip install dora` is a different package.** The Python library is `dora-rs`, the import is `dora`
- **Nodes start through `uv run`.** dora resolves `path:` before running it, so `path: .venv/bin/python` ends up at the system Python without your packages
- **Shared code via `PYTHONPATH: ../..`** in the YAML `env:`, because nodes run with the dataflow folder as working directory
- **Modules in 1.1.0 cannot find `uv` on PATH.** `examples/04-modules/modules/pyrun` is a two-line launcher next to the module file. The dora docs describe a fallback that is not in the 1.1.0 release
- **Restarts work with plain `dora run`.** No coordinator needed for `restart_policy`
- **`dora run` refuses YAML with `deploy:` sections.** Multi-computer graphs need `dora up` or a coordinator plus daemons, see example 09
- **On remote daemons nodes run in `_work/<id>/`,** so script paths in `args:` are written relative to that folder
- **`dora node connect` to a freshly added node did not deliver data in our test.** Declaring `inputs:` in the node's YAML before `dora node add` works, example 08 does that
- **`dora record` needs `dora-record-node`,** installed with `cargo install dora-record-node`. It is not part of the 1.1.0 release downloads
- **`input_timeout` is checked every `health_check_interval`,** 5 s by default. Example 06 sets it to 1 s so the timeout shows up quickly
