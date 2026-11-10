# dora, step by step

A beginner's guide to [dora](https://github.com/dora-rs/dora), the framework that connects the programs of a robot, built around dora's **own official examples**.

**Read the guide:** open `docs/index.html` in a browser, or run

```bash
cd docs && python3 -m http.server 8765    # then open http://localhost:8765
```

## Try it in two minutes

```bash
# needs uv: curl -LsSf https://astral.sh/uv/install.sh | sh
./run.sh               # installs the dora command, lists the lessons
./run.sh first         # the official first example: three programs passing numbers
./run.sh fake-camera   # the official camera + AI example, with a fake camera
```

## What is in here

| Folder | What it is |
| --- | --- |
| `docs/` | the guide. `guide.md` is the text, `index.html` the built page |
| `official/` | two examples copied unchanged from dora 1.1.0: `python-dataflow` and `python-yolo-detection` |
| `steps/` | the lessons, each a small change to an official example |
| `extras/` | more examples for single features and two robot simulations |
| `humanoid/` | the humanoid track: MuJoCo basics → dora nodes → the Unitree G1 picking up a bottle |
| `tools/` | `pretty-logs.py` (readable dora output) and `build_docs.py` (builds the guide) |
| `run.sh` | the launcher |

## Rebuild the guide after editing `docs/guide.md`

```bash
uv run --with markdown python tools/build_docs.py
```

Files under `official/` are © the dora-rs contributors, Apache License 2.0, from github.com/dora-rs/dora release v1.1.0.
