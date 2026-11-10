# The walking policy

`g1_walk.onnx` is Unitree's trained walking controller for the G1, as an ONNX file (128 KB).

- **Source:** [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym), file `deploy/pre_train/g1/motion.pt`, commit `276801e`. BSD-3-Clause, see `LICENSE`.
- **How it was made:** Unitree trained it with reinforcement learning in Isaac Gym: thousands of simulated G1s tried to walk at random speeds, and kept what didn't make them fall.
- **What it is:** a small neural network. An LSTM memory (47 numbers in, 64 numbers of memory) and two layers (64 → 32 → 12). It runs 50 times per second.
- **In:** 47 numbers. The pelvis turn rates, which way gravity points, the wanted velocity, 12 leg angles and speeds, its own last output, and where it is in the step cycle. The full list is in `g1/locomotion.py`.
- **Out:** 12 numbers, one per leg joint. Target angle = standing angle + 0.25 × output.

## Why ONNX

PyTorch (`.pt`) is a training tool and about 1 GB to install. Robots run trained networks with **ONNX Runtime**, a small library; Unitree's newer deploy code does the same. `tools/export_walk_policy.py` converts the file. It reads only the numbers stored in `motion.pt`; it never loads the file as Python code, so nothing in it can run. The tool then checks that the ONNX file gives the same answers as the original network.

To rebuild it yourself:

```bash
uv run --no-project --with torch --with onnx --with onnxruntime \
    python tools/export_walk_policy.py motion.pt policies/g1_walk.onnx
```
