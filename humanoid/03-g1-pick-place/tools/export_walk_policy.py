"""Convert Unitree's G1 walking policy (motion.pt) to ONNX (policies/g1_walk.onnx).

You only need this to rebuild policies/g1_walk.onnx yourself.

Where the policy comes from: github.com/unitreerobotics/unitree_rl_gym,
file deploy/pre_train/g1/motion.pt (BSD-3-Clause, Unitree Robotics).
Unitree trained it with reinforcement learning in Isaac Gym.

Why convert: robots run trained networks with ONNX Runtime, a small library
(Unitree's newer deploy code does the same). PyTorch is a 1 GB training tool;
the robot doesn't need it.

Safety: motion.pt is a zip file. Opening it with torch.load would run Python
code stored in the file. We don't do that: we only read the raw numbers
(float32 arrays) inside the zip. Their names and shapes are listed in WEIGHTS,
taken from the file's index (data.pkl, read with `python -m pickletools`).

The network: an LSTM memory (47 inputs -> 64 numbers) and two layers
(64 -> 32 -> 12). The memory becomes an input and an output of the ONNX file,
so the code that runs it keeps the memory between steps.

Run (PyTorch is installed only for this one command):
  uv run --no-project --with torch --with onnx --with onnxruntime \
      python tools/export_walk_policy.py motion.pt policies/g1_walk.onnx
"""

import sys
import zipfile

import numpy as np
import onnxruntime
import torch

OBS_SIZE = 47
MEMORY_SIZE = 64
HIDDEN_SIZE = 32
ACTION_SIZE = 12

# layer name -> (file inside motion.pt, shape)
WEIGHTS = {
    "memory.weight_ih_l0": ("policy_lstm_1/data/6", (4 * MEMORY_SIZE, OBS_SIZE)),
    "memory.weight_hh_l0": ("policy_lstm_1/data/7", (4 * MEMORY_SIZE, MEMORY_SIZE)),
    "memory.bias_ih_l0": ("policy_lstm_1/data/8", (4 * MEMORY_SIZE,)),
    "memory.bias_hh_l0": ("policy_lstm_1/data/9", (4 * MEMORY_SIZE,)),
    "actor.0.weight": ("policy_lstm_1/data/2", (HIDDEN_SIZE, MEMORY_SIZE)),
    "actor.0.bias": ("policy_lstm_1/data/3", (HIDDEN_SIZE,)),
    "actor.2.weight": ("policy_lstm_1/data/4", (ACTION_SIZE, HIDDEN_SIZE)),
    "actor.2.bias": ("policy_lstm_1/data/5", (ACTION_SIZE,)),
}


class WalkPolicy(torch.nn.Module):
    """The same layers as Unitree's policy, with the memory passed in and out."""

    def __init__(self):
        super().__init__()
        self.memory = torch.nn.LSTM(OBS_SIZE, MEMORY_SIZE, num_layers=1)
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(MEMORY_SIZE, HIDDEN_SIZE),
            torch.nn.ELU(),
            torch.nn.Linear(HIDDEN_SIZE, ACTION_SIZE),
        )

    def forward(self, obs, h, c):
        out, (h, c) = self.memory(obs.unsqueeze(0), (h, c))
        return self.actor(out.squeeze(0)), h, c


def read_weights(path):
    weights = {}
    with zipfile.ZipFile(path) as archive:
        for name, (member, shape) in WEIGHTS.items():
            raw = archive.read(member)
            weights[name] = np.frombuffer(raw, dtype="<f4").reshape(shape).copy()
    return weights


def numpy_policy(w, obs, h, c):
    """The same network written out by hand, to check the ONNX file."""
    gates = w["memory.weight_ih_l0"] @ obs + w["memory.bias_ih_l0"]
    gates += w["memory.weight_hh_l0"] @ h + w["memory.bias_hh_l0"]
    i, f, g, o = np.split(gates, 4)
    sigmoid = lambda x: 1 / (1 + np.exp(-x))  # noqa: E731
    c = sigmoid(f) * c + sigmoid(i) * np.tanh(g)
    h = sigmoid(o) * np.tanh(c)
    hidden = w["actor.0.weight"] @ h + w["actor.0.bias"]
    hidden = np.where(hidden > 0, hidden, np.exp(hidden) - 1)  # ELU
    return w["actor.2.weight"] @ hidden + w["actor.2.bias"], h, c


def main(source, target):
    weights = read_weights(source)
    policy = WalkPolicy()
    policy.load_state_dict({k: torch.from_numpy(v) for k, v in weights.items()})
    policy.eval()

    example = (
        torch.zeros(1, OBS_SIZE),
        torch.zeros(1, 1, MEMORY_SIZE),
        torch.zeros(1, 1, MEMORY_SIZE),
    )
    torch.onnx.export(
        policy,
        example,
        target,
        input_names=["obs", "h_in", "c_in"],
        output_names=["action", "h_out", "c_out"],
        opset_version=17,
        dynamo=False,
    )

    # Check: ONNX and the hand-written network agree over 500 random steps.
    session = onnxruntime.InferenceSession(target)
    rng = np.random.default_rng(0)
    h = np.zeros((1, 1, MEMORY_SIZE), dtype=np.float32)
    c = np.zeros((1, 1, MEMORY_SIZE), dtype=np.float32)
    h_ref = np.zeros(MEMORY_SIZE)
    c_ref = np.zeros(MEMORY_SIZE)
    worst = 0.0
    for _ in range(500):
        obs = rng.normal(size=(1, OBS_SIZE)).astype(np.float32)
        action, h, c = session.run(None, {"obs": obs, "h_in": h, "c_in": c})
        expected, h_ref, c_ref = numpy_policy(weights, obs[0], h_ref, c_ref)
        worst = max(worst, float(np.abs(action[0] - expected).max()))
    print(
        f"saved {target}; largest difference to the hand-written network: {worst:.1e}"
    )
    if worst > 1e-4:
        sys.exit("the ONNX file does not match")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
