"""Phase 6: which Whisper model hears robot commands right, and how fast, on this Mac.

No microphone needed: macOS `say` speaks each command into a 16 kHz WAV file, every
model transcribes it. NOISE=<rms> adds white noise (a busy room; speech is about
0.1 RMS). Prints per model: commands heard exactly (after lowercasing and dropping
punctuation), seconds per command, and every miss.

    humanoid/tell-the-robot/light.sh uv run --no-sync python humanoid/tell-the-robot/spikes/voice_check.py [model ...]
"""

import os
import subprocess
import sys
import tempfile
import time
import wave
from pathlib import Path

import numpy as np

LESSON = Path(__file__).resolve().parents[2] / "03-g1-pick-place"
sys.path.insert(0, str(LESSON))

from g1 import voice  # noqa: E402

COMMANDS = [
    "Put the blue bottle on the green mark.",
    "Put the yellow can on table A.",
    "Move the blue bottle to the other table.",
    "Put the red bottle next to the blue bottle.",
    "Put the blue bottle and the yellow box on table B.",
    "What do you see?",
    "I'm thirsty.",
    "Stop.",
]
VOICES = ["Samantha", "Daniel"]
NOISE = float(os.environ.get("NOISE", "0"))
MODELS = sys.argv[1:] or ["mlx-community/whisper-base.en-mlx", "mlx-community/whisper-small.en-mlx", "mlx-community/whisper-large-v3-turbo"]


def spoken(folder):
    files = []
    for v in VOICES:
        for i, sentence in enumerate(COMMANDS):
            path = folder / f"{v}-{i}.wav"
            subprocess.run(["say", "-v", v, "-o", str(path), "--data-format=LEI16@16000", sentence], check=True)
            files.append((v, sentence, with_noise(path)))
    return files


def with_noise(path):
    """The speech as float audio, with NOISE added, padded by 0.5 s of noise each side like a real recording."""
    with wave.open(str(path)) as w:
        audio = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768
    pad = np.zeros(voice.SAMPLE_RATE // 2, np.float32)
    audio = np.concatenate([pad, audio, pad])
    return audio + np.random.default_rng(len(audio)).normal(0, NOISE, len(audio)).astype(np.float32) if NOISE else audio


def main():
    with tempfile.TemporaryDirectory() as tmp:
        files = spoken(Path(tmp))
        for model in MODELS:
            voice.transcribe(files[0][2], model)  # load and warm up
            print(f"noise {NOISE}", end=": ")
            right, seconds, misses = 0, [], []
            for v, sentence, path in files:
                start = time.perf_counter()
                heard = voice.transcribe(path, model)
                seconds.append(time.perf_counter() - start)
                if voice.normalized(heard) == voice.normalized(sentence):
                    right += 1
                else:
                    misses.append(f"{v}: {sentence!r} -> {heard!r}")
            print(f"{model}: {right}/{len(files)} exact, {sum(seconds) / len(seconds):.2f} s per command (worst {max(seconds):.2f})", flush=True)
            for miss in misses:
                print(f"    {miss}", flush=True)


if __name__ == "__main__":
    main()
