"""Speech in and out for talking to the robot (tell-the-robot phase 6).

In: Whisper, run on the Mac's GPU through MLX (mlx-whisper). A model is
downloaded from Hugging Face on first use and cached.
Out: macOS `say`, one sentence at a time; a new sentence cuts off the old one.

Whisper knows English, not this room. VOCABULARY is passed as the prompt, so
"table A", "the green mark" and the colors are the likely words, and
`normalized` makes "Table a." and "table A" the same command.
"""

import re
import subprocess

import numpy as np

MODEL = "mlx-community/whisper-small.en-mlx"
SAMPLE_RATE = 16000  # Hz, what Whisper expects
VOCABULARY = "Put the blue bottle on the green mark. Table A, table B. Red, yellow, green, blue. Bottle, can, box. Stop."
TOO_SHORT = 0.3  # s: a tap of the key, not a sentence
TOO_QUIET = 0.005  # RMS of float audio in -1..1: nothing was said


def transcribe(audio, model=MODEL):
    """Text from a WAV path or a float32 array at SAMPLE_RATE; "" for silence."""
    import mlx_whisper

    if isinstance(audio, np.ndarray):
        audio = audio.astype(np.float32).ravel()
        if len(audio) < TOO_SHORT * SAMPLE_RATE or np.sqrt(np.mean(audio**2)) < TOO_QUIET:
            return ""
    result = mlx_whisper.transcribe(
        audio if isinstance(audio, np.ndarray) else str(audio),
        path_or_hf_repo=model,
        language="en",
        initial_prompt=VOCABULARY,
        condition_on_previous_text=False,
        temperature=0.0,
    )
    return result["text"].strip()


def normalized(text):
    """Lowercase words without punctuation: how two transcripts are compared."""
    return " ".join(re.sub(r"[^a-z0-9' ]", " ", text.lower()).split())


class Recorder:
    """Push-to-talk: start() opens the microphone, stop() closes it and returns the audio."""

    def __init__(self):
        self.chunks, self.stream = [], None

    def start(self):
        import sounddevice

        self.chunks = []
        self.stream = sounddevice.InputStream(
            samplerate=SAMPLE_RATE, channels=1, dtype="float32", callback=lambda data, *_: self.chunks.append(data.copy())
        )
        self.stream.start()

    def stop(self):
        if self.stream is None:
            return np.zeros(0, np.float32)
        self.stream.stop()
        self.stream.close()
        self.stream = None
        return np.concatenate(self.chunks).ravel() if self.chunks else np.zeros(0, np.float32)


def hush():
    """Stop any sentence `say` is speaking: the robot must not talk into the microphone."""
    subprocess.run(["killall", "say"], capture_output=True)


class Speaker:
    """Says one sentence at a time with macOS `say`; a new one stops the old one."""

    def __init__(self, voice=None, rate=None):
        self.args = ["say"] + (["-v", voice] if voice else []) + (["-r", str(rate)] if rate else [])
        self.process = None

    def say(self, text):
        self.stop()
        if text.strip():
            self.process = subprocess.Popen(self.args + [text])

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        self.process = None
