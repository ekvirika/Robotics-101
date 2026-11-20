"""Speech in and out for talking to the robot (Whisper in, macOS say out).

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
VOCABULARY = (
    "Put the blue bottle on the green mark. Table A, table B. Red, yellow, green, blue. Bottle, can, box. Stop."
)
TOO_SHORT = 0.3  # s: a tap of the key, not a sentence
TOO_QUIET = 0.001  # RMS of float audio in -1..1: nothing was said (a headset mic speaking is about 0.004)
PAUSE = 1.0  # s of quiet after speech: the sentence is over, recording stops by itself
SPEECH = 0.01  # RMS: louder than this, and 3x the room's floor, is speech
LONGEST = 15.0  # s: a recording stops here even without a pause
MIC = "MacBook"  # the built-in mic; a Bluetooth headset may be the default input and is quiet


def transcribe(audio, model=MODEL):
    """Text from a WAV path or a float32 array at SAMPLE_RATE; "" for silence."""
    import mlx_whisper

    if isinstance(audio, np.ndarray):
        audio = audio.astype(np.float32).ravel()
        if len(audio) < TOO_SHORT * SAMPLE_RATE or level(audio) < TOO_QUIET:
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


def microphone(name=MIC):
    """(index, name) of the first input device whose name contains `name`, else the default input."""
    import sounddevice

    inputs = [(i, d["name"]) for i, d in enumerate(sounddevice.query_devices()) if d["max_input_channels"]]
    for i, found in inputs:
        if name and name.lower() in found.lower():
            return i, found
    default = sounddevice.query_devices(kind="input")
    return None, default["name"]


def speech_ended(audio, rate=SAMPLE_RATE, pause=PAUSE):
    """Speech, then `pause` seconds without it: the speaker finished. False before anyone spoke.

    Loudness per 0.1 s window, against the quietest tenth of the recording (the room)."""
    window = rate // 10
    count = len(audio) // window
    tail = int(round(pause * 10))
    if count <= tail:
        return False
    levels = np.sqrt(np.mean(np.asarray(audio[: count * window], np.float32).reshape(count, window) ** 2, axis=1))
    loud = levels > max(SPEECH, 3 * float(np.percentile(levels, 10)))
    return bool(loud[:-tail].any() and not loud[-tail:].any())


def speech_started(audio, rate=SAMPLE_RATE):
    """Someone spoke somewhere in `audio` (same loudness rule as speech_ended)."""
    window = rate // 10
    count = len(audio) // window
    if count < 3:
        return False
    levels = np.sqrt(np.mean(np.asarray(audio[: count * window], np.float32).reshape(count, window) ** 2, axis=1))
    return bool((levels > max(SPEECH, 3 * float(np.percentile(levels, 10)))).any())


def robot_speaking():
    """macOS `say` is talking: what the microphone hears now is the robot, not a person."""
    return subprocess.run(["pgrep", "-x", "say"], capture_output=True).returncode == 0


def level(audio):
    """RMS of float audio: 0 is silence, speech close to a laptop mic is about 0.02 to 0.1."""
    return float(np.sqrt(np.mean(audio**2))) if len(audio) else 0.0


class Recorder:
    """Push-to-talk: start() opens the microphone, stop() closes it and returns the audio."""

    def __init__(self, device=None):
        self.device, self.chunks, self.stream = device, [], None

    def start(self):
        import sounddevice

        self.chunks = []
        self.stream = sounddevice.InputStream(
            device=self.device,
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="float32",
            callback=lambda data, *_: self.chunks.append(data.copy()),
        )
        self.stream.start()

    def audio(self):
        """What was recorded so far."""
        chunks = list(self.chunks)
        return np.concatenate(chunks).ravel() if chunks else np.zeros(0, np.float32)

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
