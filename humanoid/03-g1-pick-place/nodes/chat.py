"""Talk to the robot from a terminal: type a sentence, or press Enter and speak.

A dynamic dora node: start the dataflow first (./run.sh g1-agent), then in a
second terminal, from humanoid/03-g1-pick-place:

    uv run --no-sync python nodes/chat.py

Typing: a sentence, then Enter. Speaking (tell-the-robot phase 6): Enter on an
empty line starts the microphone, Enter again stops it; Whisper turns it into
text on this Mac (g1/voice.py) and the text goes to the robot as if typed.
CHAT_VOICE=0 turns speaking off, WHISPER_MODEL picks the model, CHAT_MIC the
microphone by part of its name (default: the MacBook's own, not a headset).

Type or say "stop" to stop the robot at once. Ctrl-D or Ctrl-C to leave (the robot keeps running).
"""

import os
import sys
import threading
from pathlib import Path

import numpy as np
import pyarrow as pa
from dora import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # started by hand, not by dora: find g1/

from g1 import voice  # noqa: E402

SPEAKING = os.environ.get("CHAT_VOICE", "1") != "0"
MODEL = os.environ.get("WHISPER_MODEL", voice.MODEL)
MIC = os.environ.get("CHAT_MIC", voice.MIC)

node = Node("chat")


def warm_up():
    """Load Whisper now, so the first spoken sentence isn't slow."""
    voice.transcribe(np.full(voice.SAMPLE_RATE, 0.01, np.float32), MODEL)


def read_keyboard():
    device, mic = voice.microphone(MIC) if SPEAKING else (None, "")
    if SPEAKING:
        print(f"microphone: {mic}", flush=True)
    recorder = voice.Recorder(device)
    for line in sys.stdin:
        line = line.strip()
        if line:
            node.send_output("text", pa.array([line]))
            continue
        if not SPEAKING:
            continue
        voice.hush()
        print("listening... (Enter to stop)", flush=True)
        try:
            recorder.start()
        except Exception as error:
            print(f"(microphone failed: {error})", flush=True)
            continue
        sys.stdin.readline()
        audio = recorder.stop()
        heard = voice.transcribe(audio, MODEL)
        if not heard:
            print(f"(heard nothing: {len(audio) / voice.SAMPLE_RATE:.1f} s, level {voice.level(audio):.4f}, silence below {voice.TOO_QUIET})", flush=True)
            continue
        print(f"you said: {heard}", flush=True)
        node.send_output("text", pa.array([heard]), {"spoken": True})


if SPEAKING:
    threading.Thread(target=warm_up, daemon=True).start()
threading.Thread(target=read_keyboard, daemon=True).start()
hint = ", or press Enter, speak, Enter again (click this terminal first)" if SPEAKING else ""
print(f"Type a sentence{hint}. \"stop\" stops the robot.", flush=True)
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "reply":
        print(f"robot: {event['value'][0].as_py()}", flush=True)
