"""Talk to the robot from a terminal: type a sentence, or press Enter and speak.

A dynamic dora node: start the dataflow first (dora up, dora start), then in a
second terminal on the computer with the microphone:

    python tasks/tell_the_robot/nodes/chat.py

Typing: a sentence, then Enter. Speaking: Enter on an
empty line starts the microphone; it stops after a 1 s pause, after 15 s, or on Enter; Whisper turns it into
text on this Mac (tasks/tell_the_robot/voice.py) and the text goes to the robot as if typed.
By default it listens hands-free: speak, pause 1 s, and the sentence goes; never while the robot talks
(CHAT_HANDS_FREE=0 goes back to Enter first). CHAT_VOICE=0 turns speaking off,
CHAT_AUTO_STOP=0 waits for Enter only, WHISPER_MODEL picks the model, CHAT_MIC the
microphone by part of its name (default: the MacBook's own, not a headset).

Type or say "stop" to stop the robot at once. Ctrl-D or Ctrl-C to leave (the robot keeps running).
"""

import os
import select
import sys
import threading
import time

import numpy as np
import pyarrow as pa
from dora import Node

from tasks.tell_the_robot import voice

SPEAKING = os.environ.get("CHAT_VOICE", "1") != "0"
MODEL = os.environ.get("WHISPER_MODEL", voice.MODEL)
MIC = os.environ.get("CHAT_MIC", voice.MIC)
AUTO_STOP = os.environ.get("CHAT_AUTO_STOP", "1") != "0"
HANDS_FREE = SPEAKING and os.environ.get("CHAT_HANDS_FREE", "1") != "0"
QUIET_RESTART = 10.0  # s: no speech this long, start a fresh recording (the floor stays current)

node = Node("chat")


def warm_up():
    """Load Whisper now, so the first spoken sentence isn't slow."""
    voice.transcribe(np.full(voice.SAMPLE_RATE, 0.01, np.float32), MODEL)


def hear_one(recorder):
    """Hands-free: the next sentence a person says, as audio. Never while the robot talks."""
    while True:
        while voice.robot_speaking():
            time.sleep(0.2)
        recorder.start()
        start = time.monotonic()
        while True:
            time.sleep(0.1)
            if voice.robot_speaking():
                recorder.stop()
                break
            audio = recorder.audio()
            if voice.speech_ended(audio):
                return recorder.stop()
            spoke = voice.speech_started(audio)
            waited = time.monotonic() - start
            if (not spoke and waited > QUIET_RESTART) or waited > QUIET_RESTART + voice.LONGEST:
                audio = recorder.stop()
                if spoke:
                    return audio
                break


def listen_hands_free():
    device, mic = voice.microphone(MIC)
    print(f"microphone: {mic}. Just speak; the sentence is sent after a 1 s pause.", flush=True)
    recorder = voice.Recorder(device)
    while True:
        try:
            audio = hear_one(recorder)
        except Exception as error:
            print(f"(microphone failed: {error})", flush=True)
            return
        heard = voice.transcribe(audio, MODEL)
        if heard:
            print(f"you said: {heard}", flush=True)
            node.send_output("text", pa.array([heard]), {"spoken": True})


def wait_for_end(recorder):
    """Enter, a pause after speech, or LONGEST: whichever comes first. Returns why."""
    start = time.monotonic()
    while True:
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            return "Enter"
        if AUTO_STOP and voice.speech_ended(recorder.audio()):
            return "you paused"
        if time.monotonic() - start > voice.LONGEST:
            return f"{voice.LONGEST:.0f} s"


def read_keyboard():
    device, mic = voice.microphone(MIC) if SPEAKING else (None, "")
    if SPEAKING:
        print(f"microphone: {mic}", flush=True)
    recorder = voice.Recorder(device)
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if line:
            node.send_output("text", pa.array([line]))
            continue
        if not SPEAKING:
            continue
        voice.hush()
        print(
            "listening... (stops when you pause, or press Enter)" if AUTO_STOP else "listening... (Enter to stop)",
            flush=True,
        )
        try:
            recorder.start()
        except Exception as error:
            print(f"(microphone failed: {error})", flush=True)
            continue
        why = wait_for_end(recorder)
        audio = recorder.stop()
        print(f"(stopped: {why})", flush=True)
        heard = voice.transcribe(audio, MODEL)
        if not heard:
            seconds, level = len(audio) / voice.SAMPLE_RATE, voice.level(audio)
            print(f"(heard nothing: {seconds:.1f} s, level {level:.4f}, silence below {voice.TOO_QUIET})", flush=True)
            continue
        print(f"you said: {heard}", flush=True)
        node.send_output("text", pa.array([heard]), {"spoken": True})


if SPEAKING:
    threading.Thread(target=warm_up, daemon=True).start()
if HANDS_FREE:
    threading.Thread(target=listen_hands_free, daemon=True).start()
    SPEAKING = False  # the keyboard is for typing only; the microphone is already listening
threading.Thread(target=read_keyboard, daemon=True).start()
hint = ", or press Enter and speak (click this terminal first)" if SPEAKING else ""
print(f'Type a sentence{hint}. "stop" stops the robot.', flush=True)
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "reply":
        print(f"robot: {event['value'][0].as_py()}", flush=True)
