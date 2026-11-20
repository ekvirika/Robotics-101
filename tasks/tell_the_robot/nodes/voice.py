"""The robot's voice: every reply is spoken aloud with macOS `say` (tasks/tell_the_robot/voice.py).

Inputs:  reply   what the robot says (agent)

A new reply cuts off the one still being spoken. VOICE_NAME picks a macOS voice
(`say -v ?` lists them), VOICE_RATE the words per minute, VOICE_DEVICE the output by part of its name
(default the ACCENTUM headset, so the microphone doesn't hear the robot; missing -> default output).
On a computer without `say` this node logs the reply and stays quiet.
"""

import os
import shutil

from dora import Node

from tasks.tell_the_robot.voice import Speaker, speaker_device

can_speak = shutil.which("say") is not None
speaker = Speaker(
    os.environ.get("VOICE_NAME") or None,
    os.environ.get("VOICE_RATE") or None,
    speaker_device(os.environ.get("VOICE_DEVICE", "ACCENTUM")) if can_speak else None,
)
node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT" and event["id"] == "reply":
        text = event["value"][0].as_py()
        if can_speak:
            speaker.say(text)
        else:
            node.log_info(f"(no say command) {text}")
speaker.stop()
