"""The robot's voice: every reply is spoken aloud with macOS `say` (g1/voice.py).

Inputs:  reply   what the robot says (agent)

A new reply cuts off the one still being spoken. VOICE_NAME picks a macOS voice
(`say -v ?` lists them), VOICE_RATE the words per minute. On a computer without
`say` this node logs the reply and stays quiet.
"""

import os
import shutil

from dora import Node

from g1.voice import Speaker

speaker = Speaker(os.environ.get("VOICE_NAME") or None, os.environ.get("VOICE_RATE") or None)
can_speak = shutil.which("say") is not None
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
