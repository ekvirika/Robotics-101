"""Speech in and out (g1/voice.py), without a microphone or a model.

  uv run --with pytest pytest tests/test_voice.py
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1 import scenes, voice  # noqa: E402

LESSON = Path(__file__).resolve().parents[1]


def test_transcripts_compare_without_case_and_punctuation():
    assert voice.normalized("Put the blue bottle on Table A.") == voice.normalized("put the blue bottle on table a")
    assert voice.normalized("I'm thirsty!") == "i'm thirsty"


def test_a_tap_or_silence_is_heard_as_nothing():
    assert voice.transcribe(np.zeros(voice.SAMPLE_RATE * 2, np.float32), model="never-loaded") == ""
    assert voice.transcribe(np.full(int(0.1 * voice.SAMPLE_RATE), 0.3, np.float32), model="never-loaded") == ""


def test_the_dashboard_finds_every_loose_object_and_not_the_robot():
    source = (LESSON / "nodes" / "dashboard.py").read_text()
    start = source.index("def free_objects")
    namespace = {"mujoco": mujoco}
    exec(source[start : source.index("\ndef ", start + 1)], namespace)
    scene = scenes.make_scene(6)
    model = mujoco.MjModel.from_xml_path(str(scenes.write_scene(scene, LESSON / "robot")))
    found = namespace["free_objects"](model)
    assert len(found) == len(scene.objects)
    assert all(len(half) == 3 for _, half in found)
