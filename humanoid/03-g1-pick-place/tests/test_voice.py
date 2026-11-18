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


def _speech(seconds, rng):
    t = np.arange(int(seconds * voice.SAMPLE_RATE)) / voice.SAMPLE_RATE
    return (0.1 * np.sin(2 * np.pi * 220 * t) * (0.6 + 0.4 * np.sin(2 * np.pi * 3 * t))).astype(np.float32) + _room(seconds, rng)


def _room(seconds, rng):
    return rng.normal(0, 0.003, int(seconds * voice.SAMPLE_RATE)).astype(np.float32)


def test_recording_ends_after_a_pause_following_speech():
    rng = np.random.default_rng(0)
    assert voice.speech_ended(np.concatenate([_room(0.5, rng), _speech(2.0, rng), _room(1.1, rng)]))
    assert not voice.speech_ended(np.concatenate([_room(0.5, rng), _speech(2.0, rng), _room(0.5, rng)]))
    assert not voice.speech_ended(np.concatenate([_room(0.5, rng), _speech(2.0, rng)]))


def test_room_noise_alone_never_ends_a_recording():
    rng = np.random.default_rng(1)
    assert not voice.speech_ended(_room(5.0, rng))
    assert not voice.speech_ended(rng.normal(0, 0.02, 5 * voice.SAMPLE_RATE).astype(np.float32))


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
