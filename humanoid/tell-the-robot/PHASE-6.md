# Tell the robot, phase 6: voice and the dashboard

Goal: hold a key, say a sentence, and everything after that happens with no keyboard; the robot
answers out loud, and the screen reads from across a room. All code in `humanoid/03-g1-pick-place/`.

```bash
DASHBOARD_LAYOUT=audience humanoid/tell-the-robot/demo.sh g1-agent --stop-after 600s
cd humanoid/03-g1-pick-place && uv run --no-sync python nodes/chat.py   # second terminal
```

In the chat terminal: Enter on an empty line starts the microphone, Enter again stops it. Typing
still works. The first time, macOS asks whether the terminal may use the microphone.

## What was added

| File | What it is |
| --- | --- |
| `g1/voice.py` | `transcribe` (Whisper through MLX on the Mac's GPU), `Recorder` (push-to-talk), `Speaker` (macOS `say`), `hush` |
| `nodes/chat.py` | Enter, speak, Enter: the transcript goes to the robot as if typed; `CHAT_VOICE=0` turns it off |
| `nodes/voice.py` | new node in `agent.yml`: speaks every reply; a new reply cuts off the old one |
| `nodes/dashboard.py` | `DASHBOARD_LAYOUT=audience`: 3D room, head camera, conversation only; true object boxes in grey next to the remembered ones |
| `tests/test_voice.py` | 3 tests: transcript comparison, silence, the dashboard finds every loose object |
| `spikes/voice_check.py` | every model against spoken commands, with and without room noise |
| `pyproject.toml`, `run.sh` | a `voice` extra (`mlx-whisper`, `sounddevice`), synced by `./run.sh g1-agent` |

## Which Whisper model: measured, not assumed

No microphone needed for the measurement: macOS `say` speaks 8 robot commands in 2 voices
(16 recordings), each model transcribes them, a transcript counts when it matches word for word.
White noise stands in for a busy room (speech is about 0.1 RMS).

| Model | Download | Clean | Noise 0.02 | Noise 0.05 | Time per command |
| --- | --- | --- | --- | --- | --- |
| `whisper-base.en` | 140 MB | 16/16 | 14/16 | 14/16 | 0.04–0.07 s |
| **`whisper-small.en`** | 470 MB | **16/16** | **16/16** | **16/16** | 0.12–0.16 s |
| `whisper-large-v3-turbo` | 1.5 GB | 16/16 | 16/16 | 13/16 | 0.48–0.53 s |

Decision: `whisper-small.en` (the default; `WHISPER_MODEL` changes it). Every miss of the other
two is a different command, not a typo: “and the yellow box” became “**in** the yellow box”, “What
do you see?” became “What do you think?”. The bigger model is not the safer one here: turbo knows
every language, small.en only English, and it guesses less under noise.

## Things learned the hard way

- **A wrong word is worse than no word.** “Put the bottle in the box” is a valid command. Whisper
  gets the room's words as a prompt (`VOCABULARY`), and the chat prints “you said: …” so the
  speaker sees a mishearing before the robot acts on it.
- **The robot must not hear itself.** Starting the microphone first stops any reply `say` is still
  speaking (`hush`).
- **Push-to-talk, not always listening.** A showcase room is loud; a key press decides what is a
  command. A tap shorter than 0.3 s, or silence, is ignored.
- **Show the truth beside the belief.** Grey boxes are where the objects really are, tilted when
  they fell over; colored boxes are the robot's memory. The audience sees perception errors
  without anyone explaining them.
- **The default input is not always the laptop.** A paired Bluetooth headset (ACCENTUM) became
  the default microphone and recorded at RMS 0.004, under the old 0.005 silence cut, so every
  sentence came back empty. The chat now picks the MacBook's microphone by name (`CHAT_MIC`),
  prints which one it opened, the cut is 0.001, and “heard nothing” shows the seconds and level.
- **Click the chat terminal before pressing Enter.** With the MuJoCo or Rerun window in front,
  Enter goes there and no recording starts.

## Not measured yet

- Real voices through the Mac's microphone in a room: the numbers above are synthetic speech.
- The audience layout from 5 m away, on the showcase screen.

## Next: the move to Isaac Sim

Tornike wants the project in Isaac Sim, on a cloud GPU (Isaac Sim needs an NVIDIA RTX card). Every
node here talks through dora messages, so chat, voice, agent, skills, memory and the dashboard
carry over; `sim` and `camera` are replaced, and walking, the arm and the detector are measured
again on the new physics and images.
