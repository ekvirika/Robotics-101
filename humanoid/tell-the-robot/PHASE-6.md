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
| `nodes/chat.py` | Enter, speak: recording stops after a 1 s pause, after 15 s, or on Enter; the transcript goes to the robot as if typed; `CHAT_VOICE=0` turns it off, `CHAT_AUTO_STOP=0` waits for Enter, `CHAT_MIC` picks the microphone |
| `nodes/voice.py` | new node in `agent.yml`: speaks every reply; a new reply cuts off the old one |
| `nodes/dashboard.py` | `DASHBOARD_LAYOUT=audience`: 3D room, head camera, conversation only; true object boxes in grey next to the remembered ones |
| `tests/test_voice.py` | 5 tests: transcript comparison, silence, the pause that ends a recording, room noise alone, the dashboard finds every loose object |
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

## After the first live test: shorter survey, hand tracking, stop on a pause

Seen live: asked for the blue bottle, the robot saw it at the first viewpoint and still walked on
to the second and third, then all the way back (`spikes/walk_trace.py` prints every walk, 0.5 s
apart). Three changes, all simulator-independent:

- **`look_around(find)`.** Claude passes the object the request is about; the survey stops once
  it is in the memory and says which tables it skipped (`Robot.unseen`). The original room:
  survey 30 s to 10 s, the whole task 105 s to 82 s, bottle 1.0 cm from the mark.
- **`place` looks before it puts down, on a skipped table only.** On arrival: 1 s look, then the
  spot and the hand's sweep must be free of anything the memory didn't know when it chose the
  spot. `spikes/find_eval.py`, 5 rooms with an object 15 to 20 cm from the mark: 4 placed, 1
  honest refusal (a red box in the hand's way; with every object known there is no free spot
  either), nothing else moved.
- **The memory carries the held object along.** Before, it stayed where it was picked up, so the
  camera seeing it in the hand at the other table made a second, phantom object (4 of 5 rooms
  refused the mark: "#6 blue bottle is in my hand's way"). The skills node now sends the hand's
  position in `held` every 5 cm; a sighting is the held object only with the same color and kind.
- **Recording stops on a 1 s pause** after speech (`voice.speech_ended`: loudness per 0.1 s
  against the room's quietest tenth), at 15 s, or on Enter. 32 `say` recordings, quiet and loud
  room: never cut short, stopped at most 1.0 s after the sentence. A hesitation over 1 s mid-
  sentence still cuts it; `CHAT_AUTO_STOP=0` goes back to Enter only.

`spikes/skills_eval.py`, 10 rooms (seeds 0 to 9), before / after: bottles 8/8 / 8/8, cans 5/5 /
5/5, boxes 5/5 / 3/5. Both box failures tip at "pull the hand straight back": the plans match to
1 mm, the hand drags the box 6 cm instead of 1 cm (room 6 traced step by step; the box already
ended tilted before). A knife-edge in MuJoCo's release of a box, left for the Isaac Sim tuning.

## Not measured yet

- Real voices in a loud room: one live sentence through the MacBook microphone was heard word for
  word; the numbers above are synthetic speech.
- The automatic stop with real speakers, who pause mid-sentence more than `say` does.
- The audience layout from 5 m away, on the showcase screen.

## Next: the move to Isaac Sim

Tornike wants the project in Isaac Sim, on a cloud GPU (Isaac Sim needs an NVIDIA RTX card). Every
node here talks through dora messages, so chat, voice, agent, skills, memory and the dashboard
carry over; `sim` and `camera` are replaced, and walking, the arm and the detector are measured
again on the new physics and images.
