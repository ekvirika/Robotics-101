# Tell the robot, phase 5: the brain

Goal: say a sentence, the robot does it. Claude chooses the skills from phase 4 as tools, reads
each result, decides the next step, and answers in one or two spoken sentences. It never moves a
joint. All code in `humanoid/03-g1-pick-place/`.

```bash
humanoid/tell-the-robot/demo.sh g1-agent --stop-after 120s   # robot + windows, gently
cd humanoid/03-g1-pick-place && uv run --no-sync python nodes/chat.py   # second terminal: talk to it
```

Through Exxeta's LiteLLM proxy (allowed for this project) the environment needs
`ANTHROPIC_BASE_URL=https://ai.exxeta.info`, `ANTHROPIC_AUTH_TOKEN` (your proxy token, in
`~/.zshrc`, never in the repo) and `AGENT_MODEL=claude-opus-5.5` (the proxy writes model names
with a dot). Straight against Anthropic: only `ANTHROPIC_API_KEY`.

## What was added

| File | What it is |
| --- | --- |
| `g1/agent.py` | system prompt, 5 tools, `check_call` (the safety check), `Agent` (hear, think, give), `Conductor`, `converse` |
| `nodes/agent.py` | the dora node: model call in a worker thread, skills one at a time, "stop" straight to the robot |
| `nodes/chat.py` | dynamic dora node: type sentences in a terminal, read the replies |
| `agent.yml` | the skills graph with the agent and chat instead of the commander; `./run.sh g1-agent` |
| `nodes/dashboard.py` | a conversation panel: heard, tool calls, results, replies |
| `tests/test_agent.py` | 8 tests with a scripted fake model, no API calls |
| `spikes/agent_eval.py` | 12 sentences in 5 levels, judged against the simulator's truth |
| `demo.sh`, lower rates | live demos at 3 cores at most (see the end) |

## How a sentence becomes skills

1. The sentence goes to Claude with the system prompt and 5 tools: `look_around`, `get_objects`,
   `pick`, `place`, `go_to`
2. `get_objects` returns the memory as text, with a line per object: kind, color, table, position,
   and whether `pick` could reach it, and why not
3. Every tool call passes `check_call` before the robot moves: an id the memory doesn't have,
   picking with a full hand, placing with an empty one, placing next to itself: rejected with a
   reason the model reads
4. Robot skills run one at a time; their plain-language results go back to the model
5. When the model answers without a tool call, that answer is spoken

Settings: `claude-opus-5.5`, effort `medium`, adaptive thinking, refusal fallbacks on
(`AGENT_FALLBACKS=0` turns them off). The conversation only grows, so the cached system prompt
and tools stay valid.

## Results: 12 sentences, real model, judged against the truth

| # | Level | Sentence | Result |
| --- | --- | --- | --- |
| 1 | 1 direct | Put the blue bottle on the green mark. | PASS, 1.1 cm from the mark |
| 2 | 1 | Put the yellow can on table A. | PASS |
| 3 | 1 | Put the blue bottle on table B. | PASS |
| 4 | 2 attributes | Put the can on the mark. | PASS, 0.6 cm; "It was the only can on the tables." |
| 5 | 2 | Put the red bottle next to the blue bottle. | PASS, 13 cm apart |
| 6 | 2 | Move the blue bottle to the other table. | PASS |
| 7 | 3 several steps | Put the blue bottle and the yellow box on table B. | FAIL: said both done, the bottle fell over |
| 8 | 3 | Clear the blue bottle off table A, then put the yellow box next to it. | FAIL: the box tipped over; the model explained the failed check away |
| 9 | 4 ambiguous | Put the green box on the mark. (two green boxes) | PASS: asked which one, moved nothing |
| 10 | 4 vague | I'm thirsty. | PASS: brought the only bottle to the start |
| 11 | 5 things go wrong | Put the blue bottle on the mark. (knocked out of the hand) | PASS: "it fell out of my hand while I was carrying it" |
| 12 | 5 | Put the green box that's in the middle of table A on the mark. | PASS: "21 centimeters from the nearest edge, and my hand only reaches 17" |

**10 of 12.** 4 to 12 model calls per sentence, 15 to 77 s each.

## Things learned the hard way

- **Ground the model in ids, check every call.** The model only names ids from `get_objects`; the
  safety check catches the rest before anything moves. No case in the evaluation needed it, the
  offline tests show it works.
- **Tell the model what's possible, not just what's there.** `get_objects` says which objects can
  be picked and why not. Case 12's answer ("21 cm, my hand reaches 17") comes straight from that.
- **Results written for a reader pay off.** Phase 4 wrote skill results as sentences; the model
  quotes them in its replies (cases 11, 12).
- **The model can only be as honest as its inputs.** Case 7: the second placement knocked the
  first object over, no skill noticed, the model reported success. Case 8: the memory knows where
  an object is, not whether it stands, so the model concluded a tipped box was "next to it".
  Both are perception gaps, not model errors.
- **A proxy changes names, not the code.** Exxeta's LiteLLM proxy takes `claude-opus-5.5` (dot,
  not dash), passes tools, thinking, effort and even the fallback beta. One cheap probe request
  before the evaluation found the name.
- **A key in a chat is a leaked key.** Keys go into `~/.zshrc` by the user's own hand, and get
  regenerated if they ever appear in a conversation.
- **Live windows are the heaviest load.** The live demo with the MuJoCo window and Rerun used 3.6
  cores plus the GPU; the Mac got hot before macOS recorded any warning. Now: window 30 frames
  per second (`DRAW_FPS`), head camera 10 images per second (`CAMERA_HZ`), dashboard images 2 per
  second (`IMAGE_EVERY`), and `demo.sh` stops a demo above 3 cores for 20 s. Measured with
  `demo.sh g1-skills`, look around: median 1.8 cores (was 3.6), peak 2.8, still 100% of real
  time, and the Mac stayed cool. The Rerun window itself is the biggest part, 1.2 cores while
  drawing, then the physics, 1.1.

## Still open

- **Standing or fallen over.** The memory needs each object's height (a fallen bottle is 6 cm
  high), `get_objects` should show it, and `place` should check it, also for neighbors.
- **Live demo, room 2:** `look_around` missed the yellow can that the headless evaluation found.
  Claude answered honestly ("couldn't find a yellow can"). Detection while live needs more looks
  per view, or a third viewpoint.

## Exercises

1. Read `SYSTEM` in `g1/agent.py`. Which rule produced case 9's question? Remove it, run case 9
   (`agent_eval.py 9`, a few cents), compare.
2. Why does `check_call` exist when the system prompt already says "use ids from get_objects"?
3. In case 8 the model said a failed placement worked because a fresh look showed the box nearby.
   Write one sentence for `SYSTEM` that stops that, and the memory field that would make it right.
4. Why do the tool results go back as one message with all results, instead of one message each?
5. `nodes/agent.py` runs `think()` in a thread. What would go wrong if it called it directly in the
   event loop?

## Next: phase 6, voice

Push-to-talk, Whisper on the Mac for speech to text, `say` for the replies, and the dashboard laid
out for an audience. First, the "standing or fallen over" gap from above.
