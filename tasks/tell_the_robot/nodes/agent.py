"""The brain as a dora node: sentences in, skill commands out, a spoken reply back (agent.py).

The model call takes seconds and the robot must keep running meanwhile, so
Agent.think() runs in a worker thread; a 100 ms tick checks whether it is done.
Robot skills run one at a time: a command goes to skills, the next waits for
its result.

"stop" (or "halt") never reaches the model: it goes straight to skills, which
stops walking and the arm at once, and the current request is dropped.

Inputs:  text      a sentence from a person (chat node, or AGENT_SCRIPT)
         result    JSON text from skills when a skill ends
         objects   the memory, with what pick would say about each object (skills)
         held      [id] in the hand, [-1] empty (skills)
         tick      100 ms: is the model's answer here yet?
Outputs: command   JSON text, one skill, to skills
         reply     what the robot says, to chat and dashboard
         log       one line per event (heard, tool call, result, reply), for the dashboard

Each skill is announced as it starts ("Looking around for the blue bottle."), and "Okay." when a
sentence is heard; AGENT_NARRATE=0 keeps it to the final reply.
AGENT_SCRIPT (agent.yml) or ROBOT_SAY (shell, wins): sentences separated by "|", sent one after
another when the robot is idle
(automatic runs without typing). Needs a key in the environment: ANTHROPIC_API_KEY, or through a proxy
ANTHROPIC_AUTH_TOKEN + ANTHROPIC_BASE_URL and AGENT_MODEL (the proxy's model name).
"""

import json
import os
from concurrent.futures import ThreadPoolExecutor

import pyarrow as pa
from anthropic import Anthropic
from dora import Node

from tasks.tell_the_robot import agent

brain = agent.Agent(Anthropic())
worker = ThreadPoolExecutor(max_workers=1)
thinking = None  # Future of agent.think()
conductor = None  # the calls of the current turn
cancelled = False  # a stop came while thinking: the answer is closed, not run
objects, held = [], None
said = os.environ.get("ROBOT_SAY") or os.environ.get("AGENT_SCRIPT", "")  # ROBOT_SAY from the shell wins
script = [s.strip() for s in said.split("|") if s.strip()]
NARRATE = os.environ.get("AGENT_NARRATE", "1") != "0"  # say each step as it starts: "Picking up the blue bottle."
node = Node()


def send(output, text):
    node.send_output(output, pa.array([text]))


def log(line):
    node.log_info(line)
    send("log", line)


def idle():
    return thinking is None and conductor is None


def start_thinking():
    global thinking
    thinking = worker.submit(brain.think)


def advance():
    """Hand out the next robot call of the turn, or give all results to the model."""
    global conductor
    call = conductor.next(objects, held)
    if call is not None:
        log(f"-> {call.name}({json.dumps(call.input)})")
        send("command", json.dumps(agent.skill_command(call)))
        if NARRATE:
            send("reply", agent.narration(call, objects))
        return
    if conductor.done:
        brain.give(conductor.results)
        conductor = None
        start_thinking()


def hear(text):
    global conductor, thinking, cancelled
    words = text.lower().replace(",", " ").replace(".", " ").replace("!", " ").split()
    if words and words[0] in ("stop", "halt"):  # "Stop. Wait..." stops too
        send("command", json.dumps({"skill": "stop"}))
        if conductor is not None:
            brain.give(conductor.stop())
        cancelled = thinking is not None  # the turn on its way: its calls are closed as stopped on arrival
        conductor = None
        send("reply", "Stopped.")
        log("heard stop: robot stopped")
        return
    if not idle():
        send("reply", "I'm still busy with the last request. Say stop to stop it.")
        return
    log(f"heard: {text}")
    if NARRATE:
        send("reply", "Okay.")
    brain.hear(text)
    start_thinking()


for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    name = event["id"]

    if name == "objects":
        objects = json.loads(event["value"][0].as_py())
    elif name == "held":
        value = event["value"].to_numpy()
        held = int(value[0]) if value[0] >= 0 else None
    elif name == "text":
        hear(event["value"][0].as_py())
    elif name == "result" and conductor is not None and conductor.waiting is not None:
        result = json.loads(event["value"][0].as_py())
        log(f"<- {result['skill']}: {'OK' if result['ok'] else 'FAILED'}: {result['message']}")
        conductor.finished(result["ok"], result["message"])
        advance()
    elif name == "tick":
        if thinking is not None and thinking.done():
            future, thinking = thinking, None
            try:
                turn = future.result()
            except Exception as error:  # the API call failed: say so, keep running
                cancelled = False
                send("reply", f"I can't think right now: {type(error).__name__}.")
                log(f"model error: {error}")
                continue
            if cancelled:
                cancelled = False
                if not turn.done:
                    brain.give(agent.Conductor(turn).stop())
                continue
            if turn.done:
                send("reply", turn.text)
                log(f"says: {turn.text}")
            else:
                conductor = agent.Conductor(turn)
                advance()
        elif idle() and script:
            hear(script.pop(0))
