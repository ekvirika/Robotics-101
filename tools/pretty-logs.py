import json
import signal
import re
import sys

LINE = re.compile(r"^(\d\d:\d\d:\d\d) (stdout|stderr)\s+(\S+): (.*)$")
# what a node writes with node.log_info / log_warn / log_error
NODE_LOG = re.compile(r"^(\d\d:\d\d:\d\d) (INFO|WARN|ERROR)\s+(\S+): (.*)$")
DORA_INTERNAL = ("daemon ", "spawner ", "dora_", "dora ")
signal.signal(signal.SIGPIPE, signal.SIG_DFL)
ANSI = re.compile(r"\x1b\[[0-9;]*m")

for raw in sys.stdin:
    line = ANSI.sub("", raw.rstrip("\n"))
    log = NODE_LOG.match(line)
    if log and not log.group(4).startswith(DORA_INTERNAL) and log.group(3) != "[dora]:":
        time, level, node, text = log.groups()
        text = text if level == "INFO" else f"{level} {text}"
        print(f"{time}  {node.rstrip(':'):<17} {text}", flush=True)
        continue
    match = LINE.match(line)
    if not match:
        if " INFO " in line and ("ready" in line or "finished" in line or "stop-after" in line):
            print(line)
        elif "ERROR" in line or "WARN" in line or "Traceback" in line:
            print(line)
        continue
    time, _, node, text = match.groups()
    if text.startswith("{"):
        try:
            payload = json.loads(text)
            text = f"{payload['level']:<5} {payload['fields']['message']}"
        except (ValueError, KeyError):
            pass
    print(f"{time}  {node:<17} {text}", flush=True)
