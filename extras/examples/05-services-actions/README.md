# 05 · services and actions

ROS 2 has three ways for programs to talk. dora builds all three from normal outputs plus an id in the message metadata.

```bash
./run.sh services --stop-after 30s
```

| Pattern | Here | How |
| --- | --- | --- |
| Topic | timers, every input | one sends, every connected node receives |
| Service | planner asks the gripper "how wide are you open?" | `send_service_request` adds a `request_id`, the server sends it back with `send_service_response` |
| Action | mission sends "drive to shelf 4", then "drive to dock" | a `goal_id`, progress on `feedback`, a final `result` with `goal_status` |

What you see:

```
planner  -> asks gripper width   request_id=fd6720f0
planner  <- width 66.3 mm      request_id=fd6720f0
mission  -> goal: drive to shelf 4, 3.0 m
mission  <- feedback: 2.75 m left to shelf 4
mission  <- result: shelf 4 succeeded
mission  -> goal: drive to dock, 6.0 m
mission  -> cancel dock
mission  <- result: dock canceled
```

- **The server must pass the metadata back,** that is how the client recognises its answer
- **Cancel is just another output** from client to server
