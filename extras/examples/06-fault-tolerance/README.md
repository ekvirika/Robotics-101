# 06 · fault tolerance

The YAML says what happens when a node crashes, hangs or falls behind.

```bash
./run.sh faults --stop-after 30s
```

| Node | What it does | YAML that handles it |
| --- | --- | --- |
| `sensor` | goes silent for 3 s after 8 readings, crashes after 30 | `restart_policy: on-failure`, `restart_delay: 0.5`, `max_restart_delay: 4.0`, `max_restarts: 5` |
| `monitor` | reports when readings stop | `input_timeout: 2.0` on its input |
| `slow_consumer` | needs 0.35 s per reading while readings come every 0.1 s | `queue_size: 2`, `queue_policy: drop_oldest` |

What you see:

- **Restarts with growing pauses:** `waiting 500ms before restart (attempt 1)`, then 1 s, 2 s, 4 s. The monitor reports a new sensor instance each time
- **Timeout:** `monitor: input reading closed, no reading for 2 s` during the silence, then readings flow again
- **Skipping:** `slow_consumer: got 7, skipped 2 stale readings`, it always works on recent data instead of a growing queue

- **Restarts work under plain `dora run`,** no coordinator needed
- **`health_check_interval: 1.0`** at the top of the YAML makes the timeout check run every second. With the 5 s default the timeout fires late
