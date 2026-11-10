# 01 · hello graph

The smallest useful dora graph: three programs, one YAML file.

```
timer (every 500 ms) → counter → count ──┬──→ printer
                                          └──→ doubler → doubled → printer
```

```bash
./run.sh hello --stop-after 10s
```

You see `count: 1`, `doubled: 2`, `count: 2`, `doubled: 4` and so on.

- **No node imports another.** `printer.py` knows nothing about `counter.py`, the connection lives in `dataflow.yml`
- **One sender, two receivers:** `count` goes to the doubler and the printer at the same time
- **Try:** change `dora/timer/millis/500` to `dora/timer/millis/100` and run again, the numbers come five times faster. No code change
