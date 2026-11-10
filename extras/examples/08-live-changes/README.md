# 08 · change a running graph

A graph started with `dora start` can change while it runs. The other nodes keep going.

```bash
examples/08-live-changes/live.sh
```

The script:

1. **Starts** sensor, filter v1 (passes values ≥ 10) and display in managed mode, `dora up` + `dora start`
2. **Adds** a logger with `dora node add --from-yaml logger.yml`. The logger prints every raw sensor value, the sensor is not restarted
3. **Swaps** the filter for v2 (passes values ≥ 20) with `dora node replace filter --from-yaml filter-v2.yml`
4. **Shows** the display switching from `filter v1 (>= 10)` to `filter v2 (>= 20)` without a restart of sensor or display

- **`replace` starts the new filter first** and stops the old one only once the new one is up
- **Declare inputs when adding.** `logger.yml` has `inputs: value: sensor/value`. Adding a node without inputs and wiring it afterwards with `dora node connect` reported success in our test, but no data arrived
- **Not available with `dora run`**
