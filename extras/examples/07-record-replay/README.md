# 07 · record and replay

**Not run on this machine.** `dora record` needs the `dora-record-node` binary, which is not in the 1.1.0 release downloads and is built with Rust:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh   # installs Rust
cargo install dora-record-node
```

Then:

```bash
cd examples/07-record-replay
dora record dataflow.yml --topics camera/image -o out/run.drec   # Ctrl-C after ~10 s
dora replay out/run.drec --replace camera --speed 2              # camera replaced by the recording
```

- **Record:** saves every message on `camera/image` into one `.drec` file
- **Replay:** the recorded frames feed the detector instead of the live camera, here at twice the speed
- **Example:** record a run where detection failed, change `DETECTOR` or its thresholds, replay the same frames and compare. Both runs see identical input
- **`--speed 0`** replays as fast as possible, `--loop` repeats forever
