# 09 · several computers

One graph, spread over machines. The YAML says where each node runs:

- **`deploy: machine: robot`** runs on the computer called `robot`
- **`deploy: labels: gpu: "true"`** runs on any computer started with that label

Try it on one laptop, with two daemons standing in for two computers:

```bash
examples/09-distributed/local-cluster.sh
```

```
display (pid 30159):  576  computed by pid 30139
```

`sensor` and `display` run on daemon `robot`, `compute` runs on daemon `gpu-server`, and the data crosses between them.

For real machines, `cluster.yml` lists them, and `dora cluster up cluster.yml` starts a daemon on each over SSH. The addresses in it are placeholders.

- **`dora run` refuses this YAML,** it does not support `deploy:`. Use a coordinator and daemons
- **Nodes run in `_work/<id>/`** on each daemon, so `args:` paths are relative to that folder: `../../nodes/compute.py`
- **Real machines need the code too:** each machine must have this repo at the same path, or use dora's `deploy.distribute` or git sources
