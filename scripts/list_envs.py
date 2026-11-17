"""List this repository's tasks and their Isaac Lab environments.

$ISAACLAB/isaaclab.sh -p scripts/list_envs.py
"""

import importlib
import pkgutil

import gymnasium as gym

import tasks


def main():
    for module in pkgutil.iter_modules(tasks.__path__):
        task = importlib.import_module(f"tasks.{module.name}")
        if not hasattr(task, "ENV_ID"):
            continue
        spec = gym.spec(task.ENV_ID)
        summary = (task.__doc__ or "").strip().splitlines()[0]
        print(f"{module.name:16s} {task.ENV_ID:24s} {spec.kwargs['env_cfg_entry_point']}")
        print(f"{'':16s} {summary}")


if __name__ == "__main__":
    main()
