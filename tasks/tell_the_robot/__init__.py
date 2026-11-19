"""Tell the robot: a language model turns a spoken or typed request into skills
(look_around, go_to, pick, place) and reads their results.

Unlike the other tasks it has no fixed mission, so scripts/run_task.py does
not run it: the dora graph does (dataflow.yml on Isaac Lab, dataflow_fake.yml
on any computer). The skills alone are tested on the stand-in robot
(tests/test_tell_the_robot.py).
"""

import gymnasium as gym

ENV_ID = "G1-Tell-The-Robot-v0"

gym.register(
    id=ENV_ID,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"{__name__}.env_cfg:TellTheRobotEnvCfg"},
)
