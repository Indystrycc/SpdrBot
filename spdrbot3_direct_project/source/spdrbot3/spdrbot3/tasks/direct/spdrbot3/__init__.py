# source/spdrbot3/spdrbot3/tasks/direct/spdrbot3/__init__.py
# Copyright (c) 2022-2025, The Isaac Lab Project Developers[](https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##


gym.register(
    id="Template-Spdrbot3-Direct-v0",
    entry_point=f"{__name__}.spdrbot3_env:Spdrbot3Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.spdrbot3_env_cfg:Spdrbot3EnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
        "skrl_amp_cfg_entry_point": f"{agents.__name__}:skrl_amp_cfg.yaml",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)