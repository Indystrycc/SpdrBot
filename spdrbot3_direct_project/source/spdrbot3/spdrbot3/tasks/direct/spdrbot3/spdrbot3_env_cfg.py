# spdrbot3_env_cfg.py
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.sensors import ContactSensorCfg

from isaaclab_assets.robots.spdrbot import SPDRBOT_CFG  # isort: skip


@configclass
class EventCfg:
    """Configuration for randomization."""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "mass_distribution_params": (1.0, 3.0),
            "operation": "add",
        },
    )


@configclass
class Spdrbot3EnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 20.0
    decimation = 4
    action_scale = 1
    action_space = 12
    observation_space = 48
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,
        render_interval=1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    contact_sensor: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/.*",  # Path to base_link in cloned envs
        history_length=5,
        update_period=0.005,  # Matches sim dt = 1/200
        track_air_time=True,  # Not needed for base
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=200, env_spacing=2.0, replicate_physics=True)

    # events
    events: EventCfg = EventCfg()

    # robot
    robot: ArticulationCfg = SPDRBOT_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # reward scales
    lin_vel_reward_scale = 5.0  # Increased for more movement reward
    yaw_rate_reward_scale = 0.5
    z_vel_reward_scale = -2.0
    ang_vel_reward_scale = -0.05
    joint_torque_reward_scale = -1e-5  # Reduced punishment
    joint_accel_reward_scale = -1e-7  # Reduced punishment
    action_rate_reward_scale = -0.01
    flat_orientation_reward_scale = -5.0
    max_tilt_angle_deg = 45.0