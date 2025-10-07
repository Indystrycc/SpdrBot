"""Configuration for the Spdrbot robot.

The following configuration parameters are available:

"""

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils

SPDRBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="C:/spdrbot3/spdrbot3/spdr.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
            disable_gravity=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={
            "Revolute_110": 0.0,
            "Revolute_111": 0.5,
            "Revolute_112": 0.0,
            "Revolute_113": 0.0,
            "Revolute_114": 0.5,
            "Revolute_115": 0.0,
            "Revolute_116": 0.0,
            "Revolute_117": 0.5,
            "Revolute_118": 0.0,
            "Revolute_119": 0.0,
            "Revolute_120": 0.5,
            "Revolute_121": 0.0,
        },
        joint_vel={
            "Revolute_110": 0.0,
            "Revolute_111": 0.0,
            "Revolute_112": 0.0,
            "Revolute_113": 0.0,
            "Revolute_114": 0.0,
            "Revolute_115": 0.0,
            "Revolute_116": 0.0,
            "Revolute_117": 0.0,
            "Revolute_118": 0.0,
            "Revolute_119": 0.0,
            "Revolute_120": 0.0,
            "Revolute_121": 0.0,
        },
    ),
    actuators={
        "leg_joints": ImplicitActuatorCfg(
            joint_names_expr=[
                "Revolute_110",
                "Revolute_111",
                "Revolute_112",
                "Revolute_113",
                "Revolute_114",
                "Revolute_115",
                "Revolute_116",
                "Revolute_117",
                "Revolute_118",
                "Revolute_119",
                "Revolute_120",
                "Revolute_121",
            ],
            effort_limit=5.0,
            velocity_limit=1,
            stiffness=40.0,
            damping=1.0,
        ),
    },
    soft_joint_pos_limit_factor=2,
)
"""Configuration of Spdrbot robot using implicit actuators."""