# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from . import mdp

##
# Pre-defined configs
##

# from isaaclab_assets.robots.cartpole import CARTPOLE_CFG  # isort:skip


##
# Scene definition
##


@configclass
class LocomotionSceneCfg(InteractiveSceneCfg):
    """Configuration for a ava scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    '''
    This is Ava's articulation configuration.
    Guide: https://isaac-sim.github.io/IsaacLab/main/source/how-to/write_articulation_cfg.html
    this articulation config has, but is not limited to,:
    spawn:
        loads in physical object files. In this case, we are loading in a USD of Ava
        stored in the AvaUSD folder of this project
    init_state:
        the initial state of the robot as it is loaded into the scene.
        A height of 2m puts it just above the ground. This can be fine tuned
    actuators:
        The actuator configurations for each motor specified. In each ActuatorCfg, a regex
        as joint_names_expr is used to apply each config to their respective links. This pre
        vents having to duplicate data for the three motors to the n links of Ava.
    '''
    robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="../AvaUSD/Humanoid_URDF_9-10.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,  # units?
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 2.0)
        ),
        actuators={
            "all": ImplicitActuatorCfg(  # for now, implicit. Later, explicit.
                joint_names_expr=[".*"],  # motor is applied to all joints
                effort_limit_sim=400.0,
                velocity_limit_sim=100.0,
                stiffness=10.0,
                damping=1.0
            ),
        },
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_effort = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=5.0,  # replace with actual value
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # on reset, we want to randomize the position and velocities
    # of the joints by a little bit
    reset_joint_pos_vel = EventTerm(  # we should rename this
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot",
                                        joint_names=[".*"]),  # all joints
            "position_range": (-1.0, 1.0),  # radians
            "velocity_range": (-0.5, 0.5),  # rad/s ?
        },
    )

    # # NOT DONE, but something we want
    # # randomizes the whole sale of the USD
    # reset_randomize_body_scale = EventTerm(
    #     func=mdp.randomize_rigid_body_scale,
    #     mode="reset",
    #     params={
    #         "asset_cfg": SceneEntityCfg()
    #     }
    # )
    
    # # NOT DONE, but something we want
    # # applies a random COM to the robot by modifying weights
    # reset_randomize_com = EventTerm(
    #     func=mdp.randomize_rigid_body_com,
    #     mode="reset",
    #     params=
    #     ...
    # )

    # # NOT DONE, but something we want
    # # applies an external force (or torque) distributed to the body
    # # intermittently throughout the instance's lifespan
    # intermittent_external_force = EventTerm(
    #     func=mdp.apply_external_force_torque,
    #     mode="intermittent",
    #     interval_range_s=(1.0, 5.0),  # randomly at period of 1 to 5 sec
    # )



    # # NOT DONE, but something we (may) want
    # # when the robot instance is created, have it start off with a nudge
    # # as an initial velocity
    # reset_robot_push = EventTerm(
    #     func=mdp.push_by_setting_velocity,
    #     mode="reset",

    # )




@configclass
class RewardsCfg:
    """Reward terms for the MDP.
        RewTerm:
            - func : A callable from MDP module that extracts some measurement from sim state
            - weight : how much weight does the reward term carry
            - params : a dictionary of extra arguments which define 
                       how signal is measured (which joints, which target val)
    """


    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # # (2) Failure penalty
    # terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # # (3) Primary task: keep pole upright
    # pole_pos = RewTerm(
    #     func=mdp.joint_pos_target_l2,
    #     weight=-1.0,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]), "target": 0.0},
    # )
    # # (4) Shaping tasks: lower cart velocity
    # cart_vel = RewTerm(
    #     func=mdp.joint_vel_l1,
    #     weight=-0.01,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    # )
    # # (5) Shaping tasks: lower pole angular velocity
    # pole_vel = RewTerm(
    #     func=mdp.joint_vel_l1,
    #     weight=-0.005,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    # )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # # (2) Cart out of bounds
    # cart_out_of_bounds = DoneTerm(
    #     func=mdp.joint_pos_out_of_manual_limit,
    #     params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]), "bounds": (-3.0, 3.0)},
    # )


##
# Environment configuration
##


@configclass
class LocomotionEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: LocomotionSceneCfg = LocomotionSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation