import gymnasium as gym
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import CameraCfg
from .jetauto_env_cfg import JetAutoAssetCfg
from .jetauto_pick_cfg import CubeCfg

@configclass
class JetAutoActionsCfg:
    # EE Control: W/A/S/D moves the wrist in X-Y
    arm = DifferentialInverseKinematicsActionCfg(
        asset_name="robot", joint_names=["joint1", "joint2", "joint3", "joint4", "joint5"],
        body_name="link5", controller=DifferentialIKControllerCfg(ik_method="dls")
    )
    # Option B: Software coupling for the gripper
    gripper = JointPositionActionCfg(
        asset_name="robot", 
        joint_names=["r_joint", "l_joint", "l_in_joint", "r_in_joint", "l_out_joint", "r_out_joint"],
        joint_ids_multipliers={"r_joint": 1.0, "l_joint": -1.0, "l_in_joint": -1.0, "r_in_joint": -1.0, "l_out_joint": 1.0, "r_out_joint": 1.0}
    )

@configclass
class JetAutoPickEnvCfg(ManagerBasedRLEnvCfg):
    def __post_init__(self):
        self.scene.robot = JetAutoAssetCfg()
        self.scene.cube = CubeCfg()
        # Wrist camera for Eye-in-Hand picking
        self.scene.wrist_camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/link5/wrist_camera",
            height=240, width=320, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0),
        )
        super().__post_init__()

gym.register(id="JetAuto-Pick-v0", entry_point="isaaclab.envs:ManagerBasedRLEnv", kwargs={"env_cfg_entry_point": JetAutoPickEnvCfg})