import os
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass

# Path to the USD you generated via xacro
JETAUTO_USD_PATH = os.path.join(
    os.path.expanduser("~"), "projects", "neuro-flow", "assets", "jet_auto_full", "jetauto_pro_complete" "jetauto_pro_complete.usd"
)

@configclass
class JetAutoAssetCfg(ArticulationCfg):
    prim_path = "{ENV_REGEX_NS}/Robot"
    spawn = sim_utils.UsdFileCfg(
        usd_path=JETAUTO_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=0.001, # Prevents "explosions"
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, # Essential to stop flapping
            solver_position_iteration_count=64, # High precision for gears
            solver_velocity_iteration_count=4,
            fix_root_link=True,
        ),
    )
    init_state = ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={"joint[1-5]": 0.0, "r_joint": 0.01},
    )