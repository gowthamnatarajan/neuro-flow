from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

@configclass
class CubeCfg(RigidObjectCfg):
    prim_path = "{ENV_REGEX_NS}/Cube"
    spawn = sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    )
    init_state = RigidObjectCfg.InitialStateCfg(pos=(0.4, 0.0, 0.025))