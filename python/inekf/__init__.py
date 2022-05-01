from inekf.lie_groups import SO2, SO3, SE2, SE3
from inekf.base import MeasureModel, ProcessModel, InEKF

from ._inekf import ERROR

# import inertial objects
from ._inekf import InertialProcess, DVLSensor, DepthSensor

# import SE2 objects
from ._inekf import OdometryProcess, OdometryProcessDynamic, LandmarkSensor, GPSSensor


# import fake objects for documentation when docs are being built
try:
    __sphinx_build__ # This will fail if docs not being built
    from inekf.base import ERROR
    from inekf.lie_groups import LieGroup
except:
    pass