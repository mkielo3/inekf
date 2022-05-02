from inekf.lie_groups import SO2, SO3, SE2, SE3
from inekf.base import MeasureModel, ProcessModel, InEKF

from ._inekf import ERROR

# import inertial objects
from ._inekf import InertialProcess, DVLSensor, DepthSensor

# import SE2 objects
from ._inekf import OdometryProcess, OdometryProcessDynamic, LandmarkSensor, GPSSensor


# import fake objects for documentation when docs are being built
# We need this since we don't want to have to compile the C++ side to 
# build the documentation
import os
if bool(os.getenv('SPHINX_BUILD')):
    from inekf.base import ERROR
    from inekf.lie_groups import LieGroup
    from inekf._docs import InertialProcess, DVLSensor, DepthSensor
    from inekf._docs import OdometryProcess, OdometryProcessDynamic, GPSSensor, LandmarkSensor