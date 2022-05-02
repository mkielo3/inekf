from inekf.base import ProcessModel, MeasureModel
from inekf.lie_groups import SE2, SE3

# EVERYTHING IN THIS FILE IS DUMMY CLASSES, USED FOR DOCUMENTATION ONLY

################## Inertial Models ##################
class InertialProcess(ProcessModel[SE3[2,6], 6]):
    """Inertial process model. Integrates IMU measurements and tracks biases. Requires "Imperfect InEKF" since biases 
    don't fit into Lie group structure.
    """

    def f(self, u, dt, state):
        """Overriden from base class. Integrates IMU measurements.
        
        Args: 
            u (:obj:`np.ndarray`): 6-Vector. First 3 are angular velocity, last 3 are linear acceleration.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE3[2,6]`): Current state
        
        Returns:
            :class:`inekf.SE3[2,6]`: Updated state estimate
        """
        pass

    def makePhi(self, u, dt, state):
        """Overriden from base class. Since this is used in an "Imperfect InEKF", both left and right versions are slightly state dependent.
        
        Args: 
            u (:obj:`np.ndarray`): 6-Vector. First 3 are angular velocity, last 3 are linear acceleration.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE3[2,6]`): Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
            error (:obj:`~inekf.base.ERROR`): Right or left error. Function should be implemented to handle both.
        
        Returns:
            :obj:`np.ndarray`: Phi
        """
        pass

    def setGyroNoise(self, std):
        """Set the gyro noise. Defaults to 0 if not set.
        
        Args:
            std (:obj:`float`): Gyroscope standard deviation
        """
        pass

    def setAccelNoise(self, std):
        """Set the accelerometer noise. Defaults to 0 if not set.
        
        Args:
            std (:obj:`float`): Accelerometer standard deviation
        """
        pass

    def setGyroBiasNoise(self, std):
        """Set the gryo bias noise. Defaults to 0 if not set.
        
        Args:
            std (:obj:`float`): Gyroscope bias standard deviation
        """
        pass

    def setAccelBiasNoise(self, std):
        """Set the accelerometer bias noise. Defaults to 0 if not set.
        
        Args:
            std (:obj:`float`): Accelerometer bias standard deviation
        """
        pass


class DepthSensor(MeasureModel[SE3[2,6]]):
    """Pressure/Depth sensor measurement model for use with inertial process model. Uses 
    pseudo-measurements to fit into a left invariant measurement model.

    Args:
        std (:obj:`float`): The standard deviation of a measurement.
    """
    def __init__(self, std):
        pass

    def processZ(self, z, state):
        """Overriden from the base class. Inserts psuedo measurements for the x and y value to fit the invariant measurement.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement
            state (:class:`inekf.SE3[2,6]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Processed measurement.
        """
        pass

    def calcSInverse(self, state):
        """Overriden from base class. Calculate inverse of measurement noise S, using the Woodbury Matrix Identity
        
        Args: 
            state (:class:`inekf.SE3[2,6]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Inverse of measurement noise. 
        """
        pass

    def setNoise(self, std):
        """Set the measurement noise
        
        Args:
            std (:obj:`float`): The standard deviation of the measurement.
        """
        pass


class DVLSensor(MeasureModel[SE3[2,6]]):
    """DVL sensor measurement model for use with inertial process model.

    There's a number of available constructors, see :cpp:class:`InEKF::DVLSensor` for a list of all of them.
    """
    def processZ(self, z, state):
        """Overriden from base class. Takes in a 6 vector with DVL measurement as first 3 elements and IMU as last three 
        and converts DVL to IMU, then makes it the right size and passes it on.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement
            state (:class:`inekf.SE3[2,6]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Processed measurement.
        """
        pass

    def setNoise(self, std_dvl, std_imu):
        """Set the noise covariances.
        
        Args:
            std_dvl (:obj:`float`): Standard deviation of DVL measurement.
            std_imu (:obj:`float`): Standard deviation of gyropscope measurement (needed b/c we transform frames).
        """
        pass


################## SE2 Models ##################
class OdometryProcess(ProcessModel[SE2, SE2]):
    """Odometry process model with single column.

    There's a number of available constructors, see :cpp:class:`InEKF::OdometryProcess` for a list of all of them.
    """

    def f(self, u, dt, state):
        """Overriden from base class. Propagates the model $X_{t+1} = XU$
        
        Args: 
            u (:class:`inekf.SE2`): Rigid body transformation of vehicle since last timestep.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE2`): Current state
        
        Returns:
            :class:`inekf.SE2`: Updated state estimate
        """
        pass

    def makePhi(self, u, dt, state):
        """Overriden from base class. If right, this is the identity. If left, it's the adjoint of U.
        
        Args: 
            u (:class:`inekf.SE2`): Rigid body transformation of vehicle since last timestep.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE2`): Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
            error (:obj:`~inekf.base.ERROR`): Right or left error. Function should be implemented to handle both.
        
        Returns:
            :obj:`np.ndarray`: Phi
        """
        pass

    def setQ(self, q):
        """Set Q from a variety of sources

        Args:
            q (:obj:`np.ndarray` or :obj:`float`): Can be a float, 3-vector, or 3x3-matrix. Sets the covariance Q accordingly.
        """
        pass


class OdometryProcessDynamic(ProcessModel[SE2[-1,0], SE2]):
    """Odometry process model with variable number of columns, for use in SLAM on SE2.

    There's a number of available constructors, see :cpp:class:`InEKF::OdometryProcessDynamic` for a list of all of them.
    """
    def f(self, u, dt, state):
        """Overriden from base class. Propagates the model $X_{t+1} = XU$. Landmarks are left as is.
        
        Args: 
            u (:class:`inekf.SE2`): Rigid body transformation of vehicle since last timestep.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE2[-1,0]`): Current state
        
        Returns:
            :class:`inekf.SE2[-1,0]`: Updated state estimate
        """
        pass

    def makePhi(self, u, dt, state):
        """Overriden from base class. If right, this is the identity. If left, it's the adjoint of U. 
        Landmark elements are the identity in both versions of Phi.
        
        Args: 
            u (:class:`inekf.SE2`): Rigid body transformation of vehicle since last timestep.
            dt (:obj:`float`): Delta time
            state (:class:`inekf.SE2[-1,0]`): Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
            error (:obj:`~inekf.base.ERROR`): Right or left error. Function should be implemented to handle both.
        
        Returns:
            :obj:`np.ndarray`: Phi
        """
        pass

    def setQ(self, q):
        """Set Q from a variety of sources

        Args:
            q (:obj:`np.ndarray` or :obj:`float`): Can be a float, 3-vector, or 3x3-matrix. Sets the covariance Q accordingly.
        """
        pass


class GPSSensor(MeasureModel[SE2[-1,0]]):
    """GPS Sensor for use in SE2 SLAM model.

    Args:
        std (:obj:`float`): The standard deviation of a measurement.
    """
    def __init__(self, std):
        pass

    def processZ(self, z, state):
        """Overriden from the base class. Needed to fill out H/z with correct number of columns based on number of landmarks in state.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement
            state (:class:`inekf.SE2[-1,0]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Processed measurement.
        """
        pass


class LandmarkSensor(MeasureModel[SE2[-1,0]]):
    """Landmark sensor used in SLAM on SE2

    Args:
        std_r (:obj:`float`): Range measurement standard deviation
        std_b (:obj:`float`): Bearing measurement standard deviation
    """
    def __init__(self, std_r, std_b):
        pass

    def sawLandmark(idx, state):
        """Sets H based on what landmark was recently seen.
        
        Args:
            idx Index of landmark recently seen.
            state Current state estimate. Used for # of landmarks.
        """
        pass

    def calcMahDist(z, state):
        """Calculates Mahalanobis distance of having seen a certain landmark. Used for data association.
        
        Args:
            z (:obj:`np.ndarray`): Range and bearing measurement
            state (:obj:`inekf.SE2[-1,0]`): Current state estimate

        Returns:
            :obj:`float`: Mahalanobis distance
        """
        pass

    def processZ(self, z, state):
        """Overriden from base class. Converts r,b -> x,y coordinates and shifts measurement covariance. Then fills out z accordingly.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement
            state (:class:`inekf.SE2[-1,0]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Processed measurement.
        """
        pass

    def calcSInverse(self, state):
        """Overriden from base class. If using RInEKF, takes advantage of sparsity of H to shrink matrix multiplication. 
        Otherwise, operates identically to base class.
        
        Args: 
            state (:class:`inekf.SE2[-1,0]`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Inverse of measurement noise. 
        """
        pass

    def makeHError(self, state, iekfERROR):
        """Overriden from base class. Saves filter error for later use, then calls base class.
        
        Args: 
            state (:class:`inekf.SE2[-1,0]`): Current state estimate.
            iekfERROR (:obj:`~inekf.base.ERROR`): Type of filter error.
        
        Returns:
            :obj:`np.ndarray`: H_error 
        """
        pass