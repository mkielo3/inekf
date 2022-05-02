from . import _inekf
import enum

# This isn't used except for documentation
class ERROR(enum.Enum):
    """Type of invariant error. Has options for left or right.
    """
    #: Left error
    LEFT = 0
    #: Right error
    RIGHT = 1

################# HELPER CLASSES FOR GETTING GROUP NAMES ###################
def _parse_group(key):
    name = key.__name__

    # If we got one of the C++ classes
    if "_" in name:
        return name
    # If we got one of our wrapper classes
    elif name[0:2] == "SO":
        return name + "_0"
    elif name[0:2] == "SE":
        return name + "_1_0"
    else:
        raise TypeError("Inheriting from that class not allowed")

def _parse_control(key):
    # If something like "Vec3" was passed in
    if isinstance(key, str):
        return key
    # If an integer was passed in
    elif isinstance(key, int):
        # If it's a dynamic integer
        if key == -1:
            return "Vec" + "D"
        # Otherwise assume it's that long
        else:
            return "Vec" + str(key)

    # Otherwise it must've actually been one of our Lie Groups
    else:
        return _parse_group(key)

########################### InEKF Filter Class ##############################
class _meta_InEKF(type):
    def __call__(self, pModel, x0, error):
        # Parse name
        group_name = pModel.__class__.__mro__[-3].__name__
        name = "InEKF_" + group_name.split('_',1)[1]

        # Get the class
        iekf_class = getattr(_inekf, name)
        
        # Make small wrapper to save measure models 
        # so python doesn't garbage collect
        def helper(self, name, m):
            self.mModels[name] = m
            self._addMeasureModel(name, m)

        setattr(iekf_class, "addMeasureModel", helper)

        iekf = iekf_class(pModel, x0, error)
        
        # Save the process model as well
        iekf.pModel = pModel
        iekf.mModels = {}

        return iekf

# This is a dummy class, used to template and return C++ class and for documentation
class InEKF(metaclass=_meta_InEKF):
    """The Invariant Extended Kalman Filter

    Args:
        pModel (:class:`~inekf.ProcessModel`): Process model
        state (:class:`inekf.LieGroup`): Initial state, must be of same group that process model uses and must be uncertain
        error (:obj:`~inekf.base.ERROR`): Right or left invariant error
    """
    def __init__(self, pModel, state, error):
        pass

    @property
    def state(self):
        """Current state estimate. May be read or wrriten, but can't be edited in place.
                
        Returns:
            :class:`inekf.LieGroup`: state
        """
        pass

    @state.setter
    def state(self, input):
        pass

    def predict(self, u, dt=1):
        """Prediction Step.
         
        Args:
            u (control): Must be same as what process model uses.
            dt (:obj:`float`): Delta t. Used sometimes depending on process model. Defaults to 1.
        
        Returns:
            :class:`inekf.LieGroup`: State estimate
         """
        pass

    def update(self, name, m):
        """Update Step.
        
        Args:
            name (:obj:`str`): Name of measurement model.
            z (:obj:`np.ndarray`): Measurement. May vary in size depending on how measurement model processes it.
        
        Returns:
            :class:`inekf.LieGroup`: State estimate.
         """
        pass

    def addMeasureModel(self, name, m):
        """Add measurement model to the filter.
         
        Args:
            name (:obj:`str`): Name of measurement model.
            m (:class:`~inekf.MeasureModel`): A measure model pointer, templated by the used group.
         """
        pass


############################ Measurement Model ##############################
class _meta_Measure(type):
    def __getitem__(cls,key):
        # Parse name
        name = "MeasureModel_" + _parse_group(key)

        return getattr(_inekf, name)

# This is a dummy class, used to template and return C++ class and for documentation
class MeasureModel(metaclass=_meta_Measure):
    """Base class measure model. Written to be inherited from, but in most cases this class will be sufficient.
    More information on inheriting can be seen in :ref:`extend`.

    We have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`.

    Templates:

    - ``Group`` State's group that is being tracked, of type :class:`inekf.LieGroup`.
    
    """
    def __init__(self, b, M, error):
        """Construct a new Measure Model object, automatically creating H. Should be used most of the time.
        
        Args: 
            b (:obj:`np.ndarray`): b vector from measurement model. Will be used to create H.
            M (:obj:`np.ndarray`): Measurement covariance.
            error (:obj:`~inekf.base.ERROR`) Type of invariant measurement (right or left).
        """
        pass

    def setHandb(self, b):
        """Sets measurement vector b and recreates H accordingly. Useful if vector b isn't constant.
        
        Args: 
            b (:obj:`np.ndarray`): Measurement model b
        """
        pass

    def processZ(self, z, state):
        """Process measurement before putting into InEKF. Can be used to change frames, convert r/b->x/y, or append 0s.
        By default is used to append zeros/ones onto it according to b vector set. Called first in update step.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement
            state (:class:`inekf.LieGroup`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Processed measurement.
        """
        pass

    def makeHError(self, state, iekfERROR):
        """Sets and returns H_error for settings where filter error type != measurement error type. 
        Done by multiplying H by adjoint of current state estimate. Called second in update step.
        
        Args: 
            state (:class:`inekf.LieGroup`): Current state estimate.
            iekfERROR (:obj:`~inekf.base.ERROR`): Type of filter error.
        
        Returns:
            :obj:`np.ndarray`: H_error 
        """
        pass

    def calcV(self, z, state):
        """Computes innovation based on measurement model. Called third in the update step.
        
        Args: 
            z (:obj:`np.ndarray`): Measurement.
            state (:class:`inekf.LieGroup`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Truncated innovation.
        """
        pass

    def calcSInverse(self, state):
        """Calculate inverse of measurement noise S, using H_error. Called fourth in the update step.
        
        Args: 
            state (:class:`inekf.LieGroup`): Current state estimate.
        
        Returns:
            :obj:`np.ndarray`: Inverse of measurement noise. 
        """
        pass

    @property
    def H(self):
        """Linearized matrix H. Will be automatically created from b in constructor unless overriden.
        May be read or written, but not modified in place.
                
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def H_error(self):
        """This is the converted H used in InEKF if it's a right filter with left measurement or vice versa. 
        Used in calcSInverse if overriden. Probably won't need to be overwritten.
        May be read or written, but not modified in place.
                
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def M(self):
        """Measurement covariance.
                
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def error(self):
        """Type of error of the filter (right/left)
                
        Returns:
            :obj:`~inekf.base.ERROR`
        """
        pass

    @property
    def b(self):
        """b vector used in measure model.
                
        Returns:
            :obj:`np.ndarray`
        """
        pass


############################ Process Model ##############################
class _meta_Process(type):
    def __getitem__(cls,key):
        # if only one thing was passed to us
        if not isinstance(key, tuple):
            key = (key, key)

        # Parse name
        if isinstance(key, tuple) and len(key) == 2:
            name = "ProcessModel_" + _parse_group(key[0]) + "_" + _parse_control(key[1])

            return getattr(_inekf, name)

# This is a dummy class, used to template and return C++ class and for documentation
class ProcessModel(metaclass=_meta_Process):
    """Base class process model. Must be inheriting from, base class isn't implemented.
    More information on inheriting can be seen in :ref:`extend`.

    We have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`.

    Templates:

    - ``Group`` State's group that is being tracked, of type :class:`inekf.LieGroup`.
    - ``U`` Form of control. Can be either a group of :class:`inekf.LieGroup`, or a vector. 
      Vectors can be used for example by "Vec3" or 3 for a vector of size 3. 
      -1, "D", or "VecD" for dynamic control size.
    """
    def f(self, u, dt, state):
        """Propagates state forward one timestep. Must be overriden, has no default implementation.
        
        Args: 
            u (control): Control
            dt (:obj:`float`): Delta time
            state (:class:`inekf.LieGroup`): Current state
        
        Returns:
            :class:`inekf.LieGroup`: Updated state estimate
        """
        pass

    def makePhi(self, u, dt, state):
        """Make a discrete time linearized process model matrix, with $\Phi = \exp(A\Delta t)$. Must be overriden, has no default implementation.
        
        Args: 
            u (control): Control
            dt (:obj:`float`): Delta time
            state (:class:`inekf.LieGroup`): Current state estimate (shouldn't be needed unless doing an "Imperfect InEKF")
            error (:obj:`~inekf.base.ERROR`): Right or left error. Function should be implemented to handle both.
        
        Returns:
            :obj:`np.ndarray`: Phi
        """
        pass

    @property
    def Q(self):
        """Process model covariance. May be read or written, but not modified in place.
        
        Returns:
            :obj:`np.ndarray`"""
        pass