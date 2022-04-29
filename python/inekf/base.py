from . import _inekf

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
    """    
    Description for class
    """

    @property
    def state(self):
        """    
        Description for class
        """
        pass

    def predict(self, u, dt=1):
        pass

    def update(self, name, m):
        pass

    def addMeasureModel(self, name, m):
        pass

    

############################ Measurement Model ##############################
class _meta_Measure(type):
    def __getitem__(cls,key):
        # Parse name
        name = "MeasureModel_" + _parse_group(key)

        return getattr(_inekf, name)

# This is a dummy class, used to template and return C++ class and for documentation
class MeasureModel(metaclass=_meta_Measure):
    """
    
    """
    def __init__(self, b, M, error):
        pass

    def setHandb(self, b):
        pass

    def processZ(self, z, state):
        pass

    def makeHError(self, state, iekfERROR):
        pass

    def calcV(self, z, state):
        pass

    def calcSInverse(self, state):
        pass

    @property
    def H(self):
        pass

    @property
    def H_error(self):
        pass

    @property
    def M(self):
        pass

    @property
    def error(self):
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
    
    def f(self, u, dt, state):
        pass

    def makePhi(self, u, dt, state):
        pass

    @property
    def Q(self):
        pass