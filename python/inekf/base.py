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
class InEKF:
    def __init__(self, pModel, x0, error):
        # Parse name
        group_name = pModel.__class__.__mro__[-3].__name__
        name = "InEKF_" + group_name.split('_',1)[1]

        # save for later
        self.pModel = pModel
        self.base = getattr(_inekf, name)(pModel, x0, error)

        # save measurement models so they don't go out of scope
        self.mModels = {}

    def Predict(self, *args, **kwargs):
        return self.base.Predict(*args, **kwargs)

    def Update(self, *args, **kwargs):
        return self.base.Update(*args, **kwargs)

    def addMeasureModel(self, name, model):
        self.mModels[name] = model
        return self.base.addMeasureModel(name, model)

    @property
    def state(self):
        return self.base.state

    @state.setter
    def state(self, state):
        self.base.state = state

############################ Measurement Model ##############################
class _meta_Measure(type):
    def __getitem__(cls,key):
        # Parse name
        name = "MeasureModel_" + _parse_group(key)

        return getattr(_inekf, name)

class MeasureModel(metaclass=_meta_Measure):
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

class ProcessModel(metaclass=_meta_Process):
    pass