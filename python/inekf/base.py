from . import _inekf

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
        group_name = key.__name__
        name = "MeasureModel_" + group_name

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
            name = "ProcessModel_" + key[0].__name__ + "_"
            if isinstance(key[1], str):
                name += key[1]
            elif isinstance(key[1], int):
                if key[1] == -1:
                    name += "Vec" + "D"
                else:
                    name += "Vec" + str(key[1])
            else:
                name += key[1].__name__


            return getattr(_inekf, name)

class ProcessModel(metaclass=_meta_Process):
    pass