############################ Measurement Model ##############################
class _meta_Measure(type):
    def __getitem__(cls,key):
        # Parse name
        group_name = key.__name__
        name = "MeasureModel_" + group_name

        module = __import__("_inekf")
        return getattr(module, name)

class MeasureModel(metaclass=_meta_Measure):
    pass

############################ Generic Measure Model ##############################
class _meta_GenericMeasure(type):
    def __getitem__(cls,key):
        # Parse name
        group_name = key.__name__
        name = "GenericMeasureModel_" + group_name

        module = __import__("_inekf")
        return getattr(module, name)

class GenericMeasureModel(metaclass=_meta_Measure):
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
                name += "Vec" + str(key[1])
            else:
                name += key[1].__name__

            module = __import__("_inekf")
            return getattr(module, name)

class ProcessModel(metaclass=_meta_Process):
    pass