############################ SE3 ##############################
class _meta_Measure(type):
    def __getitem__(cls,key):
        # Parse name
        group_name = key.__name__
        name = "MeasureModel_" + group_name

        module = __import__("_inekf")
        return getattr(module, name)

class MeasureModel(metaclass=_meta_Measure):
    pass