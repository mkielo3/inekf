import enum

class Eigen(enum.Enum):
    Dynamic = -1

################# HELPER CLASS TO FIND CLASSES FROM STRING ###################
def _get_class(group, param1, param2=None):
    # handle dynamic types
    param1 = "D" if param1 in ["D", -1, Eigen.Dynamic] else param1
    param2 = "D" if param2 in ["D", -1, Eigen.Dynamic] else param2

    # put together name of class made in pybind
    name = f"{group}_{param1}"
    if param2 is not None:
        name += f"_{param2}"

    # return
    module = __import__("_inekf")
    return getattr(module, name)


############################ SE3 ##############################
class _meta_SE3(type):
    # if we used both default arguments
    __name__ = "SE3_1_0"
    def __call__(self, *args, **kwargs):
        return _inekf.SE3_1_0(*args, **kwargs)

    def __getitem__(cls,key):
        # if we used 2nd default argument
        if isinstance(key, int):
            key = (key,0)
        
        # if there's 2 arguments return
        if isinstance(key, tuple) and len(key) == 2:
            return _get_class("SE3", key[0], key[1])

        raise TypeError("Invalid Options")

class SE3(metaclass=_meta_SE3):
    pass


############################ SE2 ##############################
class _meta_SE2(type):
    # if we used both default arguments
    __name__ = "SE2_1_0"
    def __call__(self, *args, **kwargs):
        return _inekf.SE2_1_0(*args, **kwargs)

    def __getitem__(cls,key):
        # if we used 2nd default argument
        if isinstance(key, int):
            key = (key,0)
        
        # if there's 2 arguments return
        if isinstance(key, tuple) and len(key) == 2:
            return _get_class("SE2", key[0], key[1])

        raise TypeError("Invalid Options")

class SE2(metaclass=_meta_SE2):
    pass


############################ SO3 ##############################
class _meta_SO3(type):
    # if we used default argument
    __name__ = "SO3_0"
    def __call__(self, *args, **kwargs):
        return _inekf.SO3_0(*args, **kwargs)

    def __getitem__(cls,key):
        # Return what they asked for
        if isinstance(key, int):
            return _get_class("SO3", key)

        raise TypeError("Invalid Options")

class SO3(metaclass=_meta_SO3):
    pass


############################ SO2 ##############################
class _meta_SO2(type):
    # if we used default argument
    __name__ = "SO2_0"
    def __call__(self, *args, **kwargs):
        return _inekf.SO2_0(*args, **kwargs)

    def __getitem__(cls,key):
        # Return what they asked for
        if isinstance(key, int):
            return _get_class("SO2", key)

        raise TypeError("Invalid Options")

class SO2(metaclass=_meta_SO2):
    pass