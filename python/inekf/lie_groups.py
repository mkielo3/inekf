import enum
from . import _inekf

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
    return getattr(_inekf, name)


############################ SE3 ##############################
class _meta_SE3(type):
    # if we used both default arguments
    def __call__(self, *args, **kwargs):
        return _inekf.SE3_1_0(*args, **kwargs)

    def __getitem__(cls,key):
        # if we used 2nd default argument
        if isinstance(key, int) or isinstance(key, str):
            key = (key,0)
        
        # if there's 2 arguments return
        if isinstance(key, tuple) and len(key) == 2:
            return _get_class("SE3", key[0], key[1])

        raise TypeError("Invalid Options")

class SE3(metaclass=_meta_SE3):
    @staticmethod
    def wedge(xi):
        return _inekf.SE3_1_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SE3_1_0.exp(xi)

    @staticmethod
    def log(g):
        return _inekf.SE3_1_0.log_(g)

    @staticmethod
    def Ad(g):
        return _inekf.SE3_1_0.Ad_(g)


############################ SE2 ##############################
class _meta_SE2(type):
    # if we used both default arguments
    def __call__(self, *args, **kwargs):
        return _inekf.SE2_1_0(*args, **kwargs)

    def __getitem__(cls,key):
        # if we used 2nd default argument
        if isinstance(key, int) or isinstance(key, str):
            key = (key,0)
        
        # if there's 2 arguments return
        if isinstance(key, tuple) and len(key) == 2:
            return _get_class("SE2", key[0], key[1])

        raise TypeError("Invalid Options")

class SE2(metaclass=_meta_SE2):
    @staticmethod
    def wedge(xi):
        return _inekf.SE2_1_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SE2_1_0.exp(xi)

    @staticmethod
    def log(g):
        return _inekf.SE2_1_0.log_(g)

    @staticmethod
    def Ad(g):
        return _inekf.SE2_1_0.Ad_(g)


############################ SO3 ##############################
class _meta_SO3(type):
    # if we used default argument
    def __call__(self, *args, **kwargs):
        return _inekf.SO3_0(*args, **kwargs)

    def __getitem__(cls,key):
        # Return what they asked for
        if isinstance(key, int) or isinstance(key, str):
            return _get_class("SO3", key)

        raise TypeError("Invalid Options")

class SO3(metaclass=_meta_SO3):
    @staticmethod
    def wedge(xi):
        return _inekf.SO3_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SO3_0.exp(xi)

    @staticmethod
    def log(g):
        return _inekf.SO3_0.log_(g)

    @staticmethod
    def Ad(g):
        return _inekf.SO3_0.Ad_(g)


############################ SO2 ##############################
class _meta_SO2(type):
    # if we used default argument
    def __call__(self, *args, **kwargs):
        return _inekf.SO2_0(*args, **kwargs)

    def __getitem__(cls,key):
        # Return what they asked for
        if isinstance(key, int) or isinstance(key, str):
            return _get_class("SO2", key)

        raise TypeError("Invalid Options")

class SO2(metaclass=_meta_SO2):
    """Proxy for communicating with a HoloOcean world

    Instantiate this object using :meth:`holoocean.holoocean.make`.

    Args:
        agent_definitions (:obj:`list` of :class:`AgentDefinition`):
            Which agents are already in the environment

        binary_path (:obj:`str`, optional):
            The path to the binary to load the world from. Defaults to None.

    """
    @staticmethod
    def wedge(xi):
        """Maps from R^n -> algebra

        Args:
            xi (np.ndarray): Tangent vector

        Returns:
            np.ndarray: Element in Lie Algebra
        """
        return _inekf.SO2_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SO2_0.exp(xi)

    @staticmethod
    def log(g):
        return _inekf.SO2_0.log_(g)

    @staticmethod
    def Ad(g):
        return _inekf.SO2_0.Ad_(g)