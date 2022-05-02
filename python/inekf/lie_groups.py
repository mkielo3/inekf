import enum
from . import _inekf

class Eigen(enum.Enum):
    Dynamic = -1

# This class doesn't do anything, but is used for documentation purposes
class LieGroup:
    @property
    def R(self):
        """Gets rotational component of the state.

        Returns:
            :class:`inekf.SO2`
        """
        pass

    @property
    def uncertain(self):
        """Returns whether object is uncertain, ie if it has a covariance.

        Returns:
            :obj:`bool`
        """
        pass

    @property
    def mat(self):
        """Get actual group element.
        
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def cov(self):
        """Get covariance of group element.
        
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def aug(self):
        """Get additional Euclidean state of object.
        
        Returns:
            :obj:`np.ndarray`
        """
        pass


    @R.setter
    def R(self, input):
        pass

    @uncertain.setter
    def uncertain(self, input):
        pass

    @mat.setter
    def mat(self, input):
        pass

    @cov.setter
    def cov(self, input):
        pass

    @aug.setter
    def aug(self, input):
        pass


    @property
    def inverse(self):
        """Invert group element. Augmented portion and covariance is dropped.
        
        Returns:
            :class:`inekf.LieGroup`
        """
        pass

    @property
    def Ad(self):
        """Get adjoint of group element.
        
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @property
    def log(self):
        """Move this element from group -> algebra -> R^n
        
        Returns:
            :obj:`np.ndarray`
        """
        pass


    @staticmethod
    def wedge(xi):
        """Move element in R^n to the Lie algebra.
        
        Args:
            xi (:obj:`np.ndarray`): Tangent vector

        Returns:
            :obj:`np.ndarray`
        """ 
        pass

    @staticmethod
    def exp(xi):
        """Move an element from R^n -> algebra -> group
        
        Args:
            xi (:obj:`np.ndarray`): Tangent vector
        
        Returns:
            :obj:`inekf.LieGroup`
        """
        pass

    @staticmethod
    def log_(g):
        """Move an element from group -> algebra -> R^n
        
        Args:
            g (:class:`inekf.LieGroup`): Group element
        
        Returns:
            :obj:`np.ndarray`
        """
        pass

    @staticmethod
    def Ad_(g):
        """Compute the linear map Adjoint
        
        Args:
            g (:class:`inekf.LieGroup`): Element of group
        
        Returns:
            :obj:`np.ndarray`
        """
        pass


    def addAug(self, a, sigma):
        """Adds an element to the augmented Euclidean state. Only usable if A = Eigen::Dynamic.
        
        Args:
            x (:obj:`float`): Variable to add.
            sigma (:obj:`float`): Covariance of element. Only used if state is uncertain.
        """
        pass

    def __matmul__(self, rhs):
        """Combine transformations. Augmented states are summed.
        
        Args:
            rhs (:class:`inekf.LieGroup`): Right hand element of multiplication.

        Returns:
            :class:`inekf.LieGroup`
        """
        pass

    def __invert__(self):
        """Invert group element. Drops augmented state and covariance.
        
        Returns:
            :class:`inekf.LieGroup`
        """
        pass


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

class SE3(LieGroup, metaclass=_meta_SE3):
    """3D rigid body transformation, also known as the 4x4 special Euclidean group, SE(3).

    See the C++ counterpart (:cpp:class:`InEKF::SE3`) for documentation on constructing an object.
    Further, we have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`. Templates include:

    Templates:

    - ``C`` Number of Euclideans columns to include. Can be -1 or "D" for dynamic. Defaults to 1.
    - ``A`` Number of augmented Euclidean states. Can be -1 or "D" for dynamic. Defaults to 0.
    """
    @staticmethod
    def wedge(xi):
        return _inekf.SE3_1_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SE3_1_0.exp(xi)

    @staticmethod
    def log_(g):
        return _inekf.SE3_1_0.log_(g)

    @staticmethod
    def Ad_(g):
        return _inekf.SE3_1_0.Ad_(g)

    def addCol(self, x, sigma):
        """Adds a column to the matrix state. Only usable if C = Eigen::Dynamic.
        
        Args:
            x (:obj:`np.ndarray`): Column to add in.
            sigma (:obj:`np.ndarray`): Covariance of element. Only used if state is uncertain.
        """
        pass

    def __getitem__(self, idx):
        """Gets the ith positional column of the group.
        
        Args:
            idx (:obj:`float`): Index of column to get, from 0 to C-1.

        Returns:
            :obj:`np.ndarray` 
        """
        pass

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

class SE2(LieGroup, metaclass=_meta_SE2):
    """2D rigid body transformation, also known as the 3x3 special Euclidean group, SE(2).
    
    See the C++ counterpart (:cpp:class:`InEKF::SE2`) for documentation on constructing an object.
    Further, we have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`. Templates include:

    Templates:

    - ``C`` Number of Euclideans columns to include. Can be -1 or "D" for dynamic. Defaults to 1.
    - ``A`` Number of augmented Euclidean states. Can be -1 or "D" for dynamic. Defaults to 0.
    """
    @staticmethod
    def wedge(xi):
        return _inekf.SE2_1_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SE2_1_0.exp(xi)

    @staticmethod
    def log_(g):
        return _inekf.SE2_1_0.log_(g)

    @staticmethod
    def Ad_(g):
        return _inekf.SE2_1_0.Ad_(g)

    def addCol(self, x, sigma):
        """Adds a column to the matrix state. Only usable if C = Eigen::Dynamic.
        
        Args:
            x (:obj:`np.ndarray`): Column to add in.
            sigma (:obj:`np.ndarray`): Covariance of element. Only used if state is uncertain.
        """
        pass

    def __getitem__(self, idx):
        """Gets the ith positional column of the group.
        
        Args:
            idx (:obj:`float`): Index of column to get, from 0 to C-1.

        Returns:
            :obj:`np.ndarray` 
        """
        pass

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

class SO3(LieGroup, metaclass=_meta_SO3):
    """3D rotational states, also known as the 3x3 special orthogonal group, SO(3).
    
    See the C++ counterpart (:cpp:class:`InEKF::SO3`) for documentation on constructing an object.
    Further, we have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`. Templates include:

    Templates:

    - ``A`` Number of augmented Euclidean states. Can be -1 or "D" for dynamic. Defaults to 0.
    """
    @staticmethod
    def wedge(xi):
        return _inekf.SO3_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SO3_0.exp(xi)

    @staticmethod
    def log_(g):
        return _inekf.SO3_0.log_(g)

    @staticmethod
    def Ad_(g):
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

class SO2(LieGroup, metaclass=_meta_SO2):
    """2D rotational states, also known as the 2x2 special orthogonal group, SO(2).
    
    See the C++ counterpart (:cpp:class:`InEKF::SO2`) for documentation on constructing an object.
    Further, we have overloaded the ``[]`` operator to function as a python template. Example of this 
    can be seen in :ref:`start`. Templates include:

    Templates:

    - ``A`` Number of augmented Euclidean states. Can be -1 or "D" for dynamic. Defaults to 0.
    """
    @staticmethod
    def wedge(xi):
        return _inekf.SO2_0.wedge(xi)

    @staticmethod
    def exp(xi):
        return _inekf.SO2_0.exp(xi)

    @staticmethod
    def log_(g):
        return _inekf.SO2_0.log_(g)

    @staticmethod
    def Ad_(g):
        return _inekf.SO2_0.Ad_(g)