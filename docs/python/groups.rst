.. _python_group:

Lie Groups
=============

SO(2)
~~~~~~
.. autoclass:: inekf.SO2
   :show-inheritance:
   
SE(2)
~~~~~~
.. autoclass:: inekf.SE2
   :show-inheritance:
   :members: addCol
   :special-members: __getitem__

SO(3)
~~~~~~
.. autoclass:: inekf.SO3
   :show-inheritance:

SE(3)
~~~~~~
.. autoclass:: inekf.SE3
   :show-inheritance:
   :members: addCol
   :special-members: __getitem__

Lie Group Base
~~~~~~~~~~~~~~~~
.. autoclass:: inekf.LieGroup
   :members:
   :undoc-members:
   :special-members: __matmul__, __invert__