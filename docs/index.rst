Welcome to InEKF's documentation!
=================================
InEKF is a C++ library with python bindings that implements the Invariant Extend Kalman Filter (InEKF) in a modular to enable easy application to any system.

Features
~~~~~~~~
- Support for Right & Left filters.
- Base classes provide easy extension via inheritance.
- Coded using static Eigen types for efficient structure.
- Fully featured python interface for use in classroom, prototyping, etc.
- C++14 and above supported.
- Fullly templated Lie Groups SO2, SO3, SE2, SE3 to enable additional tracking of Euclidean states and multiple extra columns in SE2/SE3.
- Dynamic Lie Groups types to add columns to SE2/SE3 on the fly (for InEKF SLAM and others).
- Various examples to get started.

.. toctree::
   :maxdepth: 2
   :caption: InEKF Documentation

   usage/install
   usage/start
   usage/extend
   usage/changelog

.. toctree::
   :maxdepth: 2
   :caption: C++ API

   cpp/core
   cpp/groups
   cpp/inertial
   cpp/se2models

.. toctree::
   :maxdepth: 2
   :caption: Python API

   python/core
   python/groups
   python/inertial
   python/se2models


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
