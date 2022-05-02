# Invariant Extended Kalman Filter
[![Build Status](https://robots.et.byu.edu:4000/api/badges/frostlab/inekf/status.svg)](https://robots.et.byu.edu:4000/frostlab/inekf)
[![Documentation Status](https://readthedocs.org/projects/inekf/badge/?version=latest)](https://inekf.readthedocs.io/en/latest/?badge=latest)

InEKF is a C++ library with python bindings that implements the Invariant Extend Kalman Filter (InEKF) in a modular to enable easy application to any system.

## Features
- Support for Right & Left filters.
- Base classes provide easy extension via inheritance.
- Coded using static Eigen types for efficient structure.
- Fully featured python interface for use in classroom, prototyping, etc.
- C++14 and above supported.
- Fullly templated Lie Groups SO2, SO3, SE2, SE3 to enable additional tracking of Euclidean states and multiple extra columns in SE2/SE3.
- Dynamic Lie Groups types to add columns to SE2/SE3 on the fly (for InEKF SLAM and others).
- Various examples to get started.

## Documentation
To get started, please [read the docs](https://inekf.readthedocs.io/). We have attempted to thoroughly document everything, and it should be
fairly straightforward to get up and running. They have information on all of the following (and more!)

- [Installation](https://inekf.readthedocs.io/en/latest/usage/install.html)
- [Getting Started](https://inekf.readthedocs.io/en/latest/usage/start.html)
- [Making Custom Models](https://inekf.readthedocs.io/en/latest/usage/extend.html)
- Full API Documentation

## Structure
InEKF is split into a couple different libraries

### Core
This library includes all the Lie Groups and `InEKF` classes along with the base classes `MeasureModel` and `ProcessModel`.

### Inertial
This is the implementation of the Lie Group `SE_2(3)` along with an augmented bias state. Along with it are various process/measurement models defined on this group including as of now `DVLSensor`, `DepthSensor`, and `InertialProcess`.

### SE2Models
Exactly what it sounds like, is used for SLAM in SE2. Includes dynamic models that adjust as more landmarks are seen, including a `GPSSensor` and `LandmarkSensor`.

## Contributing
This is an open-source project and contributions are more then welcome. If any there are any features or bugs you'd like to report, please add then to the [issue tracker](https://bitbucket.org/frostlab/inekf/issues?status=new&status=open). If you would like to contribute code, please feel free to fork, modify, and create a pull request.