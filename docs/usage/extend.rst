.. _extend:


Custom Models
==============

InEKF is set up so your process/measure models will be an easy extension and continue to function with ``InEKF`` and ``LieGroups`` 
if defined properly. Note this can be done in python or C++. The following is what must be defined/done to successfully do this. 
The following methods/variables for each base class must be implemented/set

MeasureModel
~~~~~~~~~~~~~
All methods are already implemented in the ``MeasureModel`` class, so the base class can be used in most scenarios.
This means when you do want to make a custom measurement class, which methods to override can be decided on a case by case basis.
The ``MeasureModel`` constructor takes in the vector ``b``, covariance ``M``, and type of error and from these makes ``H`` accordingly. 
It is also templated by the type of group that it is defined on.

If you decide to override, make sure you call the base class constructor and set the error, or set the first 4 values of the following,
otherwise they default to all zeros.

+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
|                     Method                     |                                                                                                                            Use                                                                                                                            |
+================================================+===========================================================================================================================================================================================================================================================+
| :cpp:member:`~InEKF::MeasureModel::error_`     | Type of invariant measurement, of type :cpp:enum:`InEKF::ERROR`.                                                                                                                                                                                          |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:member:`~InEKF::MeasureModel::M_`         | Noise parameter. A default should be set in the constructor, and possible a method made to set it                                                                                                                                                         |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:member:`~InEKF::MeasureModel::b_`         | ``b`` vector in invariant measurement model. Can be used to set :cpp:member:`~InEKF::MeasureModel::H_` through :cpp:func:`~InEKF::MeasureModel::setHandb` and if the first few elements are nonzero, is needed in :cpp:func:`~InEKF::MeasureModel::calcV` |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:member:`~InEKF::MeasureModel::H_`         | Linearized innovation matrix ``H``. Can be set manually or from Will be hit with adjoint depending on type of filter.                                                                                                                                     |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:func:`~InEKF::MeasureModel::processZ`     | Any preprocessing that needs to be done on z should be done here. This could include adding 0s and 1s on the end, change of frames, etc. Returns z.                                                                                                       |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:func:`~InEKF::MeasureModel::makeHError`   | Shifts :cpp:member:`~InEKF::MeasureModel::H_` by the adjoint, and saves it in :cpp:member:`~InEKF::MeasureModel::H_error_` and returns it. Likely will not need to be overriden.                                                                          |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:func:`~InEKF::MeasureModel::calcV`        | Accepts an exact size of z, and calculates/returns the innovation. Likely will not need to be overriden.                                                                                                                                                  |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| :cpp:func:`~InEKF::MeasureModel::calcSInverse` | Calculates and returns S^{-1}, the inverse of the measurement covariance. Also likely won't need to be overriden. Use :cpp:member:`~InEKF::MeasureModel::H_error_` here.                                                                                  |
+------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

Building a custom ``SE(2)`` measure model in C++ and python will look something like the following.

.. tabs::

    .. code-tab:: c++

        class MySensor : public InEKF::MeasureModel<InEKF::SE2<1,0>> {}

    .. code-tab:: python

        class MySensor(inekf.MeasureModel[inekf.SE2[1,0]]):
            pass

And then override functions as needed. For examples see the `Inertial Models in C++ <https://bitbucket.org/frostlab/inekf/src/master/include/Inertial/>`_ and the
`Underwater Inertial from scratch script in python <https://bitbucket.org/frostlab/inekf/src/master/python/examples/UnderwaterInertial_fromScratch.py>`_.

.. note::

    In python ``error_``, ``M_``, and ``H_`` are named ``error``, ``M``, and ``H``, respectively.
    Further note, due to how the python bindings function, you *can not* modify ``M`` and ``H`` in place,
    they must be written as a whole.

As a reference, here's what these functions will be used for in update step of the InEKF.

.. code:: cpp

    // Do any preprocessing on z (fill it up, frame changes, etc)
    VectorB z_ = m_model->processZ(z, state_);;

    // Change H via adjoint if necessary
    MatrixH H = m_model->makeHError(state_, error_);

    // Use measurement model to make Sinv and V
    VectorV V = m_model->calcV(z_, state_);
    MatrixS Sinv = m_model->calcSInverse(state_);

    // Caculate K + dX
    MatrixK K = state_.cov() * (H.transpose() * Sinv);    
    TangentVector K_V = K * V;

ProcessModel
~~~~~~~~~~~~
In contrast, the process model implements a few things that MUST be overriden. It is templated by both the group it is defined on, as well as 
the control input that is taken in.

+-------------------------------------------+---------------------------------------------------------------------------------------------------+
|                  Method                   |                                                Use                                                |
+===========================================+===================================================================================================+
| :cpp:func:`~InEKF::ProcessModel::f`       | State process model. Returns the state.                                                           |
+-------------------------------------------+---------------------------------------------------------------------------------------------------+
| :cpp:func:`~InEKF::ProcessModel::makePhi` | Creates exp(A*dt) to use. Make sure to check what type of error State is and make A accordingly   |
+-------------------------------------------+---------------------------------------------------------------------------------------------------+
| :cpp:member:`~InEKF::ProcessModel::Q_`    | Noise parameter. A default should be set in the constructor, and possible a method made to set it |
+-------------------------------------------+---------------------------------------------------------------------------------------------------+

Building a custom ``SE(2)`` process model with a 3-vector as controls in C++ and python will look something like the following.

.. tabs::

    .. code-tab:: c++

        class MyProcess : public ProcessModel<SE3<1,0>, Eigen::Vector3d> {

            public:
                MyProcess(MatrixCov Q) {Q_ = Q};
                ~MyProcess(){}
                SE3<1,0> f(Eigen::Vector3d u, double dt, SE3<1,0> state) override;
                MatrixCov makePhi(const Eigen::Vector3d& u, double dt, const SE3<1,0>& state, ERROR error) override;

        };

    .. code-tab:: python

        class MyProcess(inekf.ProcessModel[inekf.SE2[1,0], "Vec3"]):
            def __init__(self, Q):
                self.Q = Q

            def f(self, u, dt, state):
                # Your implementation here
                pass

            def makePhi(u, dt, state, error):
                # Your implementation here
                pass

For examples see the `Inertial Process model in C++ <https://bitbucket.org/frostlab/inekf/src/master/include/Inertial/InertialProcess.h>`_ and the
`Underwater Inertial from scratch script in python <https://bitbucket.org/frostlab/inekf/src/master/python/examples/UnderwaterInertial_fromScratch.py>`_.

.. note::

    Just like in the measure model case, here ``Q_`` is actually named ``Q`` on the python side..
    Again, due to how the python bindings function, you *can not* modify ``Q`` in place,
    it must be written as a whole.