.. _start:

Getting Started
================

The InEKF library has been designed to be straightforward and simple to use, while still being versatile.

First, we must import the library, and any dependencies.

.. tabs::

    .. code-tab:: c++

        #include <Eigen/Core>
        #include <InEKF/Core>
        #include <InEKF/SE2Models>

    .. code-tab:: python

        import numpy as np
        import inekf

.. note::
    TODO: Insert citation to InEKF tutorial here for more info about things!

Using Lie Groups
~~~~~~~~~~~~~~~~~
The Lie groups in the library are heavily templated to allow for simple usage of any Lie group. The special orthogonal groups
`SO` are templated to allow tracking additional Euclidean states alongside the group (defaults to 0), and the special
Euclidean groups `SE` are templated to allow the additional Euclidean states along with any number of positional columns (defaults to 1).

We've also repurposed the bracket ``[]`` in python to allow for a near identical usage across APIs. If using C++17 or python, 
these can templates can be omitted if using the defaults. We assume throughout C++17 is used, and ommit the empty ``<>``.

.. tabs::

    .. code-tab:: c++

        // Two columns, one Euclidean state
        InEKF::SE2<2,1> x1();

        // If using C++17
        // One column, no Euclidean states
        // 0 rotation, 1 for x and y, covariance of identities
        InEKF::SE2 x2(0, 1, 1, Eigen::Matrix3d::Identity());

        // If using older standard of C++
        InEKF::SE2<> x3(0, 1, 1, Eigen::Matrix3d::Identity());

    .. code-tab:: python

        # Two columns, one Euclidean state
        x1 = inekf.SE2[2, 1]()

        # One column, no Euclidean states
        # 0 rotation, 1 for x and y, covariance of identities
        x2 = inekf.SE2(0, 1, 1, np.eye(3))

They each have a number of constructors, see C++ :ref:`cpp_group` and Python :ref:`python_group` for more details. 

.. note::

    If no covariance is passed to the constructor, the state is assumed to be "certain" and no covariance is set.
    If you wish to have covariance tracked (necessary for use in InEKF), make sure you set this.


Further, each group is equipped with a number of common operations, such as

 - Inverse
 - Group action (multiplication)
 - Wedge ^ operator
 - Exp/Log
 - Adjoint

Along with these is overloading of ``()`` to return the state matrix, and ``[]`` to
retrieve a specific column.

.. tabs::

    .. code-tab:: c++

        InEKF::SE2 x(0,1,2);
        InEKF::SE2 y(3,4,5);
        Eigen::Vector3d xi{1,2,3};

        // Group methods
        x.inverse();
        x*y;
        x.log();
        x.Ad();

        // Static methods
        InEKF::SE2::wedge(xi);
        InEKF::SE2::exp(xi);
        InEKF::SE2::log(x);
        InEKF::SE2::Ad(x);

        // Getters
        x();
        x.mat();
        // SO2 object
        x.R();
        // Vector 1,2
        x[0];  
        // Covariance      
        x.cov();
        // Get additional Euclidean states
        x.aug();

    .. code-tab:: python

        x = inekf.SE2(0,1,2)
        y = inekf.SE2(3,4,5)
        xi = np.array([1,2,3])

        # Group methods
        x.inverse
        ~x
        x*y
        x.log
        x.Ad

        # Static methods
        inekf.SE2.wedge(xi)
        inekf.SE2.exp(xi)
        inekf.SE2.log(x)
        inekf.SE2.Ad(x)

        # Getters
        x.mat
        # SO2 object
        x.R
        # Vector 1,2
        x[0]
        # Covariance
        x.cov
        # Get additional Euclidean states
        x.aug


Making Models
~~~~~~~~~~~~~~
Next, process and measurement models must be made. You'll likely need a custom process model done via inheritance, for this see :ref:`extend`. You can also customize
measurement models (see same link), but the built in is robust enough for most purposes.

For example, here's creation of a simple odometry model in `SE(2)`,

.. tabs::

    .. code-tab:: c++

        // Set the covariance of theta (rad), x, y
        InEKF::OdometryProcess pModel(0.001, 0.05, 0.05);

    .. code-tab:: python

        # Set the covariance of theta (rad), x, y
        pModel = inekf.OdometryProcess(0.001, 0.05, 0.05)


An invariant measurement model is either a left :math:`Xb + w` or right :math:`X^{-1}b + w`. The invariant model is then defined
by this `b` vector, the covariance of `w`, and whether it's a right or a left measurement. The linearized innovation matrix `H` is then
automatically created. For example, we'll set up a GPS sensor in `SE(2)`, which is left invariant, 

.. tabs::

    .. code-tab:: c++

        // Make b vector
        Eigen::Vector3d b{0, 0, 1};

        // Make covariance
        Eigen::Matrix2d M = Eigen::Matrix2d::Identity()*.01;

        // Make model
        InEKF::MeasureModel<InEKF::SE2> gps(b, M, InEKF::ERROR::LEFT);

    .. code-tab:: python

        # Make b vector
        b = np.array([0, 0, 1])

        # Make covariance
        M = np.eye(2)*.01;

        # Make model
        gps = inekf.MeasureModel[inekf.SE2](b, M, inekf.ERROR.LEFT)

Or similarly, a compass measuring exactly true north, which is right invariant,

.. tabs::

    .. code-tab:: c++

        // Make b vector
        Eigen::Vector3d b{1, 0, 0};

        // Make covariance
        Eigen::Matrix2d M = Eigen::Matrix2d::Identity()*.01;

        // Make model
        InEKF::MeasureModel<InEKF::SE2> compass(b, M, InEKF::RIGHT);

    .. code-tab:: python

        # Make b vector
        b = np.array([1, 0, 0])

        # Make covariance
        M = np.eye(2)*.01

        # Make model
        compass = inekf.MeasureModel[inekf.SE2](b, M, inekf.ERROR.RIGHT)


Making & Using the InEKF
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Finally, we make the InEKF. The InEKF takes 3 arguments in its constructor: the process model, an initial estimate,
and whether to run a right or left InEKF.

.. tabs::

    .. code-tab:: c++

        // Make initial estimate
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity()*.1;
        InEKF::SE2 x0(0, 0, 0, cov);

        // Make Right InEKF
        InEKF::InEKF iekf(pModel, x0, InEKF::RIGHT);
        iekf.addMeasureModel("gps", &gps);
        iekf.addMeasureModel("compass", &compass);

    .. code-tab:: python

        # Make initial estimate
        cov = np.eye(3)*.1
        x0 = inekf.SE2(0, 0, 0, cov)

        # Make Right InEKF
        iekf = inekf.InEKF(pModel, x0, inekf.ERROR.RIGHT)
        iekf.addMeasureModel("gps", gps)
        iekf.addMeasureModel("compass", compass)

Using the predict and update steps is just as easy (we make fake data here to use). While technically an invariant measurement will have extra ones or zeros on the end,
the ``MeasureModel`` class will take care of appending these when needed. This steps are generally done in a loop and are executed when data is received. After each step is ran
it will return the corresponding state estimate which can also be accessed using ``getState``  in C++ or the ``state`` property in python.

.. tabs::

    .. code-tab:: c++

        InEKF::SE2 state;

        // Prediction step with some control U
        InEKF::SE2 U(.1, .1, .1);
        // Predict also takes an optional dt, which may or may not
        // be used, depending on the process model (not needed in this case)
        state = iekf.predict(U);

        // Update gps with location measurement = 1,1
        // We include the needed one here as well
        Eigen::Vector3d z_gps{1, 1, 1};
        // Updates with name we put in before
        state = iekf.update("gps", z_gps);

        // Update compass with measurement = 1, 0
        // Model appends the extra 0 is this case
        Eigen::Vector2d z_compass{1, 0};
        state = iekf.update("compass", z_compass);

        // Get most recent state
        state = iekf.getState();

    .. code-tab:: python

        # Prediction step with some control U
        U = inekf.SE2(.1, .1, .1)
        # Predict also takes an optional dt, which may or may not
        # be used, depending on the process model (not needed in this case)
        state = iekf.predict(U)

        # Update gps with location measurement = 1,1
        # We include the needed one here as well
        z_gps = np.array([1, 1, 1])
        # Updates with name we put in before
        state = iekf.update("gps", z_gps)

        # Update compass with measurement = 1, 0
        # Model appends the extra 0 is this case
        z_compass = np.array([1, 0])
        state = iekf.update("compass", z_compass)

        # Get most recent state
        state = iekf.state

More examples can be found in the bitbucket repository, both in `C++ <https://bitbucket.org/frostlab/inekf/src/master/examples/>`_ and `python <https://bitbucket.org/frostlab/inekf/src/master/python/examples/>`_.