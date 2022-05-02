Installation
=============

Installation is straightforward in either language.

.. tabs::

    .. tab:: C++

        Installation of library in C++ requires manually building from source using CMake.
        The only dependency is Eigen, and if you want to build the python binding, pybind11 as well.
        If not found on the system, both of these will be pulled from their respective git repos, and
        built locally.

        A simple build will look like:

        .. code:: bash

            mkdir build
            cd build
            cmake ..
            make
            sudo make install

        Tests, examples, and the python binding can all be enabled with the following options
        when running cmake. Note all are disabled by default.

        .. code:: bash

            cmake .. -DTESTS=ON -DPYTHON=ON -DEXAMPLES=ON 

        .. note::

            By default, cmake is set to debug mode which can significantly slow things down, it can be 
            set to release mode by passing ``-DCMAKE_BUILD_TYPE=Release`` to cmake.

        After installation, the library can be linked against using the following lines in cmake

        .. code:: cmake

            find_package(Eigen3 CONFIG REQUIRED)
            find_package(InEKF CONFIG REQUIRED)
            target_link_libraries(mytarget PUBLIC InEKF::Core InEKF::Inertial InEKF::SE2Models)

        Alternatively, cmake can be configured to pull InEKF directly from git

        .. code:: cmake

            FetchContent_Declare(
              InEKF
              GIT_REPOSITORY git@bitbucket.org:frostlab/inekf.git
              GIT_TAG        v0.1.0
              )
            FetchContent_MakeAvailable(InEKF)

        The above options can be added into this line as well, for example ``option(PYTHON ON)`` right above
        the ``FetchContent_MakeAvailable(InEKF)`` line. When installed from source, the package can 
        be removed using ``sudo make remove``.

    .. tab:: Python

        Python installation is the easiest:) Just a single line to install from pip

        .. code:: bash

            pip install inekf