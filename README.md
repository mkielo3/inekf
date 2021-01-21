# Invariant Extended Kalman Filter
InEKF is a C++ library that implements the Invariant Extend Kalman Filter (InEKF) in a modular way that makes use for arbitrary systems easy to do.

## Building & Linking
To use InEKF, only `Eigen` is necessary. This can be installed via `apt-get` or from source. Also, a version of python with numpy and matplotlib installed for plotting reasons (I recommend systemwide, conda gave me fits).

InEKF is built via cmake, and thus can be built in the usual cmake fashion:
```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

This will install a custom target that can be linked via CMake as

```cmake
find_package(Eigen3 CONFIG REQUIRED)
find_package(InEKF CONFIG REQUIRED)
target_link_libraries(mytarget PUBLIC InEKF::Core InEKF::SE2_3_Bias)
```

## Structure
InEKF is split into a couple different libraries

### Core
This library includes the `State` and `InEKF` classes along with the base classes `MeasureModel`, `ProcessModel`, and `LieGroup` (we'll talk about those lower). 

`State` used to track the state mean and covariance. It is always passed as a reference, and thus your original object will change as the filter progresses.

`InEKF` is exactly what is sounds like.

### SE2_3_Bias
This is the implementation of the Lie Group `SE_2(3)` along with an augmented bias state. Along with it are various process/measurement models defined on this group including as of now `DVLSensor`, `DepthSensor`, and `InertialModel`. 

## Extending

InEKF is set up so your process/measure/lie group will be an easy extension and continue to function with `InEKF` and `State` if defined properly. The following is what must be defined/done to successfully do this. The following methods/variables for each base class must be implemented/set

### LieGroup
|      Method       | Use                                                           |
| :---------------: | :------------------------------------------------------------ |
|  `Mountain(xi)`   | Takes an element of R^n to the Lie Algebra                    |
| `ExpMountain(xi)` | Takes an element of R^n to the Lie Group (likely via exp map) |
|    `Cross(xi)`    | Creates a skew-symmetric matrix                               |
|  `ExpMatrix(xi)`  | Creates a skew-symmetric matrix and then applies the exp map  |
|       `dim`       | Dimension of Rotation matrix (3 for SE(3), 2 for SE(2), etc)  |
|       `col`       | Number of columns to track (1 for SE(3), 2 for SE_2(3), etc)  |
|     `augment`     | Number of augmented states to track (6 for `SE2_2_Bias`)      |

### MeasureModel
|       Method        | Use                                                                                                                     |
| :-----------------: | :---------------------------------------------------------------------------------------------------------------------- |
| `Observe(z, state)` | Should calculate and set Sinv_ and V_ upon being called                                                                 |
|      `error_`       | Type of invariant measurement, either `ERROR::LEFT` or `ERROR::Right`                                                   |
|        `M_`         | Noise parameter. A default should be set in the constructor, and possible a method made to set it                       |
|      `H_base_`      | Linearized innovation matrix `H`. Will be hit with adjoint depending on type of filter. Use `H_` in `Observe(z, state)` |
|       `lie_*`       | Pointer to LieGroup object measurement model is built on                                                                      |

### ProcessModel
|         Method          | Use                                                                                               |
| :---------------------: | :------------------------------------------------------------------------------------------------ |
|    `f(u, dt, state)`    | State process model. Should set result in state                                                   |
| `MakePhi(u, dt, state)` | Creates exp(A*dt) to use. Make sure to check what type of error State is and make A accordingly   |
|          `Q_`           | Noise parameter. A default should be set in the constructor, and possible a method made to set it |
|         `lie_*`         | Pointer to LieGroup object process model is built on                                              |

## Future Work
* Make LieGroup based on a template/flexible to make it set based on dim/col/augmented state (at least col/augmented state, may need 2 templates - 1 for SE_k(2) and one for SE_k(3))
* Improve calculation of matrix exponential to improve speed
* Various optimizations (remove redundant matrix multiplications)
* Make flexible measurement model with error and b as input (so not all measurement models have to be coded up)