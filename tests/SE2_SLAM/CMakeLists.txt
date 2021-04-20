set(TESTS test_Odometry.cpp)

set(TEST_LIBS Eigen3::Eigen 
                InEKF::Core 
                InEKF::SE2_SLAM)

add_and_install_tests("${TESTS}" "${TEST_LIBS}")