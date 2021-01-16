#include <Eigen/Dense>
#include <iostream>
#include "lie/SE2_3.h"

int main(){
    Eigen::VectorXd test(9);
    test << 0, 0, 0, 1, 1, 1, 2, 2, 2;

    SE2_3 lie;
    std::cout << lie.ExpMountain(test) << std::endl;

    return 0;
}