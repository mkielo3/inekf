#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Dense>
#include "iekf/state.h"

class MeasureModel {
    
    public:
        MeasureModel() {};
        virtual void Observe(Eigen::VectorXd& z, State& state) = 0;
        Eigen::MatrixXd getSinv() { return Sinv_; };
        Eigen::MatrixXd getH() { return H_; };
        Eigen::VectorXd getV() { return V_; };

    protected:
        Eigen::MatrixXd M_;
        Eigen::MatrixXd H_;
        Eigen::MatrixXd Sinv_;
        Eigen::VectorXd V_;
        

};

#endif // BASE_MEASURE