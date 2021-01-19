#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Dense>
#include "iekf/state.h"
#include "iekf/base_lie.h"

class MeasureModel {
    
    public:        
        MeasureModel() {};
        virtual void Observe(const Eigen::VectorXd& z, const State& state) = 0;
        Eigen::MatrixXd getSinv() { return Sinv_; }
        Eigen::MatrixXd getH() { return H_; }
        Eigen::VectorXd getV() { return V_; }
        State::ERROR getError() { return error_; }

        LieGroup * lie_;

    protected:
        // These are all constant and should be set once in the constructor
        State::ERROR error_;
        Eigen::MatrixXd M_;
        Eigen::MatrixXd H_;

        // These shoudl be changed each time Observe is called.
        Eigen::MatrixXd Sinv_;
        Eigen::VectorXd V_;
        

};

#endif // BASE_MEASURE