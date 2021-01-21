#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Dense>
#include "iekf/state.h"
#include "iekf/base_lie.h"

namespace InEKF {

class MeasureModel {
    
    public:        
        MeasureModel() {};
        virtual void Observe(const Eigen::VectorXd& z, const State& state) = 0;
        Eigen::MatrixXd getSinv() { return Sinv_; }
        Eigen::MatrixXd getHBase() { return H_base_; }
        Eigen::VectorXd getV() { return V_; }
        State::ERROR getError() { return error_; }

        void setH(Eigen::MatrixXd H) { H_ = H; }

        LieGroup * lie_;

    protected:
        // These are all constant and should be set once in the constructor
        State::ERROR error_;
        Eigen::MatrixXd M_;
        Eigen::MatrixXd H_base_;

        // This is changed by InEKF based on if it's a RIGHT/LEFT filter
        Eigen::MatrixXd H_;
        // These should be changed each time Observe is called.
        Eigen::MatrixXd Sinv_;
        Eigen::VectorXd V_;
        

};

}

#endif // BASE_MEASURE