#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Dense>

class MeasureModelBase {
    
    public:
        MeasureModelBase(Eigen::MatrixXd& M) : M_(M) {};
        virtual void Observe(Eigen::VectorXd& z, State& state);
        Eigen::MatrixXd N;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Sinv;

    private:
        Eigen::MatrixXd M_;
        

};

#endif // BASE_MEASURE