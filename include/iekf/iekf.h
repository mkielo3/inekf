#ifndef INEKF
#define INEKF

#include <Eigen/Dense>
#include <string>
#include <map>

class State {

    public:
        State();


    private:
        Eigen::MatrixXd Mu_;
        Eigen::MatrixXd Sigma_;
        Eigen::MatrixXd Bias_;
};

class InEKF {
    
    public:
        InEKF();
        void Update(Eigen::VectorXd u);
        void Correct(Eigen::VectorXd z, std::string type);
        
    private:
        State state_;

};

#endif // INEKF