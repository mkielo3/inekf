#ifndef BASE_PROCESS
#define BASE_PROCESS

#include <Eigen/Dense>
#include "lie/state.h"

class ProcessModelBase {

    public:
        ProcessModelBase();
        virtual State f(Eigen::VectorXd u, State state);
        virutal Eigen::MatrixXd MakeA(Eigen::VectorXd u, State state);
        
    private:
        

};

#endif // BASE_PROCESS