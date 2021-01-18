#ifndef INEKF
#define INEKF

#include <Eigen/Dense>
#include <string>
#include <map>

#include "iekf/base_measure.h"
#include "iekf/base_process.h"

class InEKF {
    
    public:
        enum ERROR { LEFT, RIGHT };

        InEKF() {};
        InEKF(State state) : state_(state) {};
        State Update(Eigen::VectorXd u, double dt);
        State Correct(Eigen::VectorXd z, std::string type);

        void setProcessModel(ProcessModel& p);
        void addMeasureModel(MeasureModel& m, std::string name);
        
    private:
        State state_;
        std::map<std::string, MeasureModel*>  m_models_;
        ProcessModel * p_model_;
        InEKF::ERROR error_ = InEKF::ERROR::RIGHT;

};

#endif // INEKF