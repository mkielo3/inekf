#ifndef INEKF
#define INEKF

#include <Eigen/Dense>
#include <string>
#include <map>

#include "Core/State.h"
#include "Core/MeasureModel.h"
#include "Core/ProcessModel.h"

namespace InEKF {

class InEKF {
    
    public:
        InEKF(State& state) : state_(state) {};
        State Predict(const Eigen::VectorXd& u, double dt);
        State Update(const Eigen::VectorXd& z, std::string type);

        void setProcessModel(ProcessModel& p);
        void addMeasureModel(MeasureModel& m, std::string name);
        
    private:
        State& state_;
        std::map<std::string, MeasureModel*>  m_models_;
        ProcessModel * p_model_;

};

}

#endif // INEKF