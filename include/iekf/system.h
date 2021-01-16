#ifndef SYSTEM
#define SYSTEM

#include <Eigen/Dense>
#include <map>
#include "models/base_measure.h"
#include "models/base_process.h"

class System {

    public:
        System() {};

    private:
        std::map<std::string, MeasureModel>  m_models_;
        ProcessModel p_model;
};

#endif // SYSTEM