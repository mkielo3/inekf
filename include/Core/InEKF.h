#ifndef INEKF
#define INEKF

#include <Eigen/Dense>
#include <string>
#include <map>

#include "Core/LieGroup.h"
#include "Core/MeasureModel.h"
#include "Core/ProcessModel.h"

namespace InEKF {

// TODO: Make process model not a template?? Make Group a template instead?
// the benefit of letting U be anything is probably worth it staying a template.
template <class pM>
class InEKF {

    private:
        typedef typename pM::myU U;
        typedef typename pM::myGroup Group;
        typedef typename pM::MatrixCov MatrixCov;

        typedef Eigen::Matrix<double, Group::rotSize, Group::dimension> MatrixH;
        
        Group& state_;
        ERROR error_;
        std::map<std::string, MeasureModel<Group>*> mModels;

    public:
        pM pModel;

        InEKF(Group state=Group(true), ERROR error=ERROR::RIGHT) : state_(state), error_(error) {
            assert(state.Uncertain() == true);
        };

        Group Predict(const U& u, double dt=1);
        
        Group Update(const Eigen::VectorXd& z, std::string type);
        Group Update(const Eigen::VectorXd& z, std::string type, MatrixH H);

        void addMeasureModel(std::string name, MeasureModel<Group>* m);
        void addMeasureModels(std::map<std::string, MeasureModel<Group>*> m);
};

}

#include "InEKF.tpp"

#endif // INEKF