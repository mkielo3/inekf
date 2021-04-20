#ifndef INEKF
#define INEKF

#include <Eigen/Core>
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
        typedef typename pM::myGroup Group;
        // useful for predict step
        typedef typename pM::myU U;
        typedef typename Group::MatrixCov MatrixCov;
        // useful for update step
        typedef typename Eigen::Matrix<double,Group::rotSize,Group::rotSize> MatrixS;
        typedef Eigen::Matrix<double,Group::rotSize,Group::N> MatrixH;
        typedef Eigen::Matrix<double,Group::rotSize,1> VectorV;
        typedef Eigen::Matrix<double,Group::M,1> VectorB;
        typedef Eigen::Matrix<double,Group::N,Group::rotSize> MatrixK;
        typedef typename Group::TangentVector TangentVector;
        
        Group state_;
        ERROR error_;
        std::map<std::string, MeasureModel<Group>*> mModels;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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