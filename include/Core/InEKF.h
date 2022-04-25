#ifndef INEKF
#define INEKF

#include <Eigen/Core>
#include <string>
#include <map>
#include <memory>

#include "Core/LieGroup.h"
#include "Core/MeasureModel.h"
#include "Core/ProcessModel.h"

namespace InEKF {

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
        
        ERROR error_;
        pM* pModel_;
        Group state_;
        std::map<std::string, MeasureModel<Group>*> mModels;

    public:

        InEKF(pM* pModel, Group state, ERROR error=ERROR::RIGHT) : pModel_(pModel), state_(state), error_(error) {
            assert(state.Uncertain() == true);
        };

        Group Predict(const U& u, double dt=1);
        
        Group Update(std::string type, const Eigen::VectorXd& z);
        Group Update(std::string type, const Eigen::VectorXd& z, MatrixH H);

        void addMeasureModel(std::string name, MeasureModel<Group>* m);
        void addMeasureModels(std::map<std::string, MeasureModel<Group>*> m);

        const Group& getState() const { return state_; }
        void setState(const Group& state) { state_ = state; }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#include "InEKF.tpp"

#endif // INEKF