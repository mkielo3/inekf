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

/**
 * @brief The Invariant Extended Kalman Filter
 * 
 * @tparam pM Process Model. Pulls group and control info from it, can be left out if class template argument deduction (C++17) is used.
 */
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
        /**
         * @brief Construct a new InEKF object
         * 
         * @param pModel Pointer to the process model.
         * @param state Initial state, must be of same group that process model uses, and must be uncertain.
         * @param error Right or left invariant error.
         */
        InEKF(pM* pModel, Group state, ERROR error=ERROR::RIGHT) : pModel_(pModel), state_(state), error_(error) {
            assert(state.Uncertain() == true);
        };

        /**
         * @brief Prediction Step.
         * 
         * @param u Control, must be same as what process model uses.
         * @param dt Delta t. Used sometimes depending on process model. Defaults to 1.
         * @return State estimate
         */
        Group Predict(const U& u, double dt=1);
        
        /**
         * @brief Update Step.
         * 
         * @param name Name of measurement model.
         * @param z Measurement. May vary in size depending on how measurement model processes it.
         * @return State estimate.
         */
        Group Update(std::string name, const Eigen::VectorXd& z);

        /**
         * @brief Add measurement model to the filter.
         * 
         * @param name Name of measurement model.
         * @param m A measure model pointer, templated by the used group.
         */
        void addMeasureModel(std::string name, MeasureModel<Group>* m);

        /**
         * @brief Add multiple measurement models to the filter.
         * 
         * @param m Map from model names to model. Can be used passed in as {"name": model, "another": diff_model}
         */
        void addMeasureModels(std::map<std::string, MeasureModel<Group>*> m);

        /**
         * @brief Get the current state estimate
         * 
         * @return const Group& 
         */
        const Group& getState() const { return state_; }

        /**
         * @brief Set the current state estimate
         * 
         * @param state Current state estimate
         */
        void setState(const Group& state) { state_ = state; }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#include "InEKF.tpp"

#endif // INEKF