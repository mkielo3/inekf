#ifndef BASE_MEASURE
#define BASE_MEASURE

#include <Eigen/Core>
#include <Eigen/LU>
#include "Core/LieGroup.h"

namespace InEKF {

/**
 * @brief Base class measure model. Written to be inherited from, but in most cases this class will be sufficient.
 * 
 * @tparam Group State's group that is being tracked.
 */
template<class Group>
class MeasureModel {
    
    public:
        /**
        * @brief Size of matrix needed for the measurement model covariance.
        * 
        */
        typedef Eigen::Matrix<double,Group::rotSize,Group::rotSize> MatrixS;

        /**
         * @brief Size of matrix needed for linearized measurement model.
         * 
         */
        typedef Eigen::Matrix<double,Group::rotSize,Group::N> MatrixH;

        /**
         * @brief Size of vector for truncated innovation.
         * 
         */
        typedef Eigen::Matrix<double,Group::rotSize,1> VectorV;

        /**
         * @brief Size of vector for full measurement size.
         * 
         */
        typedef Eigen::Matrix<double,Group::M,1> VectorB;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        // These are all constant and should be set once
        /**
         * @brief Type of error of the filter (right/left)
         * 
         */
        ERROR error_;

        /**
         * @brief Measurement covariance
         * 
         */
        MatrixS M_ = MatrixS::Identity(Group::rotSize, Group::rotSize);

        // This one can be changed each iteration in InEKF.Update, 
        // or should be set once in constructor
        /**
         * @brief b vector used in measure model.
         * 
         */
        VectorB b_ = VectorB::Zero(Group::m, 1);

        /**
         * @brief Linearized H matrix. Will be automatically created from b in constructor unless overriden.
         * 
         */
        MatrixH H_ = MatrixH::Zero(Group::rotSize, Group::c);

        /**
         * @brief This is the converted H used in InEKF if it's a right filter with left measurement or vice versa. 
         Used in calcSInverse if overriden.
         * 
         */
        MatrixH H_error_;


    public:
        /**
        * @brief Construct a new Measure Model object
        * 
        */
        MeasureModel() {};

        /**
         * @brief Construct a new Measure Model object, automatically creating H. Should be used most of the time.
         * 
         * @param b b vector from measurement model. Will be used to create H.
         * @param M Measurement covariance.
         * @param error Type of invariant measurement (right or left).
         */
        MeasureModel(VectorB b, const MatrixS& M, ERROR error) {
            if(Group::N == Eigen::Dynamic){
                throw std::range_error("Can't use Base MeasureModel on group with dynamic columns");
            }

            // Plug in constants
            this->M_ = M;
            this->error_ = error;
            this->setHandb(b);
        }

        /**
         * @brief Process measurement before putting into InEKF. Can be used to change frames, convert r/b->x/y, or append 0s.
         By default is used to append zeros/ones onto it according to b vector set. Called first in update step.
         * 
         * @param z Measurement
         * @param state Current state estimate.
         * @return Processed measurement.
         */
        virtual VectorB processZ(const Eigen::VectorXd& z, const Group& state) {
            if(z.rows() == Group::M){
                return z;
            }
            else if(z.rows() == Group::rotSize){
                VectorB temp = b_;
                temp.head(Group::rotSize) = z;
                return temp;
            }
            else{
                throw std::range_error("Wrong sized z");
            }
        }

        /**
         * @brief Sets and returns H_error_ for settings where filter error type != measurement error type. 
         Done by multiplying H by adjoint of current state estimate. Called second in update step.
         * 
         * @param state Current state estimate.
         * @param iekfERROR Type of filter error.
         * @return H_error_ 
         */
        virtual MatrixH makeHError(const Group& state, ERROR iekfERROR){
            if( iekfERROR != error_ ){
                if(iekfERROR == ERROR::RIGHT){
                    H_error_ = H_*Group::Ad( state.inverse()() );
                }
                else{
                    H_error_ = H_*Group::Ad( state() );
                }
            }
            else{
                H_error_ = H_;
            }
            return H_error_;
        }

        /**
         * @brief Computes innovation based on measurement model. Called third in the update step.
         * 
         * @param z Measurement.
         * @param state Current state estimate.
         * @return Truncated innovation.
         */
        virtual VectorV calcV(const VectorB& z, const Group& state){
            // calculate V
            VectorV V;
            if(error_ == ERROR::RIGHT){
                V.noalias() = state().block(0,0,Group::rotSize,state().cols()) * z - b_.head(Group::rotSize);
            }
            else{
                V.noalias() = state.inverse()().block(0,0,Group::rotSize,state().cols()) * z - b_.head(Group::rotSize);
            }
            return V;
        }

        /**
         * @brief Calculate inverse of measurement noise S, using H_error_. Called fourth in the update step.
         * 
         * @param state Current state estimate.
         * @return Inverse of measurement noise. 
         */
        virtual MatrixS calcSInverse(const Group& state){
            MatrixS Sinv;
            MatrixS R = state.R()();
            if(error_ == ERROR::RIGHT){
                Sinv.noalias() = ( H_error_*state.cov()*H_error_.transpose() + R*M_*R.transpose() ).inverse();
            }
            else{
                Sinv.noalias() = ( H_error_*state.cov()*H_error_.transpose() + R.transpose()*M_*R ).inverse();
            }
            return Sinv;
        }

        /**
         * @brief Gets linearized matrix H.
         * 
         * @return MatrixH 
         */
        MatrixH getH() { return H_; }

        /**
         * @brief Get the measurement model error type.
         * 
         * @return ERROR 
         */
        ERROR getError() { return error_; }

        /**
         * @brief Sets measurement vector b and recreates H accordingly. Useful if vector b isn't constant.
         * 
         * @param b Measurement model b
         */
        void setHandb(VectorB b){
            this->b_ = b;

            this->H_ = MatrixH::Zero(Group::rotSize, Group::N);

            // Get rotation dimension
            int rDim = Group::rotSize*(Group::rotSize - 1) / 2;

            // Fill out rotation portion of H
            // SO(2) / SE(2)
            if(rDim == 1){
                this->H_(0,0) = -b_(1);
                this->H_(1,0) = b_(0);
            }
            // SO(3) / SE(3)
            else if(rDim == 3){
                this->H_.block(0, 0, Group::rotSize, Group::rotSize) = -1*SO3<>::wedge(b_.head(3));
            }

            // Fill out column portion of H
            for(int i=0; i<Group::M-Group::rotSize; i++){
                this->H_.block(0, Group::rotSize*i+rDim, Group::rotSize, Group::rotSize) = b_(i+Group::rotSize)*MatrixS::Identity();
            }
            if(error_ == ERROR::RIGHT){
                this->H_ *= -1;
            }
        }
};

}

#endif // BASE_MEASURE