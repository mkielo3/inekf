#ifndef GENERIC_MEASURE
#define GENERIC_MEASURE

#include <Eigen/Dense>
#include "Core/LieGroup.h"
#include "Core/MeasureModel.h"
#include "iostream"

namespace InEKF {

template<class Group>
class GenericMeasureModel : public MeasureModel<Group> {
    
    protected:
        typedef typename MeasureModel<Group>::MatrixS MatrixS;
        typedef typename MeasureModel<Group>::MatrixH MatrixH;
        typedef typename MeasureModel<Group>::VectorV VectorV;
        typedef typename MeasureModel<Group>::VectorB VectorB;

        VectorB b_;

    public:      
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GenericMeasureModel(MatrixH H, MatrixS M, ERROR error) {
            this->H_ = H;
            this->M_ = M;
            this->error_ = error;
        };
        GenericMeasureModel(VectorB b, const MatrixS& M, ERROR error) : b_(b) {
            assert(b.head(Group::rotSize) == VectorV::Zero() && "Non-zero b in rotation portion not supported");
            
            this->M_ = M;
            this->error_ = error;

            this->H_ = MatrixH::Zero();

            int rDim = Group::rotSize*(Group::rotSize - 1) / 2;
            for(int i=0; i<Group::mtxSize-Group::rotSize; i++){
                this->H_.block(0, Group::rotSize*i+rDim, Group::rotSize, Group::rotSize) = b(i+Group::rotSize)*MatrixS::Identity();
            }
            if(error == ERROR::RIGHT){
                this->H_ *= -1;
            }
        }
        GenericMeasureModel(MatrixS M, ERROR error) {
            this->M_ = M;
            this->error_ = error;
        };

        VectorB fillZ(const Eigen::VectorXd& z, const Group& state){
            VectorB temp = b_;
            temp.head(Group::rotSize) = z;
            return temp;
        }
};

}

#endif // GENERIC_MEASURE