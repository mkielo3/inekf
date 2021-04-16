#ifndef GENERIC_MEASURE
#define GENERIC_MEASURE

#include <Eigen/Dense>
#include "Core/LieGroup.h"
#include "Core/MeasureModel.h"

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
        GenericMeasureModel(MatrixH H, MatrixS M, ERROR error) {
            this->H_ = H;
            this->M_ = M;
            this->error_ = error;
        };
        GenericMeasureModel(VectorB b, MatrixS M, ERROR error) : b_(b) {
            this->M_ = M;
            this->error_ = error;

            this->H_ = MatrixH::Zero();
            // TODO: Make this handle 1's in rotation portion better?
            // TODO: Handle H in augmented state?
            for(int i=Group::rotSize;i<Group::mtxSize;i++){
                this->H_.block(0, Group::rotSize*i, Group::rotSize, Group::rotSize) = b(i)*MatrixS::Identity();
            }
            if(error == ERROR::RIGHT){
                this->H_ *= -1;
            }
        }
        GenericMeasureModel(MatrixS M, ERROR error) {
            this->M_ = M;
            this->error_ = error;
        };

        VectorV calcV(const VectorV& z, const Group& state){
            // TODO: Make this handle 1's in rotation portion better?
            // TODO: Handle H in augmented state?
            VectorB temp = b_;
            temp.head(Group::rotSize) = z;
            return calcV(temp, state);
        }
};

}

#endif // GENERIC_MEASURE