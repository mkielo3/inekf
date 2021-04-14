#ifndef CLASS_SE2
#define CLASS_SE2

#include <Eigen/Dense>
#include "LieGroup.h"
#include "SO2.h"

namespace InEKF {

template <int cols=1, int aug=0>
class SE2 : public LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>{
    private:
        typedef typename LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>::TangentVector TangentVector;
        typedef typename LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>::MatrixCov MatrixCov;
        typedef typename LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>::MatrixState MatrixState;
        typedef Eigen::Matrix<double, aug, 1> VectorAug;

        MatrixState State_;
        MatrixCov Cov_;
        VectorAug Aug_;
        bool isUncertain;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SE2(const MatrixState& State=MatrixState::Zero(), 
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero())
                : State_(State), Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()) {}

        SE2(const SE2& State) : SE2(State(), State.Cov(), State.Aug()) {}
        
        SE2(const SO2<>& R, const Eigen::Matrix<double,cols*2,1>& t=Eigen::Matrix<double,cols*2,1>::Zero(),
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero()) 
                : Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()) {
            State_ = MatrixState::Identity();
            State_.block(0,0,2,2) = R();
            for(int i=0;i<cols;i++){
                State_.block(0,2+i,2,1) = t.segment(2*i,2);
            }
        }

        SE2(const TangentVector& xi,
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero()) 
                : Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero()) {
            State_ = MatrixState::Identity();
            State_.block(0,0,2,2) = SO2<>(xi(0))();
            for(int i=0;i<cols;i++){
                State_.block(0,2+i,2,1) = xi.segment(2*i+1,2);
            }
        }

        SE2(double theta, double x, double y,
            const MatrixCov& Cov=MatrixCov::Zero(),
            const VectorAug& Aug=VectorAug::Zero());

        ~SE2() {}

        // Getters
        bool Uncertain() const { return isUncertain; }
        MatrixCov Cov() const { return Cov_; }
        VectorAug Aug() const { return Aug_; }
        MatrixState operator()() const { return State_; }
        SO2<> R() const { 
            Eigen::Matrix2d R = State_.block(0,0,2,2);
            return SO2<>(R); 
        }
        // TODO: Add out of bounds error
        Eigen::Vector2d operator[](int idx) const { return State_.block(0,2+idx,2,1); }
        // TODO: Do we actually want a setter or not? Maybe useful for InEKF Class?
        // Eigen::Ref<Eigen::Vector2d> operator[](int idx){ return State.block<2,1>(0,2+idx); }

        // TODO: Implement adding columns. May need to be template specialized
        // void addCol(const Eigen::Matrix3d& x){}

        // Self operations
        SE2 inverse() const{
            MatrixState S = MatrixState::Identity();
            Eigen::Matrix2d RT = this->R().inverse()();
            S.block(0,0,2,2) = RT;
            for(int i=0;i<cols;i++){
                S.block(0,2+i,2,1) = -1 * RT * (*this)[i];
            }
            return SE2(S);
        }
        using LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>::Ad;
        using LieGroup<SE2<cols,aug>,calcStateDim(2,cols,aug),calcStateMtxSize(2,cols)>::log;

        // Group action
        SE2 operator*(const SE2& rhs) const{
            // Skirt around composing covariances
            MatrixCov Cov = MatrixCov::Zero();
            if(this->Uncertain() && rhs.Uncertain()){
                throw "Can't compose uncertain LieGroups";
            }
            if(this->Uncertain()) Cov = this->Cov();
            if(rhs.Uncertain()) Cov = rhs.Cov();

            // Compose state + augment
            MatrixState State = (*this)() * rhs();
            VectorAug Aug = this->Aug() + rhs.Aug();

            return SE2<aug>(State, Cov, Aug);
        }

        // Static Operators
        static MatrixState Wedge(const TangentVector& xi){
            MatrixState X = MatrixState::Zero();
            X.block(0,0,2,2) = SO2<>::Wedge(xi.segment(0,1));
            for(int i=0;i<cols;i++){
                X.block(0,2+i,2,1) = xi.segment(2*i+1,2);
            }
            return X;
        }
        static SE2 Exp(const TangentVector& xi){
            double theta = xi(0);

            // Find V
            Eigen::Matrix2d V;
            if(theta < .0001){
                Eigen::Matrix2d thetaWedge = SO2<>::Wedge(xi.segment(0,1));
                V = Eigen::Matrix2d::Identity() + thetaWedge/2 + thetaWedge*thetaWedge/6 + thetaWedge*thetaWedge*thetaWedge/24;
            }
            else{
                V(0,0) = sin(theta);
                V(1,1) = sin(theta);
                V(1,0) = 1 - cos(theta);
                V(0,1) = cos(theta) - 1;
                V /= theta;
            }

            MatrixState X = MatrixState::Identity();
            X.block(0,0,2,2) = SO2<>::Exp(xi.segment(0,1))();
            for(int i=0;i<cols;i++){
                X.block(0,2+i,2,1) = V*xi.segment(2*i+1,2);
            }
            return SE2(X, MatrixCov::Zero(), xi.tail(aug));
        }
        static TangentVector Log(const SE2& g){
            double theta = SO2<>::Log(g.R())(0);
            double a, b, scale;
            if(theta < .0001){
                a = 1;
                b = 0;
                scale = 1;
            }
            else{
                a = sin(theta)/theta;
                b = (1-cos(theta))/theta;
                scale = 1 / (a*a + b*b);
            }

            Eigen::Matrix2d V_inv;
            V_inv << a, b, -b, a;
            V_inv *= scale;

            TangentVector xi;
            xi(0) = theta;
            for(int i=0;i<cols;i++){
                xi.segment(1+2*i,2) = V_inv*g[i];
            }
            xi.tail(aug) = g.Aug();
            return xi;
        }
        static MatrixCov Ad(const SE2& g){
            MatrixCov Ad_X = MatrixCov::Identity();
            for(int i=0;i<cols;i++){
                Ad_X.block(1+2*i,1+2*i,2,2) = g.R()();
                Ad_X(1+2*i,0) = g[i][1];
                Ad_X(2+2*i,0) = -g[i][0];
            }
            return Ad_X;
        }

};

template <>
SE2<1,0>::SE2(double theta, double x, double y,
    const MatrixCov& Cov,
    const VectorAug& Aug) 
        : Cov_(Cov), Aug_(Aug), isUncertain(Cov != MatrixCov::Zero())  {
    State_ << cos(theta), -sin(theta), x,
                sin(theta),  cos(theta), y,
                0, 0, 1;
}

template <int cols, int aug>
std::ostream& operator<<(std::ostream& os, const SE2<cols, aug>& rhs)  
{
  os << rhs();
  return os;
}

}

#endif // CLASS_SE2