#ifndef BASE_LIE
#define BASE_LIE

#include <Eigen/Dense>

namespace InEKF {

class LieGroup {
    
    public:
        LieGroup() : dim(0), cols(0), augment(0) {};
        virtual Eigen::MatrixXd Mountain(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd ExpMountain(const Eigen::VectorXd& xi) = 0;

        virtual Eigen::MatrixXd Cross(const Eigen::VectorXd& xi) = 0;
        virtual Eigen::MatrixXd ExpCross(const Eigen::VectorXd& xi) = 0;
        
        virtual Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& x) = 0;

        int getDim() { return dim; }
        int getCols() { return cols; }
        int getAugmentSize() { return augment; }
        int getSigmaSize() { return dim + cols*dim + augment; }
        int getMuSize() { return dim + cols; }

        int getMuStates() { return dim + cols*dim; }
        int getTotalStates() { return dim + cols*dim + augment; }

    protected:
        int dim;
        int cols;
        int augment;
};

}

#endif // BASE_LIE