#ifndef LANDMARKSENSOR
#define LANDMARKSENSOR

#include <Eigen/Core>
#include <InEKF/Core>

namespace InEKF {

class LandmarkSensor : public MeasureModel<SE2<Eigen::Dynamic>> {
    private:
        Eigen::VectorXd b_;

    public:
        typedef typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixS MatrixS;
        typedef typename MeasureModel<SE2<Eigen::Dynamic>>::MatrixH MatrixH;
        typedef typename MeasureModel<SE2<Eigen::Dynamic>>::VectorV VectorV;
        typedef typename MeasureModel<SE2<Eigen::Dynamic>>::VectorB VectorB;

        LandmarkSensor(double std=1);
        ~LandmarkSensor(){ }
        // Used to prep H
        void sawLandmark(int idx, const SE2<Eigen::Dynamic>& state);
        VectorB processZ(const Eigen::VectorXd& z, const SE2<Eigen::Dynamic>& state) override;
};

}

#endif // LANDMARKSENSOR