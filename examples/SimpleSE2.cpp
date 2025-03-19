#include <Eigen/Core>
#include <InEKF/Core>
#include <InEKF/SE2Models>
#include <iostream>
#include <cmath>

// Helper function to print state in (x,y,theta) form
void printState(const InEKF::SE2<1,0>& state, const std::string& label) {
    double theta = atan2(state.mat()(1,0), state.mat()(0,0));
    double x = state.mat()(0,2);
    double y = state.mat()(1,2);
    std::cout << label << " (x,y,theta):\n";
    std::cout << "x: " << x << "m, y: " << y << "m, theta: " << theta << " rad (" 
              << theta * 180/M_PI << " deg)\n\n";
}

int main() {
    // ============ Define all movements and measurements ============
    // First movement
    InEKF::SE2<1,0> U(0.5, 1, 1);

    // First GPS measurement
    Eigen::Vector3d z_gps{1, 0, 1};
    
    // Second movement
    InEKF::SE2<1,0> U2(0, 1, 1); 
    
    // Second GPS measurement
    Eigen::Vector3d z_gps2{1, 1, 1};
    
    // ============ Setup Models and Initial State ============
    // Set the covariance of theta (rad), x, y
    InEKF::OdometryProcess pModel(0.001, 0.05, 0.05);

    // Make GPS measurement model (left invariant as per docs)
    // Make b vector
    Eigen::Vector3d b{0, 0, 1};
    // Make covariance
    Eigen::Matrix2d M = Eigen::Matrix2d::Identity() * 0.01;
    // Make model
    InEKF::MeasureModel<InEKF::SE2<1,0>> gps(b, M, InEKF::ERROR::LEFT);
    
    // Make initial estimate
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.1;
    InEKF::SE2<1,0> x0(0, 0, 0, cov);
    std::cout << "Initial state matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << x0.mat() << "\n\n";

    std::cout << "Initial covariance [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << x0.cov() << "\n\n";
    printState(x0, "Initial state");
    
    // Make InEKF
    InEKF::InEKF iekf(&pModel, x0, InEKF::ERROR::LEFT);
    // InEKF::InEKF iekf(&pModel, x0, InEKF::ERROR::RIGHT);
    iekf.addMeasureModel("gps", &gps);
    
    // ============ Execute Prediction and Update Steps ============
    InEKF::SE2<1,0> state;
    
    // First predict step
    state = iekf.predict(U);
    std::cout << "State after predict matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << state.mat() << "\n\n";

    std::cout << "Covariance after predict [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << state.cov() << "\n\n";
    printState(state, "State after predict");
    
    // First GPS update
    state = iekf.update("gps", z_gps);
    std::cout << "State after GPS update matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << state.mat() << "\n\n";

    std::cout << "Covariance after GPS update [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << state.cov() << "\n\n";
    printState(state, "State after GPS update");
    
    // Second predict step
    state = iekf.predict(U2);
    std::cout << "State after second predict matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << state.mat() << "\n\n";

    std::cout << "Covariance after second predict [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << state.cov() << "\n\n";
    printState(state, "State after second predict");
    
    // Second GPS update
    state = iekf.update("gps", z_gps2);
    std::cout << "State after second GPS update matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << state.mat() << "\n\n";

    std::cout << "Covariance after second GPS update [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << state.cov() << "\n\n";
    printState(state, "State after second GPS update");
    
    // Get final state
    state = iekf.getState();
    std::cout << "Final state matrix [SE(2) format]:\n";
    std::cout << "[ R11=cos(θ)  R12=-sin(θ)  tx=x ]\n";
    std::cout << "[ R21=sin(θ)  R22=cos(θ)   ty=y ]\n";
    std::cout << "[ 0           0            1    ]\n";
    std::cout << state.mat() << "\n\n";

    std::cout << "Final covariance [ordering: θ,x,y]:\n";
    std::cout << "[ σ_θθ  σ_θx  σ_θy ]\n";
    std::cout << "[ σ_xθ  σ_xx  σ_xy ]\n";
    std::cout << "[ σ_yθ  σ_yx  σ_yy ]\n";
    std::cout << state.cov() << "\n\n";
    printState(state, "Final state");
    
    return 0;
}
