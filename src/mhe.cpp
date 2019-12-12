#include "turtlebot_MHE/mhe.h"
#include "turtlebot_MHE/types.h"
#include <ceres/ceres.h>

typedef ceres::AutoDiffCostFunction<PoseResidual,3,3> PoseCostFunction;
typedef ceres::AutoDiffCostFunction<MeasurementResidual, 2, 2> MeasurementCostFunction;

template<typename T>
T wrap(T angle)
{
    static double pi{3.14159};
    angle -= 2*pi * floor((angle+pi) * 0.5/pi);
    return angle;
}

namespace mhe
{

MHE::MHE()
{
    // set default parameters
    Omega_ = Eigen::Vector3d{1, 1, 0.5}.asDiagonal();
    R_inv_ = Eigen::Vector2d{1/0.35, 1/0.07}.asDiagonal();
}

MHE::~MHE()
{

}

void MHE::setParams(const Eigen::Vector3d &omega, double sig_r, double sig_phi)
{
    Omega_ = omega.asDiagonal();
    R_inv_ = Eigen::Vector2d{1/(sig_r*sig_r),1/(sig_phi*sig_phi)}.asDiagonal();
}

Pose MHE::propagateState(const Pose &state, const Input &u, double dt)
{
    double st{sin(state(THETA))};
    double ct{cos(state(THETA))};
    Pose out;
    out(X) = u(V) * ct * dt;
    out(Y) = u(V) * st * dt;
    out(THETA) = wrap(u(W) * dt);
    return out;
}

void MHE::update(const Pose &mu, const Meas &z, const Zidx& idx, const Input &u, double dt)
{
//    Pose mu_bar{propagateState(mu, u, dt)};

    pose_hist_.push_back(mu);
    z_hist_.push_back(z);

    optimize();
}

void MHE::optimize()
{
    ceres::Problem problem;

    //set up position residuals
    int i = std::max(0, int(pose_hist_.size() - TIME_HORIZON));
    for(i; i < pose_hist_.size(); ++i)
    {
        PoseCostFunction *cost_function{new PoseCostFunction(new PoseResidual(pose_hist_[i], Omega_))};
        problem.AddResidualBlock(cost_function, NULL, pose_hist_[i].data());
    }

    //set up measurement residuals
    i = std::max(0, int(z_hist_.size() - TIME_HORIZON));
    for(i; i < z_hist_.size(); ++i) // May need nested for loops for this one. One for the index and then one for each measurement at that index
    {
    }

    //setup options and solve
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 50;
    options.gradient_tolerance = 1e-8;
    options.function_tolerance = 1e-8;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

} // namespace mhe
