#include "turtlebot_MHE/mhe.h"
#include <ceres/ceres.h>

template<typename T>
T wrap(T angle)
{
    static double pi{3.14159};
    angle -= 2*pi * floor((angle+pi) * 0.5/pi);
    return angle;
}

struct PoseResidual
{
public:
    PoseResidual(const Eigen::Vector3d &x, const Eigen::Matrix3d &omega): x_{x}
    {
        xi_ = omega.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x, T* residual) const
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose(x);
        Eigen::Map<Eigen::Matrix<T,1,3>> res(residual);
        Eigen::Matrix<T,3,1> temp{x_ - pose};
        temp(2) = wrap(temp(2));
        res = temp.transpose() * xi_;
        return true;
    }
protected:
    Eigen::Vector3d x_;
    Eigen::Matrix3d xi_;

};

struct MeasurementResidual
{
public:
    MeasurementResidual(const Eigen::Vector2d &z, const Eigen::Vector2d &lm, const Eigen::Matrix2d &R_inv): z_{z}, lm_{lm}
    {
        xi_ = R_inv.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x, T* residual) const
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose(x);
        Eigen::Map<Eigen::Matrix<T,2,1>> res(residual);
        T range = pose.norm();
        T phi = wrap(atan2(pose(1) - lm_(1), pose(0) - lm_(0)) - pose(2)); 
        Eigen::Matrix<T,2,1> z_hat;
        z_hat << range, phi;
        Eigen::Matrix<T, 2, 1> temp{z_ - z_hat};
        temp(1) = wrap(temp(1));
        
        res = temp.transpose() * xi_;
        return true;
    }

protected:
    Eigen::Vector2d z_, lm_;
    Eigen::Matrix2d xi_;
};

typedef ceres::AutoDiffCostFunction<PoseResidual,3,3> PoseCostFunction;
typedef ceres::AutoDiffCostFunction<MeasurementResidual, 2, 3> MeasurementCostFunction;

namespace mhe
{

MHE::MHE()
{
    // set default parameters
    Omega_ = Eigen::Vector3d{1, 1, 0.5}.asDiagonal();
    R_inv_ = Eigen::Vector2d{1/0.35, 1/0.07}.asDiagonal();
    lms_ = Meas::Zero();
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
    int counter = 0;
    for(i; i < z_hist_.size(); ++i) // May need nested for loops for this one. One for the index and then one for each measurement at that index
    {
        for(int j{0}; j < NUM_LANDMARKS; ++j)
        {
            if(z_ind_(counter,j))
            {
                //Need the true landmark locations to be stored somewhere
                MeasurementCostFunction *cost_function{new MeasurementCostFunction(new MeasurementResidual(z_hist_[i].col(j), Eigen::Vector2d::Zero(), R_inv_))};
                problem.AddResidualBlock(cost_function, NULL, pose_hist_[i].data());
            }
        }
        ++counter;
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

void MHE::initializeLandmark(int index, const Eigen::Vector2d &lm)
{
    lms_.col(index) = lm;
}
} // namespace mhe
