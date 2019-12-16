#include "turtlebot_MHE/mhe.h"
#include <ceres/ceres.h>
#include <fstream>

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
        temp(mhe::THETA) = wrap(temp(mhe::THETA));
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
        Eigen::Matrix<T,2,1> diff; // = lm_ - pose.segment<2>(mhe::X);
        diff << lm_(mhe::X) - pose(mhe::X), lm_(mhe::Y) - pose(mhe::Y);
        Eigen::Map<Eigen::Matrix<T,2,1>> res(residual);
        T range = diff.norm();
        T phi = wrap(atan2(diff(mhe::Y), diff(mhe::X)) - pose(mhe::THETA));
        Eigen::Matrix<T,2,1> z_hat{range, phi};
        Eigen::Matrix<T, 2, 1> temp{z_ - z_hat};
        temp(mhe::PHI) = wrap(temp(mhe::PHI));
        
        res = temp.transpose() * xi_;
        return true;
    }

protected:
    Eigen::Vector2d z_, lm_;
    Eigen::Matrix2d xi_;
};

struct OdomResidual
{
public:
    OdomResidual(const Eigen::Vector3d &x, const Eigen::Matrix3d &omega): x_{x}
    {
        xi_ = omega.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x1, const T* const x2, T* residual) const
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose1(x1), pose2(x2);
        Eigen::Map<Eigen::Matrix<T,1,3>> res(residual);
        Eigen::Matrix<T,3,1> diff{pose2 - pose1};
        diff(mhe::THETA) = wrap(diff(mhe::THETA));
        Eigen::Matrix<T,3,1> temp{x_ - diff};
        temp(mhe::THETA) = wrap(temp(mhe::THETA));
        res = temp.transpose() * xi_;
        return true;
    }
protected:
    Eigen::Vector3d x_;
    Eigen::Matrix3d xi_;

};

typedef ceres::AutoDiffCostFunction<PoseResidual,3,3> PoseCostFunction;
typedef ceres::AutoDiffCostFunction<MeasurementResidual, 2, 3> MeasurementCostFunction;
typedef ceres::AutoDiffCostFunction<OdomResidual,3,3,3> OdomCostFunction;

namespace mhe
{

MHE::MHE()
{
    // set default parameters
    Omega_ = Eigen::Vector3d{1, 1, 0.5}.asDiagonal();
    R_inv_ = Eigen::Vector2d{1/0.35, 1/0.07}.asDiagonal();
    lms_ = Meas::Zero();
    mu_ = Pose{2.5, -1.7, 0.175}; //These values from lab. Put these in the params file
    pose_hist_.push_back(mu_);
}

MHE::~MHE()
{
    writeFile();
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
    out(THETA) = u(W) * dt;
    odom_hist_.push_back(out);
    out = state + out;
    out(THETA) = wrap(out(THETA));
    return out;
}

void MHE::update(const Pose &mu, const Meas &z, const Zidx& idx, const Input &u, double dt)
{
//    Pose mu_bar{propagateState(mu, u, dt)};
    mu_ = propagateState(mu_, u, dt);

    // pose_hist_.push_back(mu);
    pose_hist_.push_back(mu_);
    z_hist_.push_back(z);
    z_ind_ = idx;

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

//    //set up odometry residuals
//    i = std::max(0, int(odom_hist_.size() - TIME_HORIZON));
//    Eigen::Matrix3d Om = Eigen::Vector3d{1e1, 1e1, 5e-1}.asDiagonal();
//    for(i; i < odom_hist_.size(); ++i)
//    {
//        OdomCostFunction * cost_function{new OdomCostFunction(new OdomResidual(odom_hist_[i], Omega_))};
//        problem.AddResidualBlock(cost_function, NULL, pose_hist_[i].data(), pose_hist_[i+1].data());
//    }

    //set up measurement residuals
    i = std::max(0, int(z_hist_.size() - TIME_HORIZON));
    int counter = 0;
    for(i; i < z_hist_.size(); ++i) // May need nested for loops for this one. One for the index and then one for each measurement at that index
    {
        for(int j{0}; j < NUM_LANDMARKS; ++j)
        {
//            if(z_ind_(TIME_HORIZON - counter - 1,j))
            if(z_ind_(counter,j))
            {
//                if(z_hist_[i].col(j)(0) == 0.0 || z_hist_[i].col(j)(1) == 0.0)
//                    int debug =  1;
                //Need the true landmark locations to be stored somewhere
                MeasurementCostFunction *cost_function{new MeasurementCostFunction(new MeasurementResidual(z_hist_[i].col(j), lms_.col(j), R_inv_))};
                problem.AddResidualBlock(cost_function, NULL, pose_hist_[i+1].data());
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

void MHE::writeFile()
{
    std::ofstream file;
    file.open("/tmp/MHE_landmarks.txt");
    file << lms_.transpose();
    file.close();

    file.open("/tmp/MHE_outputs.txt");
    for (Pose pose : pose_hist_)
        file << pose.transpose() << std::endl;
    file.close();
}
} // namespace mhe
