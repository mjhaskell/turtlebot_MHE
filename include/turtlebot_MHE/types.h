#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <cmath>

struct PoseResidual
{
public:
    PoseResidual(const Eigen::Vector3d &x, const Eigen::Matrix3d &omega): x_{x}
    {
        xi_ = omega.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x, T* residual)
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose(x);
        Eigen::Map<Eigen::Matrix<T,1,3>> res(residual);
        res = (x_ - pose) * xi_;
        return true;
    }
protected:
    Eigen::Vector3d x_;
    Eigen::Matrix3d xi_;

};

struct MeasurementResidual
{
public:
    MeasurementResidual(const Eigen::Vector2d &z, const Eigen::Matrix2d &R_inv): z_{z}
    {
        xi_ = R_inv.llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x, T* residual)
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose(x);
        Eigen::Map<Eigen::Matrix<T,2,1>> res(residual);
        T range = pose.norm();
        T phi = wrap(atan2(pose(1), pose(0)) - pose(2)); //Need to wrap this
        Eigen::Matrix<T,2,1> z_hat;
        z_hat << range, phi;
        
        res = (z_ - z_hat) * xi_;
        return true;
    }

protected:
    Eigen::Vector2d z_;
    Eigen::Matrix3d xi_;
};

#endif // TYPES_H
