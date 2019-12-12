#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>
#include <ceres/ceres.h>

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
    MeasurementResidual(const Eigen::Vector2d &z, const Eigen::Matrix2d &R): z_{z}
    {
        xi_ = R.inverse().llt().matrixL();
    }

    template<typename T>
    bool operator()(const T* const x, T* residual)
    {
        Eigen::Map<const Eigen::Matrix<T,3,1>> pose(x);
        Eigen::Map<Eigen::Matrix<T,2,1>> res(residual);
        Eigen::Matrix<T,2,1> z_hat{h(pose)};
        
        res = (z_ - z_hat) * xi_;
        return true;
    }

protected:
    Eigen::Vector2d z_;
    Eigen::Matrix3d xi_;
};

double wrap(double angle)
{
    static double pi{3.14159};
    angle -= 2*pi * floor((angle+pi) * 0.5/pi);
    return angle;
}

struct Pose
{
    enum
    {
        SIZE = 3,
    };
    Eigen::Matrix<double,SIZE,1> vec;
    Eigen::Map<double*> x;
    Eigen::Map<double*> y;
    Eigen::Map<double*> theta;

    Pose() :
        x{vec.data()},
        y{vec.data()+1},
        theta{vec.data()+2}
    {
        vec.setZero();
    }

    Pose& operator =(const Pose& other)
    {
        vec = other.vec;
        return *this;
    }

    Pose operator +(const Pose& other)
    {
        Pose out;
        out.x = x + other.x;
        out.y = y + other.y;
        out.theta = wrap(theta + other.theta);
        return out;
    }
        Eigen::Matrix2d R_inv = R.inverse();
    Pose& operator +=(const Pose& other)
    {
        x = x + other.x;
        y = y + other.y;
        theta = wrap(theta + other.theta);
        return *this;
    }

    Pose operator -(const Pose& other)
    {
        Pose out;
        out.x = x - other.x;
        out.y = x - other.y;
        out.theta = wrap(theta - other.theta);
    }
};

#endif // TYPES_H
