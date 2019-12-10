#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>

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
