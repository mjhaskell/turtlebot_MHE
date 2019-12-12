#ifndef MHE_H
#define MHE_H

#include <Eigen/Core>
#include <vector>
#define TIME_HORIZON 5
#define NUM_LANDMARKS 9

using namespace Eigen;

enum
{
    X = 0,
    Y = 1,
    THETA = 2,

    V = 0,
    W = 1,

    R = 0,
    PHI = 1,
};

typedef Eigen::Vector3d Pose;
typedef Eigen::Vector2d Input;
typedef Eigen::Matrix<double, 2, NUM_LANDMARKS> Meas;
typedef Eigen::Matrix<bool, TIME_HORIZON, NUM_LANDMARKS> Zidx;

class MHE
{
public:
    MHE();
    virtual ~MHE();
    Vector3d propagateState(const Pose& state, const Input& u, double dt);
    void update(const Pose& mu, const Meas& z, const Zidx& idx, const Input& u, double dt);
    void optimize();

    std::vector<Vector3d> pose_hist_;
    std::vector<Matrix<double, 2, NUM_LANDMARKS>> z_hist_;
//    Matrix<double, 3, Dynamic> pose_hist_;
//    Matrix<double, 2, Dynamic> z_hist_;
    Matrix<double, TIME_HORIZON, NUM_LANDMARKS> z_ind_;
private:
//    unsigned long N_;
    Eigen::Matrix3d R_inv_;
    Eigen::Matrix3d Omega_;
};

#endif // MHE_H 

