#include "turtlebot_MHE/mhe.h"

double wrap(double angle)
{
    static double pi{3.14159};
    angle -= 2*pi * floor((angle+pi) * 0.5/pi);
    return angle;
}

MHE::MHE()
{

}

MHE::~MHE()
{

}

Vector3d MHE::propagateState(const Pose &state, const Input &u, double dt)
{
    double st{sin(state(THETA))};
    double ct{cos(state(THETA))};
    Vector3d out;
    out(X) = u(V) * ct * dt;
    out(Y) = u(V) * st * dt;
    out(THETA) = wrap(u(W) * dt);
    return out;
}

void MHE::update(const Pose &mu, const Meas &z, const Zidx& idx, const Input &u, double dt)
{
    Pose mu_bar{propagateState(mu, u, dt)};

    pose_hist_.push_back(mu_bar);
    z_hist_.push_back(z);

    optimize();
}

void MHE::optimize()
{

}

