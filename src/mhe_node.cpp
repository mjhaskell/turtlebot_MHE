#include "turtlebot_MHE/mhe_node.h"
#include <ros/console.h>

MHENode::MHENode() :
    nh_private_("~")
{
    z_cur_.setZero();
    z_idx_.setZero();
    odom_.setZero();
    last_idx_.setZero();

    id2idx_[5] = 0;
    id2idx_[25] = 1;
    id2idx_[55] = 2;
    id2idx_[64] = 3;
    id2idx_[76] = 4;
    id2idx_[110] = 5;
    id2idx_[121] = 6;
    id2idx_[245] = 7;
    id2idx_[248] = 8;

    lm_init_[5] = false;
    lm_init_[25] = false;
    lm_init_[55] = false;
    lm_init_[64] = false;
    lm_init_[76] = false;
    lm_init_[110] = false;
    lm_init_[121] = false;
    lm_init_[245] = false;
    lm_init_[248] = false;

    bool load_successful{true};
    double x0{0}, y0{0}, theta0{0};
    double x{10}, y{10}, theta{5};
    double sig_r{0.1}, sig_phi{0.01};
    double sx{1}, sy{1}, st{1};
    load_successful *= nh_private_.getParam("x0", x0);
    load_successful *= nh_private_.getParam("y0", y0);
    load_successful *= nh_private_.getParam("theta0", theta0);
    load_successful *= nh_private_.getParam("Omega_x", x);
    load_successful *= nh_private_.getParam("Omega_y", y);
    load_successful *= nh_private_.getParam("Omega_theta", theta);
    load_successful *= nh_private_.getParam("sigma_r", sig_r);
    load_successful *= nh_private_.getParam("sigma_phi", sig_phi);
    load_successful *= nh_private_.getParam("slew_x", sx);
    load_successful *= nh_private_.getParam("slew_y", sy);
    load_successful *= nh_private_.getParam("slew_theta", st);
    if (!load_successful)
        ROS_WARN("Failed to load params.");

    mhe::Pose pose0{x0, y0, theta0};
    mhe::Pose Omega{x, y, theta};
    mhe::Pose Slew{sx, sy, st};
    estimator_.setParams(pose0, Omega, Slew, sig_r, sig_phi);

    meas_sub_ = nh_.subscribe("aruco/measurements", 1, &MHENode::measCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &MHENode::odomCallback, this);
    aruco_110_sub_ = nh_.subscribe("ArUco_110_ned", 1, &MHENode::aruco110Callback, this);
    aruco_121_sub_ = nh_.subscribe("ArUco_121_ned", 1, &MHENode::aruco121Callback, this);
    aruco_245_sub_ = nh_.subscribe("ArUco_245_ned", 1, &MHENode::aruco245Callback, this);
    aruco_248_sub_ = nh_.subscribe("ArUco_248_ned", 1, &MHENode::aruco248Callback, this);
    aruco_25_sub_ = nh_.subscribe("ArUco_25_ned", 1, &MHENode::aruco25Callback, this);
    aruco_55_sub_ = nh_.subscribe("ArUco_55_ned", 1, &MHENode::aruco55Callback, this);
    aruco_5_sub_ = nh_.subscribe("ArUco_5_ned", 1, &MHENode::aruco5Callback, this);
    aruco_64_sub_ = nh_.subscribe("ArUco_64_ned", 1, &MHENode::aruco64Callback, this);
    aruco_76_sub_ = nh_.subscribe("ArUco_76_ned", 1, &MHENode::aruco76Callback, this);
//    pub_ = nh_.advertise<std_msgs::Bool>("topic", 1);
}

MHENode::~MHENode()
{

}

void MHENode::measCallback(const aruco_localization::MarkerMeasurementArrayConstPtr &msg)
{
    z_cur_.setZero();
    last_idx_.setZero();
    for (auto const& pose : msg->poses)
    {
        int idx{id2idx_[pose.aruco_id]};
        last_idx_(idx) = true;
        mhe::Pose pt{pose.position.x, pose.position.y, pose.position.z};
        z_cur_.col(idx) << pt.norm() * 10 , -atan2(pt(0), pt(2));
    }
}

void MHENode::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //Update odometry stuff
    static double prev_time{0};
    if (prev_time == 0.0)
    {
        prev_time = msg->header.stamp.toSec();
        return;
    }
    double now{msg->header.stamp.toSec()};
    double dt{now - prev_time};
    prev_time = now;

    //Shift measurements
    for (int i{0}; i < TIME_HORIZON-1; ++i)
        z_idx_.row(i) = z_idx_.row(i+1);
    z_idx_.row(TIME_HORIZON-1) = last_idx_;

    odom_ << msg->pose.pose.position.y + 3.14558,
             msg->pose.pose.position.x + 2.142763,
             -asin(msg->pose.pose.orientation.z) * 2;

    mhe::Input u_odom{msg->twist.twist.linear.x, msg->twist.twist.angular.z};

    estimator_.update(odom_, z_cur_, z_idx_, u_odom, dt);
}

void MHENode::aruco110Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[110])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[110];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[110] = true;
    }
}

void MHENode::aruco121Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[121])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[121];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[121] = true;
    }
}

void MHENode::aruco245Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[245])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[245];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[245] = true;
    }
}

void MHENode::aruco248Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[248])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[248];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[248] = true;
    }
}

void MHENode::aruco25Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[25])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[25];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[25] = true;
    }
}

void MHENode::aruco55Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[55])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[55];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[55] = true;
    }
}

void MHENode::aruco5Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[5])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[5];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[5] = true;
    }
}
void MHENode::aruco64Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[64])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[64];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[64] = true;
    }
}

void MHENode::aruco76Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(!lm_init_[76])
    {
    Eigen::Vector2d lm;
    lm << msg->pose.position.y, msg->pose.position.x;
    int idx = id2idx_[76];
    estimator_.initializeLandmark(idx, lm);
    lm_init_[76] = true;
    }
}
