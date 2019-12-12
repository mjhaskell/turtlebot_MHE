#include "turtlebot_MHE/mhe_node.h"

MHENode::MHENode()
{
    z_idx_.setZero();
    odom_.setZero();

    id2idx_[5] = 0;
    id2idx_[25] = 1;
    id2idx_[55] = 2;
    id2idx_[64] = 3;
    id2idx_[76] = 4;
    id2idx_[110] = 5;
    id2idx_[121] = 6;
    id2idx_[245] = 7;
    id2idx_[248] = 8;

    meas_sub_ = nh_.subscribe("aruco/measurements", 1, &MHENode::measCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &MHENode::odomCallback, this);

//    pub_ = nh_.advertise<std_msgs::Bool>("topic", 1);
}

MHENode::~MHENode()
{

}

void MHENode::measCallback(const aruco_localization::MarkerMeasurementArrayConstPtr &msg)
{
    for (int i{0}; i < TIME_HORIZON-1; ++i)
        z_idx_.col(i) = z_idx_.col(i+1);
    z_idx_.col(TIME_HORIZON-1).setZero();

    z_cur_.setZero();
    for (auto const& pose : msg->poses)
    {
        int idx{id2idx_[pose.aruco_id]};
        z_idx_(TIME_HORIZON-1, idx) = true;
        Pose pt{pose.position.x, pose.position.y, pose.position.z};
        z_cur_.col(idx) << pt.norm(), atan2(pt(0), pt(2));
    }
}

void MHENode::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    static double prev_time{0};
    if (prev_time == 0.0)
    {
        prev_time = msg->header.stamp.toSec();
        return;
    }
    double now{msg->header.stamp.toSec()};
    double dt{now - prev_time};
    prev_time = now;

    odom_ << msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             asin(msg->pose.pose.orientation.z) * 2;

    Input u_odom{msg->twist.twist.linear.x, msg->twist.twist.angular.z};

    mhe_.update(odom_, z_cur_, z_idx_, u_odom, dt);
}

