#include "turtlebot_MHE/mhe_node.h"

MHENode::MHENode()
{
    id2idx_[5] = 0;
    id2idx_[25] = 1;
    id2idx_[55] = 2;
    id2idx_[64] = 3;
    id2idx_[76] = 4;
    id2idx_[110] = 5;
    id2idx_[121] = 6;
    id2idx_[245] = 7;
    id2idx_[248] = 8;
}

MHENode::~MHENode()
{

}

void MHENode::measCallback(const aruco_localization::MarkerMeasurementArrayConstPtr &msg)
{
    z_idx_.setZero();
    for (auto const& pose : msg->poses)
    {
        int idx{id2idx_[pose.aruco_id]};
        z_idx_(idx) = true;
        z_cur_.col(idx) << pose.position.x, pose.position.y; // TODO: fix this
    }
}

void MHENode::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{

}

