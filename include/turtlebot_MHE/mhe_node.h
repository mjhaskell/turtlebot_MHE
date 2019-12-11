#ifndef MHE_NODE_H
#define MHE_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <aruco_localization/MarkerMeasurementArray.h>
#include "turtlebot_MHE/mhe.h"

class MHENode
{
public:
    MHENode();
    ~MHENode();

protected:
    void measCallback(const aruco_localization::MarkerMeasurementArrayConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber meas_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher est_pub_;
    std::map<int, int> id2idx_;
    Meas z_cur_;
    Zidx z_idx_;
};

#endif // MHE_NODE_H
