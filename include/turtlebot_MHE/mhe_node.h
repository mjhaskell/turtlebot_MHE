#ifndef MHE_NODE_H
#define MHE_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <aruco_localization/MarkerMeasurementArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "turtlebot_MHE/mhe.h"

class MHENode
{
public:
    MHENode();
    ~MHENode();

protected:
    void measCallback(const aruco_localization::MarkerMeasurementArrayConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void aruco110Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco121Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco245Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco248Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco25Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco55Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco5Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco64Callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void aruco76Callback(const geometry_msgs::PoseStampedConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber meas_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber aruco_110_sub_;
    ros::Subscriber aruco_121_sub_;
    ros::Subscriber aruco_245_sub_;
    ros::Subscriber aruco_248_sub_;
    ros::Subscriber aruco_25_sub_;
    ros::Subscriber aruco_55_sub_;
    ros::Subscriber aruco_5_sub_;
    ros::Subscriber aruco_64_sub_;
    ros::Subscriber aruco_76_sub_;
    ros::Subscriber file_flag_sub_;
//    ros::Publisher est_pub_;
    std::map<int, int> id2idx_;
    std::map<int, bool> lm_init_;
    mhe::MHE estimator_;
    mhe::Meas z_cur_;
    mhe::Zidx z_idx_;
    mhe::Pose odom_;
};

#endif // MHE_NODE_H
