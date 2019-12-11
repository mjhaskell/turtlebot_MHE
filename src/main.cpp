#include "turtlebot_MHE/mhe_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mhe");
    ros::NodeHandle nh;

    MHENode node;

    ros::spin();

    return 0;
}
