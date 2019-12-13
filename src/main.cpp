#include "turtlebot_MHE/mhe_node.h"
#include <signal.h>
#include <stdio.h>
#include <atomic>

std::atomic<bool> quit(false);

void sigintHandler(int sig)
{
    std::cout << "Handling Ctrl + C signal. Writing data to file." << std::endl;
    quit.store(true);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mhe");
    ros::NodeHandle nh;

    MHENode node;

    signal(SIGINT, sigintHandler);

    ros::spin();

    return 0;
}
