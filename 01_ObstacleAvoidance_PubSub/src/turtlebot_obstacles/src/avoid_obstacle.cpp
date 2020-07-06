#include "ros/ros.h"
#include "turtlebot_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoid_obstacles");
    Turtlebot_Controller controller;
    ros::spin();
    return 0;
}
