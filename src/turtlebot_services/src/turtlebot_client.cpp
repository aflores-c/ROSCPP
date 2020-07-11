
#include "ros/ros.h"
#include "turtlebot_services/Move.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_client");
    ros::NodeHandle nh;
    //This loop is just for simulation
    while (ros::Time::now().is_zero())
    {
        ros::spinOnce();
    }

    //Services instances
    ros::ServiceClient move_robot = nh.serviceClient<turtlebot_services::Move>("move_robot");
    if (!move_robot.waitForExistence(ros::Duration(5)))
    {
        ROS_ERROR("Move robot client error");
    }

    //Get Closest service request and response
    turtlebot_services::MoveResponse move_robot_resp;
    turtlebot_services::MoveRequest move_robot_request;
    move_robot_request.duration = ros::Duration(30); //30 seconds of turtlebot client duration
    move_robot.call(move_robot_request, move_robot_resp);
    ROS_INFO("Succes: %d", move_robot_resp.success);

    return 0;
}