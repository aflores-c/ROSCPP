#include "turtlebot_controller.h"

Turtlebot_Controller::Turtlebot_Controller() : rate(5)
{
    time_state = ros::Time::now();
    scan_sub_ = nh_.subscribe("/scan", 10, &Turtlebot_Controller::scan_callback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    move_robot = nh_.advertiseService("move_robot", &Turtlebot_Controller::MoveRobotService, this);
    driving_forward = 1;
    initialize_velocity();
}

void Turtlebot_Controller::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    //Obtain the minimum distance from the robot to an object in front of the robot
    int middle_pos = int(scan_msg->ranges.size() / 2); //middle position of the laser scan
    int min_pos = middle_pos - 30;                     // get 30 values to the left
    int max_pos = middle_pos + 30;                     // get 30 values to the right
    int num_values = max_pos - min_pos + 1;            //get the number of values
    std::vector<float> distances;
    for (int i = 0; i < num_values; i++)
    {
        distances.push_back(scan_msg->ranges[min_pos + i]); // get all the distances
    }
    distance_ = *std::min_element(distances.begin(), distances.end()); //obtain the minimun distance
    distances.clear(); //reset the distances
}

bool Turtlebot_Controller::MoveRobotService(turtlebot_services::MoveRequest &request, turtlebot_services::MoveResponse &response)
{
    ROS_INFO("MoveRobotService called");
    ros::Time start = ros::Time::now();
    ros::Time endwait = start + request.duration;

    
    while (start < endwait)
    {

        if (driving_forward)
        {
            if (distance_ < 0.4 || ros::Time::now() > time_state)
            {
                driving_forward = 0;
                time_state = ros::Time::now() + ros::Duration(3);
            }
        }
        else
        {
            if (ros::Time::now() > time_state)
            {
                driving_forward = 1;
                time_state = ros::Time::now() + ros::Duration(20);
            }
        }

        if (driving_forward)
        {
            linear_angular_vel_.linear.x = -0.2;
            linear_angular_vel_.angular.z = 0.0;
        }
        else
        {
            linear_angular_vel_.angular.z = 0.4;
            linear_angular_vel_.linear.x = 0.0;
        }

        //Publish the distances
        vel_pub_.publish(linear_angular_vel_);
        ROS_INFO("Dist: %f - vel.x: %f - vel.z: %f", distance_, linear_angular_vel_.linear.x, linear_angular_vel_.angular.z);

        start = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    initialize_velocity();
    response.success = 1;
    return true;
}

void Turtlebot_Controller::initialize_velocity()
{
    linear_angular_vel_.angular.z = 0.0;
    linear_angular_vel_.linear.x = 0.0;
}
