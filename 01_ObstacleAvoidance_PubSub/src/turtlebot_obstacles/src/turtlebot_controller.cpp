#include "turtlebot_controller.h"

Turtlebot_Controller::Turtlebot_Controller() : rate(5)
{
    time_state = ros::Time::now();
    scan_sub_ = nh_.subscribe("/scan", 10, &Turtlebot_Controller::scan_callback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    driving_forward = 1;
    initialize_velocity();
}

void Turtlebot_Controller::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    //Get the distance of 20 values and takes the minimum value
    int middle_value = int(scan_msg->ranges.size() / 2);
    int min_value = middle_value - 30;
    int max_value = middle_value + 30;
    int num_values = max_value - min_value + 1;
    std::vector<float> distances;
    for (int i = 0; i < num_values; i++)
    {
        distances.push_back(scan_msg->ranges[min_value + i]);
    }
    distance_ = *std::min_element(distances.begin(), distances.end());

    //Drive forward or turns the robot
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

    //Reset distances
    distance_ = 0.0;
    distances.clear();
}

void Turtlebot_Controller::initialize_velocity()
{
    linear_angular_vel_.angular.z = 0.0;
    linear_angular_vel_.linear.x = 0.0;
}
