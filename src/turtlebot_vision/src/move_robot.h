#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MoveRobot
{

public:
    MoveRobot() : rate(10)
    {
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_ = nh_.subscribe("/cmd_vel", 10, &MoveRobot::cmd_callback, this);
    }

    void cmd_callback(const geometry_msgs::Twist &cmd_msg)
    {
        linear_angular_vel_.linear = cmd_msg.linear;
        linear_angular_vel_.angular = cmd_msg.angular;
    }

    bool compare_twist_commands(const geometry_msgs::Twist &twist1, const geometry_msgs::Twist &twist2)
    {
        bool LX = twist1.linear.x == twist2.linear.x;
        bool LY = twist1.linear.y == twist2.linear.y;
        bool LZ = twist1.linear.z == twist2.linear.z;
        bool AX = twist1.angular.x == twist2.angular.x;
        bool AY = twist1.angular.y == twist2.angular.y;
        bool AZ = twist1.angular.z == twist2.angular.z;

        bool equal = LX && LY && LZ && AX && AY && AZ;
        if (!equal)
            ROS_WARN("The current twist is not the same as the one sent, resending");
        return equal;
    }

    void move(const geometry_msgs::Twist &twist)
    {
        /*
        bool current_equal_to_new = 0;
        while ((!current_equal_to_new) && (!shutdown_detected_))
        {
            pub_.publish(twist);
            current_equal_to_new = compare_twist_commands(linear_angular_vel_, twist);
            rate.sleep();
            ros::spinOnce();
        }*/
        pub_.publish(twist);
    }

    void clean()
    {
        //stop robot
        linear_angular_vel_.angular.z = 0.0;
        linear_angular_vel_.linear.x = 0.0;
        shutdown_detected_ = 1;
        move(linear_angular_vel_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    geometry_msgs::Twist linear_angular_vel_;
    ros::Rate rate;
    bool shutdown_detected_ = 0;
};