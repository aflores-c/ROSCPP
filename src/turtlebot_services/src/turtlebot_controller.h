#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlebot_services/Move.h"
#include <algorithm>

class Turtlebot_Controller
{

public:
    Turtlebot_Controller();

private:
    bool driving_forward;
    float distance_ = 0.0;

    void initialize_velocity();
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    bool MoveRobotService(turtlebot_services::MoveRequest &request, turtlebot_services::MoveResponse& response);

    ros::Subscriber scan_sub_;
    ros::Publisher vel_pub_;
    ros::ServiceServer move_robot;
    geometry_msgs::Twist linear_angular_vel_;
    ros::NodeHandle nh_;
    ros::Rate rate;
    ros::Time time_state;
};