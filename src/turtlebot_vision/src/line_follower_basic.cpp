#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "image_transport/image_transport.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "move_robot.h"

static const std::string OPENCV_WINDOW = "Image window";

class LineFollower
{

public:
    LineFollower() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &LineFollower::camera_callback, this);

    }
    ~LineFollower()
    {
        cv::destroyAllWindows();
        move_robot.clean();
    }

    void camera_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            //Make a copy of the
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat original_image;
        cv::resize(cv_ptr->image, original_image, cv::Size(), 0.5, 0.5);

        int width = original_image.size().width;
        int height = original_image.size().height;

        int descentre = 200;
        int rows_to_watch = 40;
        cv::Rect roi;
        roi.x = 0;
        roi.y = (height / 2) + descentre;
        roi.height = rows_to_watch;
        roi.width = width;

        cv::Mat cropped_image = original_image(roi);
        cv::Mat hsv_image;
        cv::cvtColor(cropped_image, hsv_image, CV_BGR2HSV);

        cv::Scalar yellow_lower(20, 100, 100);
        cv::Scalar yellow_upper(50, 255, 255);
        cv::Scalar white_lower(0, 0, 100);
        cv::Scalar white_upper(0, 0, 255);

        //thresholding
        cv::Mat threshold_image_yellow, threshold_image_white, threshold_yellow_white;
        cv::Mat result;
        cv::inRange(hsv_image, yellow_lower, yellow_upper, threshold_image_yellow);
        cv::inRange(hsv_image, white_lower, white_upper, threshold_image_white);

        cv::GaussianBlur(threshold_image_yellow, threshold_image_yellow, cv::Size(3, 3), 0);
        cv::GaussianBlur(threshold_image_white, threshold_image_white, cv::Size(3, 3), 0);
        cv::bitwise_or(threshold_image_white, threshold_image_yellow, threshold_yellow_white);

        cv::bitwise_and(cropped_image, cropped_image, result, threshold_yellow_white);

        cv::Moments m = cv::moments(threshold_yellow_white, false);
        float cx_pos;
        float cy_pos;

        cx_pos = m.m10 / m.m00;
        cy_pos = m.m01 / m.m00;

        cv::Point p(cx_pos, cy_pos);
        cv::circle(result, p, 10, cv::Scalar(128, 0, 0), -1);
        cv::imshow(OPENCV_WINDOW, original_image);
        //cv::imshow("HSV", hsv_image);
        cv::imshow("Mask", threshold_yellow_white);
        cv::imshow("RES", result);
        cv::waitKey(30);

        float error_x = cx_pos - width/2;
        twist_.linear.x = 0.2;
        twist_.angular.z = -error_x/100;
        ROS_INFO("ANGULAR VALUE SENT -> %f", twist_.angular.z);
        move_robot.move(twist_);

    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv_bridge::CvImagePtr cv_ptr;
    geometry_msgs::Twist twist_;
    MoveRobot move_robot;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    LineFollower ic;
    ros::spin();
    return 0;
}
