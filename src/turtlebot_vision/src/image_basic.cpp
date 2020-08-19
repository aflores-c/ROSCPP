#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
class cam_test
{

public:
    cam_test()
    {
        cv::VideoCapture cap(cv::CAP_ANY);
        if(!cap.isOpened())
        {   
            std::cout << "Cannot open the video cam" << std::endl;
        }

        double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

        std::cout << "Frame size: " << dWidth << "x" << dHeight;

        cv::namedWindow("FirstVideo", cv::WINDOW_AUTOSIZE);

        while (1)
        {
            cv::Mat frame;
            bool bSuccess = cap.read(frame);
            if(!bSuccess)
            {
                std::cout<< "Cannot read from video stream" << std::endl;
                break;
            }

            cv::imshow("MyVideo", frame);

            if(cv::waitKey(30) == 27)
            {
                std::cout<< "esc key is pressed by the user" << std::endl;
                break;
            }


        }

    }

    ~cam_test()
    {
        cv::destroyWindow("MyVideo");
    }
};

int main(int argc, char **argv)
{
    /* code */
    ros::init(argc, argv, "image_basic");
    cam_test cam_object;
    ROS_INFO("Cam tested");
    return 0;
}
