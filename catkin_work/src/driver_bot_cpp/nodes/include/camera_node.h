#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include "../../support_classes/include/camera_operations.h"

class Camera
{
    public:
        Camera();
        ~Camera() = default;
        
        bool Validate(cv::VideoCapture& capture);
        void run();
        void PublishCamera();
        
    private:
        image_transport::Publisher m_publishCameraData;    
        ros::NodeHandle m_node;     
        cv::Mat m_frame;
        cv::VideoCapture m_capture;
};