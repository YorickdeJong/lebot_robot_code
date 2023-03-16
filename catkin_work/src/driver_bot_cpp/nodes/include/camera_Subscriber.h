#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


class CameraSubscriber
{
    public:
        CameraSubscriber();
        ~CameraSubscriber() = default;
        
        void CameraCallBack(const sensor_msgs::ImageConstPtr& msg);
        bool Validate();
        void run();

    private:
        ros::NodeHandle m_node;
        cv::Mat m_frame;
        image_transport::Subscriber m_subCamera; 
};