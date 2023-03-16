#include "camera_Subscriber.h"


CameraSubscriber::CameraSubscriber()
{
    image_transport::ImageTransport imageTransport(m_node);
    m_subCamera = imageTransport.subscribe("cam_pub", 10, &CameraSubscriber::CameraCallBack, this);
}

void CameraSubscriber::CameraCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        m_frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        std::cout << "image received" << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool CameraSubscriber::Validate()
{
    if (!m_frame.empty())
    {
        std::cout << "image is valid" << std::endl;
        return true;
    }
    std::cout << "image is NOT valid" << std::endl;
    return false;
}

 
void CameraSubscriber::run()
{
    
    ros::Rate rate(10);
    int result = 0;
    while (ros::ok())
    {
        std::string im_name = "src/driver_bot_cpp/nodes/src/Sensors/figures/picture_" + std::to_string(result) +  ".png";

        if (Validate())
        {
            if (!cv::imwrite (im_name, m_frame))
            {
                result += 0;
                std::cout<<"Image can not be saved as '"<<im_name<<"'\n";
            }
            else
            {
                //image succesfully saved
                std::cout<<"Image saved in '"<<im_name<<"'\n";
                result += 1;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cameraSubscriber");
    CameraSubscriber cameraSubscriber;
    cameraSubscriber.run();
}