#include "camera_node.h"

Camera::Camera()
{
    image_transport::ImageTransport m_imageTransport(m_node);
    m_publishCameraData = m_imageTransport.advertise("cam_pub", 10);
}

bool Camera::Validate(cv::VideoCapture& capture)
{
    /*Validates if image is empty or not*/
    if (capture.isOpened())
    {
        std::cout << "Camera Opened IN CAMERA NODE" << std::endl;
        return true;
    }
    else
    {
        std::cout << "camera is not opened IN CAMERA NODE" << std::endl;
        return false;
    }
    
}

void Camera::run()
{
    /*Runs script which opens the camera, validates if it's open, 
    pushes image data to a matrix and publishes the outcome*/
    int imageShow;
    m_node.param("image_show", imageShow, 0);
    
    cv::VideoCapture capture(0, cv::CAP_V4L2); //opens camera that is connected
    ros::Rate rate(10);

    while (m_node.ok())
    {
        if (!Validate(capture))
        {
            rate.sleep();
            continue; //camera is not opened, skip rest of code
        }

        capture >> m_frame; //transfer data from cameraOperations to camera
        
        //show image
        if (imageShow)
        {
            cv::imshow("Cam View", m_frame);
            cv::waitKey(30);
        }
        if (!m_frame.empty())
        {
            std::cout << "publishing image in CAMERA NODE" << std::endl;
            PublishCamera();
        }
        rate.sleep();
    }
}

void Camera::PublishCamera()
{
    /*Publishes frame data*/
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_frame).toImageMsg();
    m_publishCameraData.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    Camera camera;
    std::cout << "check" << std::endl;
    camera.run();
}