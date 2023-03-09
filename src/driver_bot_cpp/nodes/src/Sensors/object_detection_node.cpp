#include "object_detection_node.h"

ObjectDetection::ObjectDetection()
/*Runs camera simulation 
    Variables:
    self.filter: containes filter methods hsv, maska and bitwise

    self.draw: contains draw functions such as contour, circle and square

    self.camera: contains data about frame and checks the validaty

    self.overlap: contains information about overlap between contour and image
*/
:m_pubOverlap{m_node.advertise<std_msgs::Bool>("overlap", 10)},
m_frameAnalysis{}, m_draw{}, m_overlap{}
{
    image_transport::ImageTransport imageTransport(m_node);
    m_subCamera = imageTransport.subscribe("cam_pub", 10, &ObjectDetection::CameraCallBack, this);
    std::cout << "OBJECT DETECTION OBJECT CREATED" << std::endl;
}

void ObjectDetection::CameraCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    /*Receives message from 'camera' topic. 
        msg: Contains data from the image.
    returns:
        m_frame: matrix containing data about the pixels of the image
    */
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

void ObjectDetection::DataAnalyses(cv::Mat& frame)
{
    /* Calculates hue saturation value (hsv), masking for a blue color
       and retrieves the radius and center of a contour and a circle */
    cv::Mat hsv = m_frameAnalysis.HsvFilter(frame);
    cv::Mat maskingBlue = m_frameAnalysis.MaskingBlueFilter(hsv);

    auto [m_radiusContour, m_centerContour] = m_draw.Contour(maskingBlue, frame);
    auto [m_radiusCircle, m_centerCicle] = m_draw.Circle(maskingBlue, frame);

}

bool ObjectDetection::Validate()
{
    /*Validates if image is empty or not*/
    if (!m_frame.empty())
    {
        std::cout << "image is valid" << std::endl;
        return true;
    }
    std::cout << "image is NOT valid" << std::endl;
    return false;
}

void ObjectDetection::Detection()
{
    /*Runs script and publishes overlap data*/
    /*Calculates circle and contour (in circle form) of target object and returns an image */
    ros::Rate rate{10};

    while (!Validate() && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        continue; //wait for frames to be valid
    }

    while (Validate() && ros::ok())
    {
        bool checkBlue = DetectionCircle();
        PublishOverlap(checkBlue);
        ros::spinOnce();
        rate.sleep();
    }

}


//--------------- SUPPORT FUNCTIONS ------------------//
bool ObjectDetection::DetectionCircle()
{
    /*Detects if circle overlaps with contour
    returns:
        true if overlap is detected, false if not*/
    DataAnalyses(m_frame); //calculates radius and center of circle and contour
    return m_overlap.CircleInCircle(m_radiusContour, m_radiusCircle, m_centerContour, m_centerCicle);

}

void ObjectDetection::PublishOverlap(bool checkBlue)
{
    /*Publishes overlap data*/
    std_msgs::Bool msg;
    msg.data = checkBlue;
    m_pubOverlap.publish(msg);
    std::cout << "published overlap: " << checkBlue << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "objectDetection");
    ObjectDetection objectDetection;
    objectDetection.Detection();
}

