#include "qr_code_detection_node.h"

QRCodeDetectionNode::QRCodeDetectionNode()
:m_pubcheckQRCodeDetection{m_node.advertise<driver_bot_cpp::qrCode>("qrCode", 10)}
{
    image_transport::ImageTransport imageTransport(m_node);
    m_subCamera = imageTransport.subscribe("cam_pub", 10, &QRCodeDetectionNode::CameraCallBack, this);
    std::cout << "OBJECT DETECTION OBJECT CREATED" << std::endl;   
}

void QRCodeDetectionNode::CameraCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    /*Receives message from 'camera' topic. 
        msg: Contains data from the image.
    returns:
        m_frame: matrix containing data about the pixels of the image
    */
    try
    {
        m_frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        std::cout << "image received IN QR CODE NODE" << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

bool QRCodeDetectionNode::Validate()
{
    /*Validates if image is empty or not*/
    if (!m_frame.empty())
    {
        std::cout << "image is valid IN QR CODE NODE" << std::endl;
        return true;
    }
    std::cout << "image is NOT valid IN QR CODE NODE" << std::endl;
    return false;
}

void QRCodeDetectionNode::Run()
{
    /*Runs script that reads out a QR code. If the qr code is detected
    and read succesfully, the data is published on the 'qrCode' topic*/
    ros::Rate rate{10};

    while (ros::ok())
    {
        //No Image received
        if (!Validate())
        {
            ros::spinOnce();
            rate.sleep();
            continue; //wait for frames to be valid
        }

        m_qrCodeDetection.CheckQRCode(m_frame); 
        bool checkQR = m_qrCodeDetection.GetCheckQRCode();

        //No QR Code detected
        if (!checkQR)
        {
            ros::spinOnce();
            continue;
        }

        struct quirc_data qrCodeData = m_qrCodeDetection.GetData();
        PublishQRCodeData(checkQR, qrCodeData);
        
        ros::spinOnce();
        rate.sleep();
    }
}

//--------------- SUPPORT FUNCTIONS ------------------//
void QRCodeDetectionNode::PublishQRCodeData(bool checkQRCodeDetection, struct quirc_data& qrCodeData)
{
    /*Publishes a bool and std::vector<uint8_t> on the 'qrCode' topic
    Args:
        checkQRCodeDetection: bool that is true if qr code is detected, else false
        qrCodeData: Contains data about the qrcode if checkQRCodeDetection is true*/

    if (checkQRCodeDetection)
    {
        /*Publishes overlap data*/
        driver_bot_cpp::qrCode msg;
        msg.checkQRCode = checkQRCodeDetection;
        std::vector<uint8_t> data(&qrCodeData.payload[0], & qrCodeData.payload[qrCodeData.payload_len]);
        
        msg.qrCode = data;
        m_pubcheckQRCodeDetection.publish(msg);
        std::cout << "published QR CODE IN QR CODE NODE: " << checkQRCodeDetection << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qrCodeDetection");
    QRCodeDetectionNode qrCodeDetectionNode;
    qrCodeDetectionNode.Run();
}
