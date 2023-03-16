#include "../../support_classes/include/qr_code_detection.h"
#include <ros/ros.h>
#include "driver_bot_cpp/qrCode.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class QRCodeDetectionNode
{
    public:
        QRCodeDetectionNode();
        ~QRCodeDetectionNode() = default;
        void Run();
    
    private:    
        void CameraCallBack(const sensor_msgs::ImageConstPtr& msg);
        bool Validate();
        void PublishQRCodeData(bool checkQRCodeDetection, struct quirc_data& qrCodeData);

    private:
        bool m_checkQRCodeDetection;
        QRCodeDetection m_qrCodeDetection; 
        ros::NodeHandle m_node;
        ros::Publisher m_pubcheckQRCodeDetection;
        image_transport::Subscriber m_subCamera; 
        cv::Mat m_frame;
};  