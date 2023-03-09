#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "driver_bot_cpp/qrCode.h"

class QRCodeSubscriber
{
    public:
        QRCodeSubscriber();
        ~QRCodeSubscriber() = default;
        
        void QRCodeCallBack(const driver_bot_cpp::qrCode& msg);
        bool Validate();
        void Run();

    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subQRCode; 
        std::vector<uint8_t> m_qrCodeData;
        bool m_checkQRCode; 
};