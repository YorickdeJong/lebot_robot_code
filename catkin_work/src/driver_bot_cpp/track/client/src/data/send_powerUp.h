#include <ros/ros.h>
#include "driver_bot_cpp/qrCode.h"
#include "std_msgs/Int8.h"
#include <thread>
#include <chrono>
#include <ros/callback_queue.h>
#include <math.h>


class SendPowerUp
{
    public:
        SendPowerUp();
        ~SendPowerUp() = default;
        
        void QRCodeCallBack(const driver_bot_cpp::qrCode::ConstPtr& msg);
        void RoundCallback(const std_msgs::Int8::ConstPtr& msg);
        bool Validate();
        void powerUp();
        void PublishPowerUp();
        int powerUpGenerator();
        void Wait();
        void Empty();
        
    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subQRCode;
        ros::Subscriber m_subRound; 
        ros::Publisher m_pubPowerUp;
        std::vector<uint8_t> m_qrCodeData;

        bool m_checkQRCode; 
        int m_powerUp;
        int m_ROUND;
};