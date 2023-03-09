#include <ros/ros.h>
#include "driver_bot_cpp/qrCode.h"
#include "std_msgs/Int32.h"
#include <thread>
#include <chrono>
#include <ros/callback_queue.h>
#include <math.h>


class CPPTOJS
{
    public:
        CPPTOJS();
        ~CPPTOJS() = default;
        
        void QRCodeCallBack(const driver_bot_cpp::qrCode::ConstPtr& msg);
        bool Validate();
        void Round();
        void Run();
        void PublishRound(int round);
        void Wait();
        void Empty();
        
    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subQRCode; 
        ros::Publisher m_pubRound;
        std::vector<uint8_t> m_qrCodeData;

        bool m_checkQRCode; 
        int m_round;
        int RACE_ROUNDS;
};