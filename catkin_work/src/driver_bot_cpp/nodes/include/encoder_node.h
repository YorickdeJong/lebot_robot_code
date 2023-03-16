// include the necessary libraries
#include <wiringPi.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "driver_bot_cpp/distanceData.h"

class EncoderNode{
    public:
        EncoderNode();
        ~EncoderNode() = default;


        void handleEncoderInterrupt(int pin);
        static int currentInterruptPinA;
        static int currentInterruptPinB;
        static void handleEncoderInterruptWrapperA();
        static void handleEncoderInterruptWrapperB();
        static EncoderNode* getInstance();
        void DistanceTravelled();
        void KeyStrokesCallback(const std_msgs::String::ConstPtr &msg);
        void PublishStopNode();
        void PublishEncoderData();
        void run();


    public:
        ros::NodeHandle m_node;
        ros::Publisher m_pubEncoderData;
        ros::Publisher m_pubStopNode;
        ros::Subscriber m_subKeyStrokes;

        float m_distanceTravelled;
        float wheelRadiusCM;
        static int m_encoderCount;
        std::string m_key;
        static EncoderNode* instance;

};