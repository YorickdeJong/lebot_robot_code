// include the necessary libraries
#include <wiringPi.h>
#include <ros/ros.h>
#include <numeric>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "driver_bot_cpp/distanceData.h"
#include "driver_bot_cpp/keyStrokes.h"

class EncoderNode{
    public:
        EncoderNode();
        ~EncoderNode() = default;


        static void handleEncoderInterruptWrapperA1();
        static void handleEncoderInterruptWrapperB1();
        static void handleEncoderInterruptWrapperA2();
        static void handleEncoderInterruptWrapperB2();
        static void handleEncoderInterruptWrapperA3();
        static void handleEncoderInterruptWrapperB3();
        static void handleEncoderInterruptWrapperA4();
        static void handleEncoderInterruptWrapperB4();

        void handleEncoderInterrupt(int pin, int& encoderCount, const int ENCODER_A, const int ENCODER_B);
        static EncoderNode* getInstance();
        void DistanceTravelled();
        void MeanDistanceTravelled();
        void KeyStrokesCallback(const driver_bot_cpp::keyStrokes::ConstPtr &msg);
        void PublishStopNode();
        void PublishEncoderData();
        void run();


    public:
        ros::NodeHandle m_node;
        ros::Publisher m_pubEncoderData;
        ros::Publisher m_pubStopNode;
        ros::Subscriber m_subKeyStrokes;

        float m_meanDistanceTravelled_M1;
        float m_meanDistanceTravelled_M2;
        float m_meanDistanceTravelled_M3;
        float m_meanDistanceTravelled_M4;

        std::vector<float> m_distanceTravelled_M1;
        std::vector<float> m_distanceTravelled_M2;
        std::vector<float> m_distanceTravelled_M3;
        std::vector<float> m_distanceTravelled_M4;

        static int currentInterruptPinA1;
        static int currentInterruptPinB1;
        static int currentInterruptPinA2; 
        static int currentInterruptPinB2;
        static int currentInterruptPinA3;
        static int currentInterruptPinB3;
        static int currentInterruptPinA4; 
        static int currentInterruptPinB4;


        static int m_encoderCountMotor1;
        static int m_encoderCountMotor2;
        static int m_encoderCountMotor3;
        static int m_encoderCountMotor4;
        
        float wheelRadiusCM;
        std::string m_key;
        static EncoderNode* instance;
};