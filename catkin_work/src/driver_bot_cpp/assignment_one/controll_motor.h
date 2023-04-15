#include "../motorDriver/include/adafruitmotorhat.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <csignal>

class MotorOnOff{
    public:
        MotorOnOff(bool motorOne, bool motorTwo, bool motorThree, bool motorFour);
        static void signalHandler(int signum);
        void CoupleMotors();
        void stopMotors();
        void setMotors();
        void runScript();

    private:
        std::shared_ptr<AdafruitDCMotor> m_motor1; 
        std::shared_ptr<AdafruitDCMotor> m_motor2; 
        std::shared_ptr<AdafruitDCMotor> m_motor3; 
        std::shared_ptr<AdafruitDCMotor> m_motor4;     
        AdafruitMotorHAT m_hat;

        bool m_motorOne;
        bool m_motorTwo;
        bool m_motorThree;
        bool m_motorFour;

        bool m_scriptRunning;
        static MotorOnOff* s_instance;
};