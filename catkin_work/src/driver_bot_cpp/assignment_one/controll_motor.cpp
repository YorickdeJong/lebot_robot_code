#include "controll_motor.h"
using namespace std::chrono_literals;


#define I2CLINK "\
#!/bin/bash \n\
sudo chmod a+rw /dev/i2c-* \n\
"

MotorOnOff* MotorOnOff::s_instance = nullptr;

MotorOnOff::MotorOnOff(bool motorOne, bool motorTwo, bool motorThree, bool motorFour){
    
    CoupleMotors();
    m_motorOne = motorOne;
    m_motorTwo = motorTwo;
    m_motorThree = motorThree;
    m_motorFour = motorFour;

    s_instance = this;
}

void MotorOnOff::CoupleMotors()
{
    /*Couples motor, that has been set to a certain pin, to a 
    location on the vehicles frame. i.e. Left Forward, Left BackWard,
    Right Forward, Right Backward
    Args:
        motorPin: pin to which motor is connected 1, 2, 3 or 4
        position: position of the motor on the robot frame
    
    Returns:
        motor position that has been set to a certain pin 
    */
    if (m_motorOne)
    {
        m_motor1 = m_hat.getMotor(1);
    }
    if (m_motorTwo)
    {
        m_motor2 = m_hat.getMotor(2);
    }
    if (m_motorThree)
    {
        m_motor3 = m_hat.getMotor(3);
    }
    if (m_motorFour)
    {
        m_motor4 = m_hat.getMotor(4);
    }

}


void MotorOnOff::stopMotors() {
    if (m_motorOne){
        m_motor1->run(AdafruitDCMotor::kRelease);
    }
    if (m_motorTwo){
        m_motor2->run(AdafruitDCMotor::kRelease);
    }
    if (m_motorThree){
        m_motor3->run(AdafruitDCMotor::kRelease);
    }
    if (m_motorFour){
        m_motor4->run(AdafruitDCMotor::kRelease);
    }
}

void MotorOnOff::setMotors(){
    if (m_motorOne)
    {
        m_motor1->setSpeed (255);
        m_motor1->run(AdafruitDCMotor::kForward);
    }
    if (m_motorTwo)
    {
        m_motor2 ->setSpeed (255);
        m_motor2->run(AdafruitDCMotor::kBackward);
    }
    if (m_motorThree)
    {
        m_motor3->setSpeed (255);
        m_motor3->run(AdafruitDCMotor::kBackward);
    }
    if (m_motorFour)
    {
        m_motor4->setSpeed (255);
        m_motor4->run(AdafruitDCMotor::kForward);
    }
}
void MotorOnOff::runScript(){
    signal(SIGINT, signalHandler);
    setMotors();
    m_scriptRunning = true;

    while (m_scriptRunning){
        std::cout << "Running motors" << std::endl;
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // stopMotors();

}

void MotorOnOff::signalHandler(int signum) {
    std::cout << "Received signal: " << signum << ". Stopping the loop." << std::endl;
    if (s_instance) {
        s_instance->m_scriptRunning = false;
        s_instance->stopMotors();
    }
}
int main(int argc, char *argv[]) {
    // call script as: ./controll_motor 1 0 0 0 where 1 is true and 0 is false
    system(I2CLINK);
    
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " motorOne motorTwo motorThree motorFour\n";
        return 1;
    }

    bool motorOne = std::stoi(argv[1]) != 0;
    bool motorTwo = std::stoi(argv[2]) != 0;
    bool motorThree = std::stoi(argv[3]) != 0;
    bool motorFour = std::stoi(argv[4]) != 0;

    MotorOnOff motorController(motorOne, motorTwo, motorThree, motorFour);
    motorController.runScript();
    return 0;
}