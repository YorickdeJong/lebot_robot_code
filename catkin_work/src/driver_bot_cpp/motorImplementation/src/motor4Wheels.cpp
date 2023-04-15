#include "motor4Wheels.h"

#define I2CLINK "\
#!/bin/bash \n\
sudo chmod a+rw /dev/i2c-* \n\
"

Motor4Wheels::Motor4Wheels()
{
    system(I2CLINK);
    SetMotorsToPosition();
    std::cout << "4 Wheel motor object constructed" << std::endl;
}

void Motor4Wheels::SetMotorInitialVelocity()
{
    /*Sets motor speed to 0*/
    m_motorLF -> setSpeed(MIN_VELOCITY);
    m_motorRF -> setSpeed(MIN_VELOCITY);
    m_motorLB -> setSpeed(MIN_VELOCITY);
    m_motorRB -> setSpeed(MIN_VELOCITY);
}

void Motor4Wheels::SetMotorVelocity(const int velocity)
{
    /*Set scaled motor velocity
    Args:
        velocity: value from 0 - 100 that sets motor speed*/
    if (velocity > MAX_VELOCITY || velocity < MIN_VELOCITY)
    {
        std::invalid_argument("[Erorr] please provide a velocity between 0 and 100");
    }

    int scaledVelocity = Scale(velocity); 
    m_motorLF -> setSpeed(scaledVelocity);
    m_motorRF -> setSpeed(scaledVelocity);
    m_motorLB -> setSpeed(scaledVelocity);
    m_motorRB -> setSpeed(scaledVelocity);    

}

float Motor4Wheels::Scale(const int velocity)
{
    /*Scales velocity to a universal 0 - 100 range*/
    float scale = ((float)velocity / (float)NORMALISED_RANGE) * MAXIMUM_RANGE_MOTOR;
    return scale;
}

std::unique_ptr<MotorInterface> Motor4Wheels::Instantiate()
{
    /*Sets pointer for MotorFactory*/
    return std::make_unique<Motor4Wheels>();
}

//Couple motor position to correct motor
std::shared_ptr<AdafruitDCMotor> Motor4Wheels::GetLeftFrontMotor() 
{
    return m_motorLF;
}
std::shared_ptr<AdafruitDCMotor> Motor4Wheels::GetRightFrontMotor() 
{   
    return m_motorRF;
}
std::shared_ptr<AdafruitDCMotor> Motor4Wheels::GetLeftBackMotor()
{
    return m_motorLB;
}
std::shared_ptr<AdafruitDCMotor> Motor4Wheels::GetRightBackMotor()
{
    return m_motorRB;
}

//--------------SUPPORT FUNCTIONS ---------------------//
void Motor4Wheels::SetMotorsToPosition()
{
    /*Calls functions for all motors that couple motor to command, which has been 
    set to a certain pin, to a location on the vehicles frame.
    i.e. Left Forward, Left BackWard, Right Forward, Right Backward*/
    CoupleMotorPosition(PIN_MOTOR_1, LOCATION_MOTOR_1);
    CoupleMotorPosition(PIN_MOTOR_2, LOCATION_MOTOR_2);
    CoupleMotorPosition(PIN_MOTOR_3, LOCATION_MOTOR_3);
    CoupleMotorPosition(PIN_MOTOR_4, LOCATION_MOTOR_4);
}


void Motor4Wheels::CoupleMotorPosition(const int motorPin, const std::string& position)
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
    if (position == "LF")
    {
        if (m_motorLF = m_hat.getMotor(motorPin))
        {
            std::cout << "motor " << motorPin << " found at location " << position << std::endl;
        }
    }
    if (position == "RF")
    {
        if (m_motorRF = m_hat.getMotor(motorPin))
        {
            std::cout << "motor " << motorPin << " found at location " << position << std::endl;
        }
    }
    if (position == "LB")
    {
        if (m_motorLB = m_hat.getMotor(motorPin))
        {
            std::cout << "motor " << motorPin << " found at location " << position << std::endl;
        }
    }
    if (position == "RB")
    {
        if (m_motorRB = m_hat.getMotor(motorPin))
        {
            std::cout << "motor " << motorPin << " found at location " << position << std::endl;
        }
    }
}

void Motor4Wheels::CoupleCommandToMovementInXDirection(const int motorPin, const std::string& position, const std::string& positionInput, AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo)
{
    /*Couples Forward drive command to motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame 
        positionInput: inputs where the motor might be located on the frame, for 2wheels, 
        some motors won't be set as there are only two motors
        commandOne: is either kBackward or kForward depending on the direction the car has to move 
        commandTwo: is either kBackward or kForward depending on the direction the car has to move 

    Returns:
        drive command to make the robot move in the x direction 
    */
    if (position == positionInput)
        {
        if (motorPin == 1) 
        {
            m_motorLF -> run(commandOne);
        }
        if (motorPin == 2)
        {
            m_motorLB -> run(commandTwo);
        }
        if (motorPin == 3)
        {
            m_motorRB -> run(commandOne);
        } 
        if (motorPin == 4)
        {
            m_motorRF -> run(commandTwo);
        }
    }
}

void Motor4Wheels::SetCommandToMovementInXDirection(const int motorPin, const std::string& position, AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo)
{
    /*Couples Forward drive command to all motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame 
        commandOne: is either kBackward or kForward depending on the direction the car has to move 
        commandTwo: is either kBackward or kForward depending on the direction the car has to move 

    Returns:
        sets all motors to the drive command in the x direcion
    */
    CoupleCommandToMovementInXDirection(motorPin, position, "LF", commandOne, commandTwo);
    CoupleCommandToMovementInXDirection(motorPin, position, "LB", commandOne, commandTwo);

    //Switching commands around, since the position of the motor has altered
    CoupleCommandToMovementInXDirection(motorPin, position, "RF", commandTwo, commandOne);
    CoupleCommandToMovementInXDirection(motorPin, position, "RB", commandTwo, commandOne);
}


void Motor4Wheels::CoupleCommandToMovementInZDirection(const int motorPin, const std::string& position, const std::string& positionInput, AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo)
{
    /*Couples Forward drive command to motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame 
        positionInput: inputs where the motor might be located on the frame, for 2wheels, 
        some motors won't be set as there are only two motors
        commandOne: is either kBackward or kForward depending on the direction the car has to move 
        commandTwo: is either kBackward or kForward depending on the direction the car has to move 

    Returns:
        drive command to make the robot turn in the z direction 
    */
    if (position == positionInput)
        {
        if (motorPin == 1)
        {
            m_motorLF -> run(commandOne);
        }
        if (motorPin == 2)
        {
            m_motorLB -> run(commandTwo);
        }
        if (motorPin == 3)
        {
            m_motorRB -> run(commandTwo);
        } 
        if (motorPin == 4)
        {
            m_motorRF -> run(commandOne);
        }
    }
}

void Motor4Wheels::SetCommandToMovementInZDirection(const int motorPin, const std::string& position, AdafruitDCMotor::Command commandOne, AdafruitDCMotor::Command commandTwo)
{
    /*Couples Forward drive command to motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame 
        commandOne: is either kBackward or kForward depending on the direction the car has to move 
        commandTwo: is either kBackward or kForward depending on the direction the car has to move 

    Returns:
       sets all motors to the drive command in the z direcion
    */
    CoupleCommandToMovementInZDirection(motorPin, position, "LF", commandOne, commandTwo);
    CoupleCommandToMovementInZDirection(motorPin, position, "LB", commandOne, commandTwo);

    //Switching commands around, since the position of the motor has altered
    CoupleCommandToMovementInZDirection(motorPin, position, "RF", commandTwo, commandOne);
    CoupleCommandToMovementInZDirection(motorPin, position, "RB", commandTwo, commandOne);
}

void Motor4Wheels::CoupleCommandToMotorRelease(const int motorPin, const std::string& position, const std::string& positionInput, AdafruitDCMotor::Command releaseCommand)
{
    /*Couples Forward drive command to motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame 
        positionInput: inputs where the motor might be located on the frame, for 2wheels, 
        some motors won't be set as there are only two motors
        commandOne: is either kBackward or kForward depending on the direction the car has to move 
        commandTwo: is either kBackward or kForward depending on the direction the car has to move 

    Returns:
       released motors 
    */
    if (position == positionInput)
        {
        if (motorPin == 1)
        {
            m_motorLF -> run(releaseCommand);
        }
        if (motorPin == 2)
        {
            m_motorLB -> run(releaseCommand);
        }
        if (motorPin == 3)
        {
            m_motorRB -> run(releaseCommand);
        } 
        if (motorPin == 4)
        {
            m_motorRF -> run(releaseCommand);
        }
    }
}


void Motor4Wheels::SetCommandToMotorRelease(const int motorPin, const std::string& position, AdafruitDCMotor::Command releaseCommand)
{
    /*Couples Forward drive command to motor pin and its location
    Args:
        motorPin: connected pin on the arduino board
        position: Position of the wheel on the car frame
        releaseCommand: releases motor 

    Returns:
        sets all motors to be released
    */
    CoupleCommandToMotorRelease(motorPin, position, "LF", releaseCommand);
    CoupleCommandToMotorRelease(motorPin, position, "LB", releaseCommand);

    //Switching commands around, since the position of the motor has altered
    CoupleCommandToMotorRelease(motorPin, position, "RF", releaseCommand);
    CoupleCommandToMotorRelease(motorPin, position, "RB", releaseCommand);
}
