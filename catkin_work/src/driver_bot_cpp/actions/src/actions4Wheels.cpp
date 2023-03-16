#include "actions4Wheels.h"

Actions4Wheels::Actions4Wheels()
{
    MotorFactory motorFactory;
    m_motor = motorFactory.Create(MOTOR_TYPE);
    std::cout << "Actions object 4 Wheels created" << std::endl;
}

void Actions4Wheels::Stop()
{
    m_motor -> SetMotorInitialVelocity();
    m_motor -> SetCommandToMotorRelease(PIN_MOTOR_1, LOCATION_MOTOR_1, AdafruitDCMotor::kRelease);
    m_motor -> SetCommandToMotorRelease(PIN_MOTOR_2, LOCATION_MOTOR_2, AdafruitDCMotor::kRelease);
    m_motor -> SetCommandToMotorRelease(PIN_MOTOR_3, LOCATION_MOTOR_3, AdafruitDCMotor::kRelease);
    m_motor -> SetCommandToMotorRelease(PIN_MOTOR_4, LOCATION_MOTOR_4, AdafruitDCMotor::kRelease);
    std::cout << "Stopped and released motor" << std::endl;
}
void Actions4Wheels::DriveForward(const int velocity)
{
    /*Sets x velocity in robot frame to be positive
    Args:
        velocity: velocity in the x direction of the vehicle in the robot frame
    */
    m_motor -> SetMotorVelocity(velocity);
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_1, LOCATION_MOTOR_1, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_2, LOCATION_MOTOR_2, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward);
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_3, LOCATION_MOTOR_3, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward);
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_4, LOCATION_MOTOR_4, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward);
    std::cout << "Driving Forwards" << std::endl;

}
void Actions4Wheels::DriveBackward(const int velocity)
{
    /*Sets x velocity in robot frame to be negative
    Args:
        velocity: velocity in the x direction of the vehicle in the robot frame
    */
    m_motor -> SetMotorVelocity(velocity);
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_1, LOCATION_MOTOR_1, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward); 
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_2, LOCATION_MOTOR_2, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_3, LOCATION_MOTOR_3, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward); 
    m_motor -> SetCommandToMovementInXDirection(PIN_MOTOR_4, LOCATION_MOTOR_4, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    std::cout << "Driving Backwards" << std::endl;
}

void Actions4Wheels::TurnLeft(const int velocity)
{
    /*Sets positive z angular velocity in the robot frame
    Args:
        velocity: velocity in the z angular direction of the vehicle in the robot frame
    */
    m_motor -> SetMotorVelocity(velocity);
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_1, LOCATION_MOTOR_1, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_2, LOCATION_MOTOR_2, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_3, LOCATION_MOTOR_3, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_4, LOCATION_MOTOR_4, AdafruitDCMotor::kBackward, AdafruitDCMotor::kForward);  
    std::cout << "Turning Left" << std::endl;
}
void Actions4Wheels::TurnRight(const int velocity)
{
    /*Sets negaitve z angular velocity in the robot frame
    Args:
        velocity: velocity in the z angular direction of the vehicle in the robot frame
    */
    m_motor -> SetMotorVelocity(velocity);
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_1, LOCATION_MOTOR_1, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_2, LOCATION_MOTOR_2, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_3, LOCATION_MOTOR_3, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    m_motor -> SetCommandToMovementInZDirection(PIN_MOTOR_4, LOCATION_MOTOR_4, AdafruitDCMotor::kForward, AdafruitDCMotor::kBackward); 
    std::cout << "Turning Right" << std::endl;
}

bool Actions4Wheels::Turn90DegreesRight(const float gamma, const int velocity)
{
    /*When a target object is scanned by the camera, the vehicle turns by
    a certain threshold (90 degrees) to the right

    Args:
        gamma: angle between robot and world frame of reference 
        velocity: velocity in the z angular direction of the vehicle in the robot frame

    Returns: turned vehicle compared to target object
    */

    float threshold = 0.47;
    TurnRight(velocity);
    if (gamma < threshold)
    {
        std::cout << "Turning right, gamma is: " << gamma << std::endl;
        return true;
    }
    Stop();
    return false;
}
bool Actions4Wheels::Turn90DegreesLeft(const float gamma, const int velocity)
{
    /*When a target object is scanned by the camera, the vehicle turns by
    a certain threshold (90 degrees) to the left

    Args:
        gamma: angle between robot and world frame of reference 
        velocity: velocity in the z angular direction of the vehicle in the robot frame

    Returns: turned vehicle compared to target object
    */
    float threshold = 0.47;
    TurnLeft(velocity);
    if (gamma < threshold)
    {
        std::cout << "Turning left, gamma is: " << gamma << std::endl;
        return true;
    }
    Stop();
    return false;
}

bool Actions4Wheels::CorrectForDistance(const float distToWall, const int velocity, float threshold)
{
    /*Vehicle drives backwards untill the distance is below a threshold
    Args:
        distToWall: distance to the wall, measured in an angle range angle1 < theta < angle2 
        velocity: velocity in the z angular direction of the vehicle in the robot frame
        threshold: value that makes the robot stop moving in its direction

    Returns: vehicle that is a sufficient distance away from object
    */
    DriveBackward(velocity);
    if (distToWall < threshold)
    {
        std::cout << "Driving backwards, distance to the object is: " << distToWall << std::endl;
        return true;
    }
    Stop();
    return false;
}

bool Actions4Wheels::DriveToWall(const float distToWall, const int velocity, float threshold)
{
    /*Robot drives towards the wall in a straight line with its x component perpendicular to it.
    When a threshold is met, it stops
    Args:
        distToWall: distance to the wall, measured in an angle range angle1 < theta < angle2 
        velocity: velocity in the z angular direction of the vehicle in the robot frame
        threshold: value that makes the robot stop moving in its direction
        
    Return:
        vehicle that has driven towards the wall
    */
    DriveForward(velocity);
    if (distToWall > threshold)
    {
        std::cout << "Driving forwards, distance to the object is: " << distToWall << std::endl;
        return true;
    }
    Stop();
    return false;
}
bool Actions4Wheels::CorrectForAngleBetweenWorldAndRobotFrame(const float gamma, const int sign, const int velocity, float thresholdAngle)
{
    /*Vehicle turns untill gamma is below a certain threshold
        Args:
            sign: Direction in which has to be turned, positive means 
            gamma: angle between robot and world frame of reference

        Returns:
            Correct course measured against nearest object 
    */
    if (gamma > thresholdAngle)
    {
        if (sign > 0) //turning rightside
        {
            TurnRight(velocity);
        }
        else
        {
            TurnLeft(velocity);
        }
        std::cout << "Turning, correction for gamma is: " << gamma << std::endl;
        return true;
    }
    Stop();
    return false; 

}

std::unique_ptr<ActionsInterface> Actions4Wheels::Instantiate()
{
    /*Sets pointer for MotorFactory*/
    return std::make_unique<Actions4Wheels>();
}