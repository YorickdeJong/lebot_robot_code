#include "../include/run.h"
//#include <driver_bot_cpp/run.h>

Drive::Drive()
:m_subObjectAngle{m_node.subscribe("objectAngles", 10, &Drive::ObjectAngleCallback, this)}, 
m_neirestDistance{m_node.subscribe("minDistance", 10, &Drive::MinDistanceCallback, this)},
m_subGamma{m_node.subscribe("gamma", 10, &Drive::GammaCallback, this)}, 
m_subDist{m_node.subscribe("distance", 10, &Drive::DistToWallCallback, this)}, 
m_gamma{GAMMA_START_VALUE}, 
m_distToWall{DISTANCE_START_VALUE}, 
m_rightSide{INTIAL_VALUE_RIGHT_SIDE}, 
m_angle_new{ANGLE_NEW_START_VALUE}, 
m_sign{INITIAL_VELOCITY_DIRECTION},
m_minDistance{DISTANCE_START_VALUE}
{
    ActionsFactory actionsFactory;
    m_actions = actionsFactory.Create(MOTOR_TYPE);
    std::cout << "Constructed run" << std::endl;
}


void Drive::ObjectAngleCallback(const driver_bot_cpp::objectAngle::ConstPtr& msg)
{
    /*Callback for objectAngle message defined in angle_node/AngleDetection()
    Args:
        msg: containing data rightSide (bool), angle_new (float), sign (int)
    Returns:
        rightSide, which checks if object is on the rightside or not
        angle_new: adjusted angle for position of the object compared to the vehicle 
        sign: checks if yvelocity compared to the object is positive or negative 
    */
    m_rightSide = msg -> rightSide;
    m_angle_new = msg -> angle_new;
    m_sign = msg -> sign;
}

void Drive::MinDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
       /*Gets data from minDistance topic defined in distance_node/rplidar()
        Args:
            msg: contains distance to neirest object
        */
        m_minDistance = msg -> data;
        std::cout << "minimum distance to object: " << m_minDistance << std::endl;

}

void Drive::GammaCallback(const std_msgs::Float32::ConstPtr& msg)
{
    /* reads gamma callback messages from 'gamma'
    Args:
        msg: retrieves data from 'gamma' topic, which contains the angle between the robot frame and 
            the world frame

    Returns:
            gamma, which is the angle between robot frame and world frame
    */
    m_gamma = msg -> data;
    std::cout << "gamma is: " << m_gamma << std::endl;
}
void Drive::DistToWallCallback(const std_msgs::Float32::ConstPtr& msg)
{
    /* reads ranges call back message from 'distance'
    Args:
        msg: retrieves data from 'distance' topic, which contains the distance infront 
            of the vehicle, measured by the RPLIDAR

    Returns:
            distance between 11/6 and 1/6 pi and 0.15m < d < 12m
    */
    m_distToWall = msg -> data;
    std::cout << "distance to wall: " << m_distToWall << std::endl;
}


bool Drive::ValidationCallbacks()
{
    /*Validates received values from subscribers. If data is the same as the initialized data,
    an error message will be printed. 
    */
    if (m_rightSide != false && m_rightSide != true)
    {
        std::cout << "[ERROR] rigthSide in RunScript has not been set correctly" << std::endl;
        std::cout << "[VALUE]: " << m_rightSide << std::endl;
        return false;
    }

    if (m_angle_new == ANGLE_INCREMENT_START_VALUE)
    {
        std::cout << "[ERROR] angle_new in RunScript has not been set correctly" << std::endl;
        std::cout << "[VALUE]: " << m_angle_new << std::endl;
        return false;
    }


    if (m_sign == INITIAL_VELOCITY_DIRECTION)
    {
        std::cout << "[ERROR] sign in RunScript has not been set correctly" << std::endl;
        std::cout << "[VALUE]: " << m_sign << std::endl;
        return false;
    }

    if (m_minDistance == DISTANCE_START_VALUE)
    {
        std::cout << "[ERROR] minDistance in RunScript has not been set correctly" << std::endl;
        std::cout << "[VALUE]: " << m_minDistance << std::endl;
        return false;
    }

    if (m_distToWall == DISTANCE_START_VALUE)
    {
        std::cout << "[ERROR] WALL DISTANCE DATA HAS NOT BEEN RECEIVED" << std::endl;
        std::cout << "[VALUE]: " << m_distToWall << std::endl;
        return false;
    }

    if (m_gamma == GAMMA_START_VALUE)
    {
        std::cout << "[ERROR] GAMMA HAS NOT BEEN RECEIVED" << std::endl;
        std::cout << "[VALUE]: " << m_gamma << std::endl;
        return false;
    }
    return true;
}

void Drive::WallLeftAdjustRight(const float error, const float thresholdDist, const int velocity, const float thresholdAngle)
{
    /*Checks if wall is to the left of the vehicle and adjusts by turning right
    Args:
        error: allowable difference between robot and world frame 
        thresholdDist: allowable distance between neirest object and robot
        velocity: speed of robot, runs from 0 - 255 
    Returns:
        Corrected position of robot compared to object
    */
    if (!(m_rightSide) && m_sign == NEGATIVE_VELOCITY_DIRECTION && (m_gamma > error || m_minDistance < thresholdDist))
    {
        bool checkGamma = true;
        while (checkGamma && ros::ok())
        {
            std::cout << "Moving towards left wall, adjusting right as to not hit the wall!" << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(m_gamma, m_sign, velocity, thresholdAngle);
            ros::spinOnce(); //asking for subscriber data

        }
    }
}

void Drive::WallLeftAdjustLeft(const float error, const float thresholdDist, const int velocity, const float thresholdAngle)
{
    /*Checks if wall is to the left of the vehicle and adjusts by turning left
    Args:
        error: allowable difference between robot and world frame 
        thresholdDist: allowable distance between neirest object and robot
        velocity: speed of robot, runs from 0 - 255 
    Returns:
        Corrected position of robot compared to object
    */
    if (!(m_rightSide) && m_sign == POSITIVE_VELOCITY_DIRECTION && (m_gamma > error || m_minDistance < thresholdDist))
    {
        bool checkGamma = true;

        while (checkGamma && ros::ok())
        {
            std::cout << "Moving away from left wall, adjusting left as to stay on course!" << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(m_gamma, m_sign, velocity, thresholdAngle);
            ros::spinOnce(); //asking for subscriber data
        }
    }
}

void Drive::WallRightAdjustRight(const float error, const float thresholdDist, const int velocity, const float thresholdAngle)
{
    /*Checks if wall is to the right of the vehicle and adjusts by turning right
    Args:
        error: allowable difference between robot and world frame 
        thresholdDist: allowable distance between neirest object and robot
        velocity: speed of robot, runs from 0 - 255 
    Returns:
        Corrected position of robot compared to object
    */
    if (m_rightSide && m_sign == NEGATIVE_VELOCITY_DIRECTION && (m_gamma > error || m_minDistance < thresholdDist))
    {
        bool checkGamma = true;

        while (checkGamma && ros::ok())
        {
            std::cout << "Moving away from right wall, adjusting right as to stay on course!" << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(m_gamma, m_sign, velocity, thresholdAngle);
            ros::spinOnce(); //asking for subscriber data
        }
    }
}

void Drive::WallRightAdjustLeft(const float error, const float thresholdDist, const int velocity, const float thresholdAngle)
{
    /*Checks if wall is to the right of the vehicle and adjusts by turning left
    Args:
        error: allowable difference between robot and world frame 
        thresholdDist: allowable distance between neirest object and robot
        velocity: speed of robot, runs from 0 - 255 
    Returns:
        Corrected position of robot compared to object
    */
    if (m_rightSide && m_sign == POSITIVE_VELOCITY_DIRECTION && (m_gamma > error || m_minDistance < thresholdDist))
    {
        bool checkGamma = true;

        while (checkGamma && ros::ok())
        {
            std::cout << "Moving towards left wall, adjusting right as to not hit the wall" << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(m_gamma, m_sign, velocity, thresholdAngle);
            ros::spinOnce(); //asking for subscriber data
        }
    }
}

void Drive::WallRightAhead(const float thresholdDistance, const int velocity, const float thresholdAngle)
{
    /*Checks if wall is within a threshold distance to the front of the robot
    Args:
        thresholdDistance: threshold distance to the fron of the robot and an object
        velocity: speed of robot, runs from 0 - 255 
    Returns:
        Robot drives back untill a certain threshold is met, Robot turns untill a certain 
        threshold is met
    */
    int sign;
    bool checkGamma = true, checkDistToWall = true;
    float thresholdDistanceWall = 0.5;

    if (m_distToWall < thresholdDistance)
    {
        if (m_rightSide)
        {
            sign = -1; //Turn left, since wall is on the right side
        }
        else
        {
            sign = 1; //Turn right, since wall is on the left side
        }

        //MANOUVER: goes back untill threshold distance is met, then turns an continous path
        while (checkDistToWall && ros::ok()) 
        {
            std::cout << "Moving towards a wall, moving backwards!" << std::endl;
            checkDistToWall = m_actions -> CorrectForDistance(m_distToWall, velocity, thresholdDistanceWall);
            ros::spinOnce(); //asking for subscriber data
        }

        while (checkGamma && ros::ok())
        {
            std::cout << "Moving towards a wall, turning!" << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(m_gamma, sign, velocity, thresholdAngle);
            ros::spinOnce(); //asking for subscriber data
        }
    }
}

void Drive::AdjustPosition()
{
    /*Checks for input from lidar and camera and adjusts robots speed accordingly*/
    float minDistToWall = 0.50;
    float thresholdDistAhead = 0.6, thresholdAngle = (float)5/90;
    float error = (float)20/90;
    float thresholdDist = 0.35;
    float velocity = 70; //runs from 0 to 255 

    ros::Rate rate(1);
    
    while (!ValidationCallbacks() && ros::ok())
    {
        std::cout << "[ERROR] RUNSCRIPT NODE HAS NOT RECEIVED DATA YET" << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    while (m_minDistance < thresholdDistAhead && ros::ok()) 
    {
        //Checks where object is and adjust course accordingly
        WallRightAhead(minDistToWall, velocity, thresholdAngle);
        WallLeftAdjustRight(error, thresholdDist, velocity, thresholdAngle);
        WallLeftAdjustLeft(error, thresholdDist, velocity, thresholdAngle);
        WallRightAdjustRight(error, thresholdDist, velocity, thresholdAngle);
        WallRightAdjustLeft(error, thresholdDist, velocity, thresholdAngle);
        ros::spinOnce(); //asking for subscriber data
    }
    
    m_actions -> DriveForward(velocity);
}
void Drive::Run()
{
    /*Runs script, which makes the robot drive autonomously*/
    ros::Rate rate(10);

    while (ros::ok())
    {

        AdjustPosition();
        ros::spinOnce();
        rate.sleep();
    }

    m_actions -> Stop();
}


int main(int argc, char **argv)
{
    std::cout << "check" << std::endl;
    ros::init(argc, argv, "runScript");
    Drive drive{};
    drive.Run();
}