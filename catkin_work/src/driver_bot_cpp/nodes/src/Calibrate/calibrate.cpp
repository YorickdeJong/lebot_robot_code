//#include <driver_bot_cpp/calibrate.h>
#include "calibrate.h" 

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


Calibrate::Calibrate(const float angle1, const float angle2)
:m_subDistance{m_node.subscribe("rplidar", 10, &Calibrate::DistanceCallback, this)},
m_pubDetailedVelocityResults{m_node.advertise<driver_bot_cpp::distanceVelocity>("distanceVelocity", 10)},
m_operationsDistance{angle1, angle2},
m_angle1{angle1}, 
m_angle2{angle2}
{
    ActionsFactory actionsFactory;
    m_actions = actionsFactory.Create(MOTOR_TYPE);
    std::cout << "Calibration objects constructed" << std::endl;
}


void Calibrate::DistanceCallback(const driver_bot_cpp::lidar::ConstPtr& msg)
{
    /*Gets data from distance in range of angle1 and angle2
    Args:
        msg: contains data from 'distance' node that gives the distance and angle_increment

    */

    std::vector<float> angleArray;

    m_operationsDistance.SetDist(msg -> distance); //is message deleted after this scope, if not we can point to it.
    m_operationsDistance.SetAngleIncrement(msg -> angle_increment);
    angleArray = m_operationsDistance.DataAnalyses();
    m_minDistanceRange = m_operationsDistance.Detection(angleArray); //pass to operationsAngle to get its angle
    m_angleAtDistance = m_operationsDistance.AngleWithAssociatedDistance(m_minDistanceRange, angleArray); //calculates angle for certain distance

    if (m_angleAtDistance == -1)
    {
        std::cout << "m_angleAtDistance is NOT VALID" << std::endl;
    }
}



bool Calibrate::Validation()
{
    if (m_operationsDistance.GetDist()[0] == 20.0)
    {
        return false;
    }
    return true;
}
//-----------------MAIN FUNCIONALITY ----------------------//
bool Calibrate::Calibration(const int calibrationSteps)
{
    /*Calibrates speeds of robot for different motor inputs
    Args:
        calibrationSteps: amount of calibrations needed to be performed
    Returns:
        calibrated data (time, distance, velocity) that is published on the 'distanceVelocity' node
     */

    float beginningThresholdMeter = 2, thresholdMeter = 0.7;
    int initialSpeed = 10;

    if (calibrationSteps > MAX_CALIBRATION_STEPS)
    {
        std::cout << "Please choose calibrationSteps to be between 0 and 10" << std::endl;
        return false;
    }

    while (!Validation() && ros::ok())
    {
        std::cout << "[ERROR] INPUT DATA HAS NOT BEEN SET CORRECTLY" << std::endl;
        ros::spinOnce();
    }

    CheckDistance(beginningThresholdMeter);
    CountDown();

    for (int i{}; i < calibrationSteps; i++)
    {
        VelocityCalculation(thresholdMeter, initialSpeed, i);
        PublishDetailedVelocityData(m_pointDistance, m_pointTime);
        DriveToInitialPosition(beginningThresholdMeter);

    }

    return true;
}

void Calibrate::PublishDetailedVelocityData(std::vector<float>& distanceCalibration, std::vector<double>& timeCalibration)
{   
    /* Publishes distance, time and velocity on the 'distanceVelocity' node
    Args:
        distanceCalibration: vector containing data about the distance to the wall during calibration
        timeCalibration: vector containing data about the time duration of the calibration
    Returns:
        published data on the 'distanceVelocity' topic. The data is only published once, when the subscriber is ready
    
    */
    driver_bot_cpp::distanceVelocity msg;
    
    msg.time = m_pointTime;
    msg.distance = m_pointDistance;
    msg.velocity = Velocity(distanceCalibration, timeCalibration);

    while (ros::ok() && m_pubDetailedVelocityResults.getNumSubscribers() < 1) {
    // wait for a connection to publisher, since we only publish the message once
        std::cout << "WAITING FOR CONNECTION TO PUBLISHER" << std::endl;
    }
    m_pubDetailedVelocityResults.publish(msg);
    ClearData(); //clearing vectors for new message to be published

}

//-----------------SUPPORT FUNCTIONS ----------------------//
void Calibrate::CheckDistance(const float beginningThresholdMeter)
{
    /*Checks if, at the start of the calibraiton, the distance to the wall is bigger than 
    the beginning threshold
    Args:
        beginningThresholdMeter: initial distance threshold to the wall in meters 
    */
    while (ros::ok() && m_minDistanceRange < beginningThresholdMeter)
    {
        std::cout << "minDistance is: " << m_minDistanceRange << std::endl;
        std::cout << "Place the robot further away from the wall!" << std::endl;
        ros::spinOnce();
    }
}

void Calibrate::CountDown()
{
    /*If the distance has the starting requirements, a countdown begins
    before the calibration starts*/
    ros::Duration duration{4.5};
    ros::Time time = ros::Time::now() + duration;
    int count = COUNT_DOWN_TIME;

    while (time > ros::Time::now() && ros::ok())
    {
        std::cout << count << " seconds untill calibration starts!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        count -= 1; 
        ros::spinOnce();
    }
}

void Calibrate::VelocityCalculation(const float thresholdMeter, const int initialSpeed, const int calibrationStep)
{
    /*Calculates the velocity between every time point during the valibration
    Args:
        thresholdMeter: end threshold of every calibration, as to not make the robot
        collide with the wall
        intialSpeed: beginning speed of the calibration
        calibrationSteps: amount of calibrations that will be performed, increments the initialSpeed 
    Returns:
        Velocity between time points     
    */
    //velocity variables
    bool checkDistToWall = true;
    
    m_motorSpeed = initialSpeed * (calibrationStep + 1);
    ros::Rate rate(2); //calculates velocity 5 times / second
    double time = ros::Time::now().toSec(); //actions for beginning time 
    
    while (ros::ok() && checkDistToWall)
    {

        checkDistToWall = m_actions -> DriveToWall(m_minDistanceRange, m_motorSpeed, thresholdMeter);

        //save distance points
        SaveDistanceTimePoints(time);
        ros::spinOnce();
        rate.sleep();
    }
    m_actions -> Stop(); 

}

void Calibrate::DriveToInitialPosition(const float beginningThresholdMeter)
{
    /*When a calibration step is performed, the robot automatically drives back the the 
    beginning threshold and also positions itself perpendicular to the wall
    Args:
        beginningThresholdMeter: initial distance threshold to the wall in meters 
    */
    int driveBackSpeed = 60, sign;
    bool checkDistToWall = true, checkGamma = true;
    float gamma, thresholdAngle = (float) 1/30;

    while (ros::ok() && checkDistToWall)
    {
        checkDistToWall = m_actions -> CorrectForDistance(m_minDistanceRange, driveBackSpeed, beginningThresholdMeter);
        
        while (ros::ok() && checkGamma)
        {

            /*Check if angle occurs on left or rigth side.*/
            if (m_angleAtDistance <= m_angle1)
            {
                gamma = m_angleAtDistance;
                sign = NEGATIVE_VELOCITY_DIRECTION;
            }
            if (m_angleAtDistance >= m_angle2)
            {
                gamma = TWO_PI - m_angleAtDistance;
                sign = POSITIVE_VELOCITY_DIRECTION;
            }
            std::cout << "gamma is: " << gamma << std::endl;
            checkGamma = m_actions -> CorrectForAngleBetweenWorldAndRobotFrame(gamma, sign, 60, thresholdAngle);
            ros::spinOnce();
        }
        checkGamma = true; //reset gamma;
        ros::spinOnce();
    }
    m_actions -> Stop();
}

std::vector<float> Calibrate::Velocity(const std::vector<float>& distanceCalibration, const std::vector<double>& timeCalibration)
{
    /*Calculates the velocity between every time point during the valibration
    Args:
        distanceCalibration: vector containing distance of every measured step during calibration
        timeCalibration: vector containing time of every measured step during calibration
    Returns:
        Velocity between time points     
    */
    /*Calculates */

    std::vector<float> velocity;
    float velocityPoint, distance, time;

    for (int i{}; i < distanceCalibration.size() - 1; i++)
    {
        distance = fabsf((distanceCalibration[i + 1] - distanceCalibration[i]) / cos(m_angleAtDistance * M_PI)); 
        time = fabsf(timeCalibration[i + 1] - timeCalibration[i]);
        velocityPoint =  distance / time;

        velocity.emplace_back(velocityPoint);

        //check if velocity has been set correctly
        std::cout << "m_angleAtDistance: " << m_angleAtDistance << std::endl;
        std::cout << "velocity is: " << velocity[i] << std::endl;
        std::cout << "distance difference is: " << distanceCalibration[i + 1] - distanceCalibration[i] << std::endl;
        std::cout << "time difference is: " << timeCalibration[i + 1] - timeCalibration[i] << std::endl;
    }
    return velocity;
}

void Calibrate::SaveDistanceTimePoints(double time)
{
    /*saves distance and time points in m_pointDistance and m_pointTime vectors*/
    m_pointDistance.emplace_back(fabsf(m_minDistanceRange  / cos(m_angleAtDistance * M_PI))); //account for driving oblique
    std::cout << "time is: " << ros::Time::now().toSec() - time << std::endl;
    m_pointTime.emplace_back(ros::Time::now().toSec() - time);
}


void Calibrate::ClearData()
{
    /*Cleares vectors such that they don't contain data for the next calibration*/
    m_pointTime.clear();
    m_pointDistance.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration");
    const float angle1 = 0.05;
    const float angle2 = 1.95;
    int calibrationSteps = 10;

    Calibrate calibration{angle1, angle2};
    calibration.Calibration(calibrationSteps);
    return 0;
    
}   
