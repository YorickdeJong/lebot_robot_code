//#include <driver_bot_cpp/angle_node.h>
#include "angle_node.h"

AngleDetection::AngleDetection()
:m_pubGamma{m_node.advertise<std_msgs::Float32>("gamma", 10)},
m_pubObjectAngle{m_node.advertise<driver_bot_cpp::objectAngle>("objectAngles", 10)},
m_subLidar{m_node.subscribe("rplidar", 10, &AngleDetection::LidarCallback, this)},
m_operationsAngle{}
{
}


//
void AngleDetection::LidarCallback(const driver_bot_cpp::lidar::ConstPtr &msg)
{
    /*Callback function containing data from rplidar
     *   Args:
     *       msg: contains data for distance and angle_increment
     */
    m_operationsAngle.SetDist(msg -> distance);
    m_operationsAngle.SetAngleIncrement(msg -> angle_increment);
    
}

void AngleDetection::Run()
{
    /*Functions runs script and publishes gamma and objectLocation*/
    ros::Rate rate(10);

    bool rightSide;
    float angleNew;
    int sign;
    
    while (ros::ok())
    {
        
        m_operationsAngle.Detection();
        rightSide = m_operationsAngle.GetRightSide();
        angleNew = m_operationsAngle.GetAngleNew();
        sign = m_operationsAngle.GetSign();

        PubGammaData(angleNew);
        PubObjectLocation(rightSide, angleNew, sign);
        ros::spinOnce();
        rate.sleep();
    }   
}

//additional functionality
void AngleDetection::PubGammaData(float angleNew)
{
    /*publishes angle between robot frame and world frame*/
    std_msgs::Float32 gamma;
    gamma.data = MAX_GAMMA_VALUE - angleNew;
    m_pubGamma.publish(gamma);
}

void AngleDetection::PubObjectLocation(bool rightSide, float angleNew, int sign)
{
    /*publishes data about orientation of vehicle compared to object
     *Args:
     *   rightSide: true if object is on the right side of the vehicle
     *
     *   angle_new: adjusted angle based on in which quadrant the object is located in the robot frame
     *   
     *   sign: 1 if y velocity is positive, -1 if negative
     */
    driver_bot_cpp::objectAngle msg{};
    msg.rightSide = rightSide;
    msg.angle_new = angleNew;
    msg.sign = sign;

    m_pubObjectAngle.publish(msg);
}

int main(int argc, char **argv)
{
    /*Initialize RPLidar object*/
    ros::init(argc, argv, "angleDetection");
    AngleDetection angles{};
    angles.Run();
    return 0;
}