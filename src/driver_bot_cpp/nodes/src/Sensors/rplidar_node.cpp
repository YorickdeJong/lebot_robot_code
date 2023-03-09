//#include <driver_bot_cpp/rplidar_node.h>
#include "rplidar_node.h"

RPLidar::RPLidar()
: m_dist{DISTANCE_START_VALUE}, m_angleIncrement{ANGLE_INCREMENT_START_VALUE}, 
m_subLidar{m_node.subscribe("scan", 10, &RPLidar::Read, this)},
m_subKeyStrokes{m_node.subscribe("key_strokes", 10, &RPLidar::KeyStrokesCallback, this)},
m_pubValData{m_node.advertise<driver_bot_cpp::lidar>("rplidar", 20)},
m_pubStopNode{m_node.advertise<std_msgs::Bool>("stopNode", 20)} 
{
    std::cout << "rplidar node constructed" << std::endl;
}


void RPLidar::KeyStrokesCallback(const std_msgs::String::ConstPtr &msg) 
{
    if (msg->data.length() == 0) {
        return;
    }
    m_key = msg->data;
}


//General Sensor Functions
void RPLidar::Read(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /*Returns multiple callbacks and validates if the data is correct*/

    DistanceCallback(msg);
    AngleCallback(msg);
    Validate();
}


void RPLidar::Validate()
{
    /*Validates if received values for m_dist and m_angleIncrement are correct*/
    if (m_dist[0] == DISTANCE_START_VALUE)
    {
        ROS_WARN("[ERROR] DISTANCE HAS NOT BEEN RECEIVED");
    }
    if (m_angleIncrement == ANGLE_INCREMENT_START_VALUE)
    {
        ROS_WARN("[ERROR] ANGLE_INCREMENT HAS NOT BEEN RECEIVED");
    }
}


//Support Functions
void RPLidar::DistanceCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /*Reads ranges call back message from 'scan'
     *Args:
     *   msg: retrieves data from 'scan' topic, which contains the distance at a certain angle, measured by the RPLIDAR
     *
     *Returns:
     *   distance between 0 and 2 pi and 0.15m < d < 12m
     */
    m_dist = msg -> ranges;
}

void RPLidar::AngleCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    /*Reads angle_increment callback messages from 'scan'
     *Args:
     *   msg: retrieves data from 'scan' topic, which contains the distance at a certain angle, measured by the RPLIDAR
     *   
     *Returns:
     *   angle increment in between measured scan
     */
    m_angleIncrement = msg -> angle_increment;
}

void RPLidar::PublishValidatedData()
{
    /*Publishes altered message with distance and angle_increment*/
    driver_bot_cpp::lidar msg;

    if (m_dist.empty() || m_angleIncrement == ANGLE_INCREMENT_START_VALUE)
    {
        return;
    }

    std::cout << "PUBLISHED VALIDATED DATA" << m_dist[0] << std::endl;
    msg.distance = m_dist; //copy -> otherwise ros breaks
    msg.angle_increment = m_angleIncrement;
    msg.type = "lidar";
    m_pubValData.publish(msg);

    
}


void RPLidar::PublishStopNode() 
{
    /*Publishes a message to stop the node*/
    std_msgs::Bool msg;
    msg.data = true;
    m_pubStopNode.publish(msg);
    std::cout << "Published stop node message" << std::endl;
}

void RPLidar::Run()
{
    /*Runs script with a while loop that exits when ctrl + c is pressed*/
    ros::Rate rate (20);

    while (ros::ok() && !m_stopNode)
    {
        PublishValidatedData();
        ros::spinOnce();
        rate.sleep();

        // Check if the ` key is pressed
        if (m_key == "`") {
            ROS_INFO("Stopping RPLidar node");
            break; // Exit the while loop and stop the node
        }
    }

    PublishStopNode();
    ros::shutdown();
}



int main(int argc, char **argv)
{
    /*Initialize RPLidar object*/
    ros::init(argc, argv, "distance");
    RPLidar lidar{};
    lidar.Run();
}