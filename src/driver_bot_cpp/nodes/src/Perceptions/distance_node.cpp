//#include <driver_bot_cpp/distance_node.h>
#include "distance_node.h"

DistanceDetection::DistanceDetection(const float angle1, const float angle2)
:m_pubDistance{m_node.advertise<std_msgs::Float32>("distance", 10)},
 m_pubMinDistance{m_node.advertise<std_msgs::Float32>("minDistance", 10)},
 m_subLidar{m_node.subscribe("rplidar", 10, &DistanceDetection::LidarCallback, this)},
 m_operationsDistance{angle1, angle2}
{
        
}

//Main functionalities
void DistanceDetection::LidarCallback(const driver_bot_cpp::lidar::ConstPtr& msg)
{
    /*Callback function containing data from rplidar
     *   Args:
     *       msg: contains data for distance and angle_increment
     */
    m_operationsDistance.SetDist(msg -> distance);
    m_operationsDistance.SetAngleIncrement(msg -> angle_increment);
}

void DistanceDetection::Run()
{
    /*Computes distance in range of 11/6 to 1/6 pi and minimum distance. Hereafter,
    the values are published on "distance" and "minDistance" topics.
    */
    float distanceRange;
    float minDistance;

    ros::Rate rate(10);

    while (ros::ok())
    {
        std::vector<float> angleArray = m_operationsDistance.DataAnalyses();
        distanceRange = m_operationsDistance.Detection(angleArray);
        std::vector<float> distance = m_operationsDistance.GetDist(); //can I prevent this copy here?
        minDistance = m_operationsDistance.MinimumDistance(distance);

        PublishDistanceRange(distanceRange);
        PublishMinDistance(minDistance);

        ros::spinOnce();
        rate.sleep();
    }
}

//Additional functionalities
void DistanceDetection::PublishDistanceRange(const float distance)
{
    /*Publishes distance for a specific angle range*/
    std_msgs::Float32 msg;
    msg.data = distance;
    m_pubDistance.publish(msg);
}

void DistanceDetection::PublishMinDistance(const float distance)
{
    /*Publishes minimum distance for 0 to 2 pi*/
    std_msgs::Float32 msg;
    msg.data = distance;
    m_pubMinDistance.publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "distanceDetection");
    float angle1 = ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT * M_PI, angle2 = ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT * M_PI;
    DistanceDetection dist{angle1, angle2};
    dist.Run();
    return 0;
}