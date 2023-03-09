#include "plot_data_transformation_node.h"

DataTransformationPlot::DataTransformationPlot()
: m_subDistance{m_node.subscribe("rplidar", 20, &DataTransformationPlot::DistanceCallback, this)},
m_subStopNode{m_node.subscribe("stopNode", 20, &DataTransformationPlot::StopNodeCallback, this)},
m_pubLidarData{m_node.advertise<driver_bot_cpp::distanceVelocity>("lidarData", 20)},
m_beginTime{ros::Time::now().toSec()}, 
m_stopNode{false},
m_operationsDistance{ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT, ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT},
m_adjustTimeToStartAtZero{0}
{
    /* Constructor for the DataTransformationPlot class
    * Args:
    *   m_subDistance: subscriber to the distance topic
    *   m_subStopNode: subscriber to the stopNode topic
    *   m_pubLidarData: publisher to the lidarData topic
    *   m_beginTime: time when the node is started
    *   m_stopNode: boolean to stop the node
    */
}


void DataTransformationPlot::StopNodeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    /* Function stops the node when the stopNode message is received
    */
    std::cout << "STOP NODE MESSAGE TRANSORMATION PLOT" << msg -> data << std::endl;

    m_stopNode = msg->data;
    std::cout << "Received stopNode message" << std::endl;
}


void DataTransformationPlot::DistanceCallback(const driver_bot_cpp::lidar::ConstPtr& msg) 
{
    /* Callback reads msg data which contains information about the distance of all the objects in a 360 degree 2D view
    * The callback then calculates the minimum distance of the objects in a specific range of angles and publishes the data
    */
    double currentTime = ros::Time::now().toSec();
    std::vector<float> distance =  msg -> distance;
    float minDistance = m_operationsDistance.MinimumDistance(distance);

    if (m_type.empty()) {  // Check if m_type is already set
        m_type = msg->type;
    }

    if (m_adjustTimeToStartAtZero == 0.0) {
        m_adjustTimeToStartAtZero = currentTime - m_beginTime;
    }

    m_time.emplace_back(currentTime - m_beginTime - m_adjustTimeToStartAtZero);
    m_distance.emplace_back(minDistance);

    std::cout << "MIN DISTANCE TRANSFORMATION PLOT: " << minDistance << std::endl;
}   

void DataTransformationPlot::VelocityCalculation()
{
    /* Function calculates velocity based on time and distance
    * Args:
    *   time: array containing time values
    *   distance: array containing distance values
    * Returns:
    *   velocity: array containing velocity values
    */

    if (m_distance.size() <= 1) {
        // there are not enough elements in m_distance to calculate velocity
        std::cout << "NOT ENOUGH ELEMENTS IN m_distance TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
        return;
    } 
    else if (m_distance.size() - 1 > m_velocity.max_size()) {
        std::cout << "TOO MANY ELEMENTS TO RESIZE m_velocity IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    // Set the interval size to 5 data points
    const int interval_size = 3;

    double sum_distance = 0.0;
    double sum_time = 0.0;
    for (size_t i = 0; i < m_distance.size() - interval_size; i += interval_size) {
        // Calculate the sum of distances and times in the current interval
        for (size_t j = 0; j < interval_size; ++j) {
            sum_distance += m_distance[i + j + 1] - m_distance[i + j];
            sum_time += m_time[i + j + 1] - m_time[i + j];
        }

        // Calculate the average velocity over the current interval and add it to the velocity array
        m_velocity.push_back(fabs(sum_distance / sum_time));

        // Reset the sum of distances and times for the next interval
        sum_distance = 0.0;
        sum_time = 0.0;
    }

    // Calculate the time intervals for the velocity plot
    m_timeVelocityPlot.push_back(0.0);
    for (size_t i = 0; i < m_velocity.size() - 1; ++i) {
        double interval_time = 0.0;
        for (size_t j = 0; j < interval_size; ++j) {
            interval_time += m_time[i * interval_size + j + 1] - m_time[i * interval_size + j];
        }
        m_timeVelocityPlot.push_back(m_timeVelocityPlot.back() + interval_time);
    }
}


void DataTransformationPlot::PublishLidarData()
{
    /* Function publishes velocity data
    * Args:
    *   time: array containing time values
    *   distance: array containing distance values
    *   velocity: array containing velocity values
    */
    //Calculates the velocity before publishing
    VelocityCalculation();

    if (m_time.size() == 0 || m_distance.size() == 0 || m_velocity.size() == 0) {
        std::cout << "NO DATA TO PUBLISH" << std::endl;
        return;
    }

    driver_bot_cpp::distanceVelocity msg;
    msg.time = m_time;
    msg.timeVelocityPlot = m_timeVelocityPlot;
    msg.type = m_type;
    msg.distance = m_distance;
    msg.velocity = m_velocity;

    std::cout << "Check point 2" << std::endl;

    while (ros::ok() && m_pubLidarData.getNumSubscribers() < 1) {
    // wait for a connection to publisher, since we only publish the message once
        std::cout << "WAITING FOR CONNECTION TO PUBLISHER" << std::endl;
    }

    m_pubLidarData.publish(msg);
    ClearData();
}

void DataTransformationPlot::run()
{
    ros::Rate rate(20);

    while (ros::ok() && !m_stopNode)
    {
        rate.sleep();
        ros::spinOnce();
    }
    PublishLidarData();
    ros::shutdown();
}

void DataTransformationPlot::ClearData()
{
    /*Cleares vectors such that they don't contain data for the next calibration*/
    m_time.clear();
    m_distance.clear();
    m_velocity.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_data_transformation_node");
    DataTransformationPlot dataTransformationPlot;
    dataTransformationPlot.run();
    return 0;
}