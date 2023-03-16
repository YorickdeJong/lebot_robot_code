#include "plot_data_transformation_node.h"

DataTransformationPlot::DataTransformationPlot()
: m_subDistance{m_node.subscribe("distanceData", 2, &DataTransformationPlot::DistanceCallback, this)},
m_subStopNode{m_node.subscribe("stopNode", 2, &DataTransformationPlot::StopNodeCallback, this)},
m_pubData{m_node.advertise<driver_bot_cpp::distanceVelocity>("deviceData", 2)},
m_beginTime{ros::Time::now().toSec()}, 
m_stopNode{false},
m_operationsDistance{ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT, ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT},
m_adjustTimeToStartAtZero{0},
m_velocity{0}, m_force{0}, m_energy{0}, m_timeVelocity{0}, m_timeEnergy{0}, m_distanceForce{0}, m_rate{2}
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


void DataTransformationPlot::DistanceCallback(const driver_bot_cpp::distanceData::ConstPtr& msg) 
{
    /* Callback reads msg data which contains information about the distance of all the objects in a 360 degree 2D view
    * The callback then calculates the minimum distance of the objects in a specific range of angles and publishes the data
    */
    double currentTime = ros::Time::now().toSec(); 

    //TODO change data for lidar to fit the distanceData message

    if (m_type.empty()) {  // Check if m_type is already set
        m_type = msg->type;
    }

    if (m_adjustTimeToStartAtZero == 0.0) {
        m_adjustTimeToStartAtZero = currentTime - m_beginTime;
    }

    m_time.emplace_back(currentTime - m_beginTime - m_adjustTimeToStartAtZero);
    m_distance.emplace_back(msg -> distance);

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

    if (m_distance.size() <= 2) {
        // there are not enough elements in m_distance to calculate velocity
        std::cout << "NOT ENOUGH ELEMENTS IN m_distance TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
        return;
    } 
    else if (m_distance.size() - 1 > m_velocity.max_size()) {
        std::cout << "TOO MANY ELEMENTS TO RESIZE m_velocity IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    
    // Set the interval size to 5 data points
    const int interval_size = 1;

    float sum_distance = 0.0;
    double sum_time = 0.0;
    for (size_t i = m_distance.size() - 1; i < m_distance.size(); i++) {
        // Calculate the sum of distances and times in the current interval
        for (size_t j = 0; j < interval_size; j++) {
            sum_distance += m_distance[i + j] - m_distance[i + j - 1];
            sum_time += m_time[i + j] - m_time[i + j -1];
        }

        std::cout << "sum distance: " << sum_distance << std::endl;
        std::cout << "sum time: " << sum_time << std::endl;

        if (sum_distance == 0.0 || sum_time == 0.0) {
            // If the sum of distances or times is 0, the velocity is 0
            m_velocity.emplace_back(0.0);
            m_timeVelocity.emplace_back(m_timeEnergy.back()  + 1.0 / m_rate);
            continue;
        }

        // Calculate the average velocity over the current interval and add it to the velocity array
        m_velocity.emplace_back(fabs(sum_distance / sum_time));
        m_timeVelocity.emplace_back(m_timeEnergy.back()  + 1.0 / m_rate);

        std::cout << "velocity size: " << m_velocity.size() << std::endl;
        // Reset the sum of distances and times for the next interval
        sum_distance = 0.0;
        sum_time = 0.0;
    }


    // Calculate the time intervals for the velocity plot
    
    for (size_t i = 0; i < m_timeVelocity.size(); i++) {
        std::cout << "time velocity: " << m_timeVelocity[i] << std::endl;
    }
}

void DataTransformationPlot::ForceCalculation()
{
    /* Function calculates velocity based on time and distance
    * Args:
    *   time: array containing time values
    *   distance: array containing distance values
    * Returns:
    *   velocity: array containing velocity values
    */

    if (m_velocity.size() <= 2) {
        // there are not enough elements in m_distance to calculate velocity
        std::cout << "NOT ENOUGH ELEMENTS IN m_velocity TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
        return;
    } 
    else if (m_velocity.size() - 1 > m_force.max_size()) {
        std::cout << "TOO MANY ELEMENTS TO RESIZE m_force IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    // Set the interval size to 5 data points
    const int interval_size = 1;
    const float mass = 0.5;  // mass of the car in kg

    float sum_velocity = 0.0;
    double sum_time = 0.0;
    for (size_t i = m_velocity.size() - 1; i < m_velocity.size(); i++) {
        // Calculate the sum of distances and times in the current interval
        for (size_t j = 0; j < interval_size; j++) {
            sum_velocity += m_velocity[i + j] - m_velocity[i + j - 1];
            sum_time += m_timeVelocity[i + j] - m_timeVelocity[i + j - 1];
        }

        // Calculate the average velocity over the current interval and add it to the velocity array

        if (sum_time == 0.0 || sum_velocity == 0.0){
            m_force.emplace_back(0.0);
            m_distanceForce.emplace_back(m_distanceForce.back() + 0);
            return;
        }
        m_force.emplace_back(sum_velocity / sum_time * mass);
        m_distanceForce.emplace_back(m_distanceForce.back() + sum_velocity * sum_time);

        // Reset the sum of distances and times for the next interval
        sum_velocity = 0.0;
        sum_time = 0.0;
    }

    // Calculate the distance intervals for the force plot

}



void DataTransformationPlot::EnergyCalculation()
{
    /* Function calculates velocity based on time and distance
    * Args:
    *   time: array containing time values
    *   distance: array containing distance values
    * Returns:
    *   velocity: array containing velocity values
    */

    if (m_force.size() <= 2) {
        // there are not enough elements in m_distance to calculate velocity
        std::cout << "NOT ENOUGH ELEMENTS IN m_force TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
        return;
    } 
    else if (m_force.size() - 1 > m_energy.max_size()) {
        std::cout << "TOO MANY ELEMENTS TO RESIZE m_energy IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    // Set the interval size to 5 data points
    const int interval_size = 1;

    float sum_force = 0.0;
    float sum_distance = 0.0;
    for (size_t i = m_force.size() - 1; i < m_force.size(); i++) {
        // Calculate the sum of distances and times in the current interval
        for (size_t j = 0; j < interval_size; j++) {
            sum_force += m_force[i + j] - m_force[i + j - 1];
            sum_distance += m_distanceForce[i + j] - m_distanceForce[i + j - 1];
        }

        if (sum_force == 0.0 || sum_distance == 0.0){
            m_energy.emplace_back(0.0);
            m_timeEnergy.emplace_back(m_timeEnergy.back()  + 1.0 / m_rate);
            return;
        }

        // Calculate the average velocity over the current interval and add it to the velocity array
        m_energy.emplace_back(sum_force * sum_distance);
        m_timeEnergy.emplace_back(m_timeEnergy.back()  + 1.0 / m_rate);

        // Reset the sum of distances and times for the next interval
        sum_force = 0.0;
        sum_distance = 0.0;
    }

    // Calculate the time intervals for the velocity plot

}

void DataTransformationPlot::PublishData()
{
    /* Function publishes velocity data
    * Args:
    *   time: array containing time values
    *   distance: array containing distance values
    *   velocity: array containing velocity values
    */
    //Calculates the velocity before publishing
    VelocityCalculation();
    ForceCalculation();
    EnergyCalculation();
    
    if (m_time.size() == 0 || m_distance.size() == 0 || m_velocity.size() == 0) {
        std::cout << "NO DATA TO PUBLISH IN TRANSFORMATION NODE" << std::endl;
        return;
    }

    

    driver_bot_cpp::distanceVelocity msg;
    msg.time = m_time;
    msg.timeVelocity = m_timeVelocity;
    msg.timeEnergy = m_timeEnergy;
    msg.distance = m_distance;
    msg.distanceForce = m_distanceForce;
    msg.velocity = m_velocity;
    msg.force = m_force;
    msg.energy = m_energy;
    msg.type = m_type;


    m_pubData.publish(msg);
    std::cout << "PUBLISHED DATA IN TRANSFORMATION NODE" << std::endl;
    // ClearData();
}


void DataTransformationPlot::run()
{
    ros::Rate rate(m_rate);

    while (ros::ok() && !m_stopNode)
    {
        PublishData();

        rate.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
}

void DataTransformationPlot::ClearData()
{
    /*Cleares vectors such that they don't contain data for the next calibration*/
    m_time.clear();
    m_timeVelocity.clear();
    m_timeEnergy.clear();
    m_distance.clear();
    m_distanceForce.clear();
    m_velocity.clear();
    m_force.clear();
    m_energy.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_data_transformation_node");
    DataTransformationPlot dataTransformationPlot;
    dataTransformationPlot.run();
    return 0;
}