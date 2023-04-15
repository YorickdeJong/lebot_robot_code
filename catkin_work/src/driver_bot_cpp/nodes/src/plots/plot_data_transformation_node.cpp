#include "plot_data_transformation_node.h"

DataTransformationPlot::DataTransformationPlot()
: m_subDistance{m_node.subscribe("distanceData", 10, &DataTransformationPlot::DistanceCallback, this)},
m_subStopNode{m_node.subscribe("stopNode", 10, &DataTransformationPlot::StopNodeCallback, this)},
m_pubData{m_node.advertise<driver_bot_cpp::distanceVelocity>("deviceData", 10)},
m_beginTime{ros::Time::now().toSec()}, 
m_stopNode{false}, m_setMotors{true},
m_operationsDistance{ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT, ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT},
m_adjustTimeToStartAtZero{0}, m_rate{7}, 
m_velocity{4, std::vector<float>(1, -10000.0)}, m_force{4, std::vector<float>(1, -10000.0)}, 
m_energy{4, std::vector<float>(1, -10000.0)}, m_distance{4, std::vector<float>(1, -10000.0)}
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
    // //check if connnection_to_app_node is subscribed to the topic
    
    if (m_pubData.getNumSubscribers() < 1) {
    // wait for a connection to publisher, since we only publish the message once
        std::cout << "CONNECTION TO APP NODE NOT YET ESTABLISHED " << std::endl;
        return;
    }
    
    
    double currentTime = ros::Time::now().toSec(); 

    //TODO change data for lidar to fit the distanceData message

    if (m_type.empty()) {  // Check if m_type is already set
        m_type = msg->type;
    }


    if (msg -> distance[0] != -10000.0){
        if (m_distance[0][0] == -10000.0){
            m_distance[0][0] = msg -> distance[0]; 
        }
        else{
            m_distance[0].emplace_back(msg -> distance[0]); //change to distanceM2
        }
    }

    if (msg -> distance[1] != -10000.0){
        if (m_distance[1][0] == -10000.0){
            m_distance[1][0] = msg -> distance[1]; 
        }
        else{
            m_distance[1].emplace_back(msg -> distance[1]); //change to distanceM2
        }
    }

    if (msg -> distance[2] != -10000.0){
        std::cout << "check" << std::endl;
        if (m_distance[2][0] == -10000.0){
            m_distance[2][0] = msg -> distance[2]; 
        }
        else{
            m_distance[2].emplace_back(msg -> distance[2]); //change to distanceM2
        }
    }

    if (msg -> distance[3] != -10000.0){
        if (m_distance[3][0] == -10000.0){
            m_distance[3][0] = msg -> distance[3]; 
        }
        else{
            m_distance[3].emplace_back(msg -> distance[3]); //change to distanceM2
        }
    }

    if (m_adjustTimeToStartAtZero == 0.0) {
        m_adjustTimeToStartAtZero = currentTime - m_beginTime;
    }

    // remove elements untill the 
    m_time.emplace_back(currentTime - m_beginTime - m_adjustTimeToStartAtZero);
  
    //Publish the data immidiately once the message is received
    PublishData();
}   

// With a starting value of  0 -> 0 time steps behind
void DataTransformationPlot::VelocityCalculation()
{
    if (m_time.size() < 3) {
        std::cout << "NOT ENOUGH ELEMENTS IN m_time TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    float new_velocity;
    int column;
    //looping over rows
    for (size_t row = 0; row < m_velocity.size(); ++row) {
        std:: cout << "m_distance[row].size() " << m_distance[row].size() << std::endl;

        // not enough elements
        if (m_distance[row].size() <= 2){
            std::cout << "NOT ENOUGH ELEMENTS IN m_distance TO CALCULATE VELOCITY IN TRANSFORMATION_NODE" << std::endl;
            continue;
        }

        // check if the same size
        if (m_distance[row].size() != m_time.size()){
            std::cout << "m_distance and m_time are not the same size" << std::endl;
            continue;
        }

        //next element calculation
        column = m_distance[row].size() - 2; //minus 2 since size starts at 1 and we are looking at the next element with column + 1
        float distance_diff = m_distance[row][column + 1] - m_distance[row][column];
        double time_diff = m_time[column + 1] - m_time[column];
      


        // If the sum of distances or times is 0, the velocity is 0
        if (distance_diff == 0.0 || time_diff == 0.0) {
            new_velocity = 0.0;
        }
        else{
            new_velocity = distance_diff / time_diff;
        }

        // Replace the -10000 value if present
        if (m_velocity[row][0] == -10000.0) {
            m_velocity[row][0] = 0.0;
            m_velocity[row].emplace_back(new_velocity);
        } 
        else {
            m_velocity[row].emplace_back(new_velocity);
        }
    }


    for (int j {0}; j < m_velocity.size(); j++){
        for (int i{0}; i < m_velocity[j].size(); ++i) {
            std::cout << "m_velocity " << j << " " << i << " " << m_velocity[j][i] << std::endl;
        }
    }
}

// With a starting value of  0 -> 1 time steps behind
void DataTransformationPlot::ForceCalculation()
{
    if (m_time.size() < 3) {
        std::cout << "NOT ENOUGH ELEMENTS IN m_time TO CALCULATE FORCE IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    const float mass = 0.5;  // mass of the car in kg
    float new_force;

    for (size_t row = 0; row < m_force.size(); ++row) {

        if (m_distance[row].size() <= 2){
            std::cout << "NOT ENOUGH ELEMENTS IN m_distance TO CALCULATE FORCE IN TRANSFORMATION_NODE" << std::endl;
            continue;
        }

        if (m_distance[row].size() != m_time.size()){
            std::cout << "m_distance and m_time are not the same size" << std::endl;
            continue;
        }

        int column = m_distance[row].size() - 2;
        float distance_diff1 = m_distance[row][column] - m_distance[row][column - 1];
        float distance_diff2 = m_distance[row][column + 1] - m_distance[row][column];
        double time_diff1 = m_time[column] - m_time[column - 1];
        double time_diff2 = m_time[column + 1] - m_time[column];
        float acceleration = (distance_diff2 - distance_diff1) / (time_diff1 + time_diff2);
        new_force = mass * acceleration;

        // Replace the -10000 value if present or add initial force as zero
        if (m_force[row][0] == -10000.0) {
            m_force[row][0] = 0.0;
            m_force[row].emplace_back(new_force);
        }
        else {
            m_force[row].emplace_back(new_force);
        }
    }

    for (int j {0}; j < m_force.size(); j++){
        for (int i{0}; i < m_force[j].size(); ++i) {
            std::cout << "m_force " << j << " " << i << " " << m_force[j][i] << std::endl;
        }
    }
}

// With a starting value of  0 -> 2 time steps behind
void DataTransformationPlot::EnergyCalculation()
{
    if (m_time.size() < 3) {
        std::cout << "NOT ENOUGH ELEMENTS IN m_time TO CALCULATE ENERGY IN TRANSFORMATION_NODE" << std::endl;
        return;
    }

    float new_energy;
    for (size_t row = 0; row < m_energy.size(); ++row) {

        if (m_force[row].size() <= 2){
            std::cout << "NOT ENOUGH ELEMENTS IN m_force TO CALCULATE ENERGY IN TRANSFORMATION_NODE" << std::endl;
            continue;
        }

        if (m_force[row].size() != m_distance[row].size() - 1){
            std::cout << "m_force and m_time are not the same size" << std::endl;
            std::cout << "m_force[row].size() " << m_force[row].size() << std::endl;
            std::cout << "m_distance[row].size() " << m_distance[row].size() << std::endl;
            continue;
        }

        int column = m_force[row].size() - 2;
        float force_avg = (m_force[row][column] + m_force[row][column + 1]) / 2;
        float distance_diff = m_distance[row][column + 1] - m_distance[row][column - 1]; // extra + 1 here to be conform with the force 
        new_energy = force_avg * distance_diff;

        // Replace the -10000 value if present
        if (m_energy[row][0] == -10000.0) {
            m_energy[row][0] = 0.0; //initial energy -> object starts at rest
            m_energy[row].emplace_back(new_energy);
        } 
        else {
            m_energy[row].emplace_back(new_energy);
        }
    }

    for (int j {0}; j < m_energy.size(); j++){
        for (int i{0}; i < m_energy[j].size(); ++i) {
            std::cout << "m_energy " << j << " " << i << " " << m_energy[j][i] << std::endl;
        }
    }
    std::cout << "energy size: " << m_energy[1].size() << std::endl;
    std::cout << "m_time size: " << m_time.size() << std::endl;
}

std::vector<std_msgs::Float32MultiArray> DataTransformationPlot::filterAndConvertVector(std::vector<std::vector<float>>& data) {
    std::vector<std_msgs::Float32MultiArray> output;

    int count = 0;

    for (const auto& row : data) {
        bool has_real_data = true;
        for (float value : row) {
            if (value == -10000.0) {
                has_real_data = false;
                break;
            }
        }
        
        // increase motor number 
        count++;

        if (has_real_data) { // TODO: should not be necassary if we define the vectors as Float32MultiArray right away
            if (m_setMotors){
                m_motorNumber.emplace_back(count); // append motor number to vector if it has real data
            }
            std_msgs::Float32MultiArray row_msg;
            for (const float& element : row) {
                row_msg.data.emplace_back(element);
            }
            output.emplace_back(row_msg);
        }
    }

    m_setMotors = false;
    
    return output;
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
    
    if (m_time.size() == 0) {
        std::cout << "NO DATA TO PUBLISH IN TRANSFORMATION NODE" << std::endl;
        return;
    }
    
    driver_bot_cpp::distanceVelocity msg;
    msg.time = m_time;
    msg.distance = filterAndConvertVector(m_distance);
    msg.velocity = filterAndConvertVector(m_velocity);
    msg.force = filterAndConvertVector(m_force);
    msg.energy = filterAndConvertVector(m_energy);

    msg.motorNumber = m_motorNumber;
    msg.type = m_type;
    m_pubData.publish(msg);
    std::cout << "PUBLISHED DATA IN TRANSFORMATION NODE" << std::endl;

    
}


void DataTransformationPlot::run()
{
    ros::Rate rate(m_rate);

    while (ros::ok() && !m_stopNode)
    {
        rate.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
}

void DataTransformationPlot::ClearData()
{
    /*Cleares vectors such that they don't contain data for the next calibration*/
    m_time.clear();
    m_distance.clear();
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
