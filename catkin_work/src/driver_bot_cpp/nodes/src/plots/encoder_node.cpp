#include "encoder_node.h"


#define GPIOLINK "\
#!/bin/bash \n\
sudo chmod 777 /dev/gpiomem  \n\
sudo chmod 777 /dev/mem  \n\
"

// Set the GPIO pins for the encoders
const int ENCODER_A1 = 20;
const int ENCODER_B1 = 21;
const int ENCODER_A2 = 16;
const int ENCODER_B2 = 19;
const int ENCODER_A3 = 12;
const int ENCODER_B3 = 13;
const int ENCODER_A4 = 5;
const int ENCODER_B4 = 6;


// Define the global instance of the EncoderNode class
EncoderNode* g_encoderNodeInstance = nullptr;
int EncoderNode::m_encoderCountMotor1 = 0;
int EncoderNode::m_encoderCountMotor2 = 0;
int EncoderNode::m_encoderCountMotor3 = 0;
int EncoderNode::m_encoderCountMotor4 = 0;


// // Define the current interrupt pin
int EncoderNode::currentInterruptPinA1 = -1;
int EncoderNode::currentInterruptPinB1 = -1;
int EncoderNode::currentInterruptPinA2 = -1;
int EncoderNode::currentInterruptPinB2 = -1;
int EncoderNode::currentInterruptPinA3 = -1;
int EncoderNode::currentInterruptPinB3 = -1;
int EncoderNode::currentInterruptPinA4 = -1;
int EncoderNode::currentInterruptPinB4 = -1;

EncoderNode::EncoderNode()
:m_distanceTravelled_M2{0.0},
 m_pubEncoderData{m_node.advertise<driver_bot_cpp::distanceData>("distanceData", 10)},
 m_subKeyStrokes{m_node.subscribe("key_strokes", 10, &EncoderNode::KeyStrokesCallback, this)},
 m_pubStopNode{m_node.advertise<std_msgs::Bool>("stopNode", 10)}, 
 wheelRadiusCM{3.25}, m_meanDistanceTravelled_M1{10000.0},
 m_meanDistanceTravelled_M2{10000.0}, m_meanDistanceTravelled_M3{10000.0},
 m_meanDistanceTravelled_M4{10000.0}
{
    system(GPIOLINK);
    g_encoderNodeInstance = this;
    std::cout << "ENCODER NODE CONSTRUCTED" << std::endl;
}



void EncoderNode::handleEncoderInterrupt(int pin, int& encoderCount, const int ENCODER_A, const int ENCODER_B) {
    // read the current state of the encoder channels
    int pinA = digitalRead(ENCODER_A);
    int pinB = digitalRead(ENCODER_B);

    // update the encoder count based on the channel states


    if (pin == ENCODER_A) {
        if (pinA == pinB) {
            encoderCount--;
        } 
        else {
            encoderCount++;
        }
    } 
    else if (pin == ENCODER_B) {
        if (pinA != pinB) {
            encoderCount--;
        } 
        else {
            encoderCount++;
        }
    }
}

void EncoderNode::DistanceTravelled(){
    //define for distance motor 1, 2, 3, 4
    //if encodercount remains 0, 

    float distance_M1; 
    if (m_encoderCountMotor1 != 0) {
        distance_M1 = -(static_cast<float>(m_encoderCountMotor1) / (32.0 * 120.0)) * (2 * 3.1416 * wheelRadiusCM) / 100; //calculate distance in meters
        m_distanceTravelled_M1.emplace_back(distance_M1);
    }
    float distance_M2; 
    if (m_encoderCountMotor2 != 0) {
        distance_M2 = -(static_cast<float>(m_encoderCountMotor2) / (32.0 * 120.0)) * (2 * 3.1416 * wheelRadiusCM) / 100; //calculate distance in meters
        m_distanceTravelled_M2.emplace_back(distance_M2);
    }
    float distance_M3; 
    if (m_encoderCountMotor3 != 0) {
        distance_M3 = (static_cast<float>(m_encoderCountMotor3) / (32.0 * 120.0)) * (2 * 3.1416 * wheelRadiusCM) / 100; //calculate distance in meters
        m_distanceTravelled_M3.emplace_back(distance_M3);
    }
    float distance_M4; 
    if (m_encoderCountMotor4 != 0) {
        distance_M4 = (static_cast<float>(m_encoderCountMotor4) / (32.0 * 120.0)) * (2 * 3.1416 * wheelRadiusCM) / 100; //calculate distance in meters
        m_distanceTravelled_M4.emplace_back(distance_M4);
    }
} 

void EncoderNode::MeanDistanceTravelled(){
    //Change to calculate mean distance for all 4 encoders if present
    // check which encoders are present
    if (m_distanceTravelled_M1.size() != 0) {
        m_meanDistanceTravelled_M1 = std::accumulate(m_distanceTravelled_M1.begin(), m_distanceTravelled_M1.end(), 0.0) / m_distanceTravelled_M1.size();
    }
    if (m_distanceTravelled_M2.size() != 0) {
        m_meanDistanceTravelled_M2 = std::accumulate(m_distanceTravelled_M2.begin(), m_distanceTravelled_M2.end(), 0.0) / m_distanceTravelled_M2.size();
    }
    if (m_distanceTravelled_M3.size() != 0) {
        m_meanDistanceTravelled_M3 = std::accumulate(m_distanceTravelled_M3.begin(), m_distanceTravelled_M3.end(), 0.0) / m_distanceTravelled_M3.size();
    }
    if (m_distanceTravelled_M4.size() != 0) {
        m_meanDistanceTravelled_M4 = std::accumulate(m_distanceTravelled_M4.begin(), m_distanceTravelled_M4.end(), 0.0) / m_distanceTravelled_M4.size();
    }
}

void EncoderNode::KeyStrokesCallback(const driver_bot_cpp::keyStrokes::ConstPtr &msg) 
{
    if (msg->keyStroke.length() == 0) {
        return;
    }
    std::cout << "check key strokes" << std::endl;
    m_key = msg->keyStroke;
}

void EncoderNode::PublishStopNode() 
{
    /*Publishes a message to stop the node*/
    std_msgs::Bool msg;
    msg.data = true;
    m_pubStopNode.publish(msg);
    std::cout << "Published stop node message" << std::endl;
}


void EncoderNode::PublishEncoderData(){
    //in the message also indicate which motors are active and which ones are not
    // define a second array with the motor numbers

    if (m_encoderCountMotor1 == 0 && m_encoderCountMotor2 == 0 && m_encoderCountMotor3 == 0 && m_encoderCountMotor4 == 0) {
        std::cout << "NO DATA TO PUBLISH" << std::endl;
        return;
    }
    driver_bot_cpp::distanceData msg;

    msg.distance = {-10000, -10000, -10000, -10000};
    if (m_encoderCountMotor1 != 0) {
        msg.distance[0] = m_meanDistanceTravelled_M1;
        std::cout << "PUBLISHED DISTANCE M1: " << m_meanDistanceTravelled_M1 << std::endl;
        std::cout << "CURREN COUNT M1: " << m_encoderCountMotor1 << std::endl;
    }
    if (m_encoderCountMotor2 != 0){
        msg.distance[1] = m_meanDistanceTravelled_M2;
        std::cout << "PUBLISHED DISTANCE M2: " << m_meanDistanceTravelled_M2 << std::endl;
        std::cout << "CURREN COUNT M2: " << m_encoderCountMotor2  << std::endl;
    }
    if (m_encoderCountMotor3 != 0){
        msg.distance[2] = m_meanDistanceTravelled_M3;
        std::cout << "PUBLISHED DISTANCE M3: " << m_meanDistanceTravelled_M3 << std::endl;
        std::cout << "CURREN COUNT M3: " << m_encoderCountMotor3 << std::endl;

    }
    if (m_encoderCountMotor4 != 0){
        msg.distance[3] = m_meanDistanceTravelled_M4;
        std::cout << "PUBLISHED DISTANCE M4: " << m_meanDistanceTravelled_M4 << std::endl;
        std::cout << "CURREN COUNT M4: " << m_encoderCountMotor4 << std::endl;
    }



    msg.type = "encoder";
    m_pubEncoderData.publish(msg);
}

// Counts amount of interruptsof encoders
void EncoderNode::handleEncoderInterruptWrapperA1() { //define 1, 2, 3, 4
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinA1, m_encoderCountMotor1, ENCODER_A1, ENCODER_B1);
    }
}

void EncoderNode::handleEncoderInterruptWrapperB1() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinB1,  m_encoderCountMotor1, ENCODER_A1, ENCODER_B1);
    }
}

// Counts amount of interruptsof encoders
void EncoderNode::handleEncoderInterruptWrapperA2() { //define 1, 2, 3, 4
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinA2, m_encoderCountMotor2, ENCODER_A2, ENCODER_B2);
    }
}

void EncoderNode::handleEncoderInterruptWrapperB2() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinB2,  m_encoderCountMotor2, ENCODER_A2, ENCODER_B2);
    }
}

// Counts amount of interruptsof encoders
void EncoderNode::handleEncoderInterruptWrapperA3() { //define 1, 2, 3, 4
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinA3, m_encoderCountMotor3, ENCODER_A3, ENCODER_B3);
    }
}

void EncoderNode::handleEncoderInterruptWrapperB3() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinB3,  m_encoderCountMotor3, ENCODER_A3, ENCODER_B3);
    }
}

// Counts amount of interruptsof encoders
void EncoderNode::handleEncoderInterruptWrapperA4() { //define 1, 2, 3, 4
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinA4, m_encoderCountMotor4, ENCODER_A4, ENCODER_B4);
    }
}

void EncoderNode::handleEncoderInterruptWrapperB4() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinB4,  m_encoderCountMotor4, ENCODER_A4, ENCODER_B4);
    }
}

void EncoderNode::run(){
    ros::Rate rate (10);

    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Failed to initialize wiringPi." << std::endl;
    }

    // set up the encoder pins as inputs and enable pull-up resistors

    pinMode(ENCODER_A1, INPUT);
    pinMode(ENCODER_B1, INPUT);
    pullUpDnControl(ENCODER_A1, PUD_UP);
    pullUpDnControl(ENCODER_B1, PUD_UP);


    pinMode(ENCODER_A2, INPUT);
    pinMode(ENCODER_B2, INPUT);
    pullUpDnControl(ENCODER_A2, PUD_UP);
    pullUpDnControl(ENCODER_B2, PUD_UP);

    pinMode(ENCODER_A3, INPUT);
    pinMode(ENCODER_B3, INPUT);
    pullUpDnControl(ENCODER_A3, PUD_UP);
    pullUpDnControl(ENCODER_B3, PUD_UP);

    pinMode(ENCODER_A4, INPUT);
    pinMode(ENCODER_B4, INPUT);
    pullUpDnControl(ENCODER_A4, PUD_UP);
    pullUpDnControl(ENCODER_B4, PUD_UP);

    // Set up the interrupt handler
    currentInterruptPinA1 = ENCODER_A1;
    wiringPiISR(ENCODER_A1, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperA1);

    currentInterruptPinB1 = ENCODER_B1;
    wiringPiISR(ENCODER_B1, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperB1);


    currentInterruptPinA2 = ENCODER_A2;
    wiringPiISR(ENCODER_A2, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperA2);
        
    currentInterruptPinB2 = ENCODER_B2;
    wiringPiISR(ENCODER_B2, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperB2);

    currentInterruptPinA3 = ENCODER_A3;
    wiringPiISR(ENCODER_A3, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperA3);

    currentInterruptPinB3 = ENCODER_B3;
    wiringPiISR(ENCODER_B3, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperB3);


    currentInterruptPinA4 = ENCODER_A4;
    wiringPiISR(ENCODER_A4, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperA4);

    currentInterruptPinB4 = ENCODER_B4;
    wiringPiISR(ENCODER_B4, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperB4);


    int count = 0;
    while (ros::ok())
    {
        count += 1;
        DistanceTravelled();

        if (count % 7 == 0){
            MeanDistanceTravelled();
            PublishEncoderData();
            m_distanceTravelled_M1.clear();
            m_distanceTravelled_M2.clear();
            m_distanceTravelled_M3.clear();
            m_distanceTravelled_M4.clear();
        }

        ros::spinOnce();
        rate.sleep();


        // Check if the ` key is pressed
        if (m_key == "`") {
            ROS_INFO("Stopping Encoder node");
            break; // Exit the while loop and stop the node
        }
    }

    PublishStopNode();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_node");
    EncoderNode encoderNode;
    encoderNode.run();
    return 0;
}