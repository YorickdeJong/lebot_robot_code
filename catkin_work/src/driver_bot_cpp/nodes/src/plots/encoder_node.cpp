#include "encoder_node.h"


#define GPIOLINK "\
#!/bin/bash \n\
sudo chmod 777 /dev/gpiomem  \n\
sudo chmod 777 /dev/mem  \n\
"

// Set the GPIO pins for the encoders
const int ENCODER_A = 20;
const int ENCODER_B = 21;

// Define the global instance of the EncoderNode class
EncoderNode* g_encoderNodeInstance = nullptr;
int EncoderNode::m_encoderCount = 0;

// // Define the current interrupt pin
int EncoderNode::currentInterruptPinA = -1;
int EncoderNode::currentInterruptPinB = -1;

EncoderNode::EncoderNode()
:m_distanceTravelled{0.0},
 m_pubEncoderData{m_node.advertise<driver_bot_cpp::distanceData>("distanceData", 2)},
 m_subKeyStrokes{m_node.subscribe("key_strokes", 10, &EncoderNode::KeyStrokesCallback, this)},
 m_pubStopNode{m_node.advertise<std_msgs::Bool>("stopNode", 2)}, 
 wheelRadiusCM{3.25}
{
    system(GPIOLINK);
    g_encoderNodeInstance = this;
    std::cout << "ENCODER NODE CONSTRUCTED" << std::endl;
}



void EncoderNode::handleEncoderInterrupt(int pin) {
    // read the current state of the encoder channels
    int pinA = digitalRead(ENCODER_A);
    int pinB = digitalRead(ENCODER_B);

    // update the encoder count based on the channel states


    if (pin == ENCODER_A) {
        if (pinA == pinB) {
            m_encoderCount--;
        } 
        else {
            m_encoderCount++;
        }
    } else if (pin == ENCODER_B) {
        if (pinA != pinB) {
            m_encoderCount--;
        } 
        else {
            m_encoderCount++;
        }
    }
}

void EncoderNode::DistanceTravelled(){
    m_distanceTravelled = (static_cast<float>(m_encoderCount) / (32.0 * 120.0)) * (2 * 3.1416 * wheelRadiusCM);
} 

void EncoderNode::KeyStrokesCallback(const std_msgs::String::ConstPtr &msg) 
{
    if (msg->data.length() == 0) {
        return;
    }
    m_key = msg->data;
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
    driver_bot_cpp::distanceData msg;
    msg.distance = m_distanceTravelled;
    msg.type = "encoder";
    m_pubEncoderData.publish(msg);
}


void EncoderNode::handleEncoderInterruptWrapperA() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinA);
    }
}

void EncoderNode::handleEncoderInterruptWrapperB() {
    if (g_encoderNodeInstance) {
        g_encoderNodeInstance->handleEncoderInterrupt(currentInterruptPinB);
    }
}
void EncoderNode::run(){
    ros::Rate rate (2);

    if (wiringPiSetupGpio() < 0) {
        std::cerr << "Failed to initialize wiringPi." << std::endl;
    }

    // set up the encoder pins as inputs and enable pull-up resistors
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    pullUpDnControl(ENCODER_A, PUD_UP);
    pullUpDnControl(ENCODER_B, PUD_UP);

    currentInterruptPinA = ENCODER_A;
    wiringPiISR(ENCODER_A, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperA);
        
    currentInterruptPinB = ENCODER_B;
    wiringPiISR(ENCODER_B, INT_EDGE_BOTH, &EncoderNode::handleEncoderInterruptWrapperB);


    while (ros::ok())
    {
        DistanceTravelled();
        PublishEncoderData();

        std::cout << "distance travelled: " << m_distanceTravelled << std::endl;
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