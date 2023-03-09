#include "send_round.h"

#define KILLALLNODES "\
#!/bin/bash \n\
echo 'RACE ENDED!' \n\
rosnode kill --all \n\
"

CPPTOJS::CPPTOJS()
: m_checkQRCode{false}, m_round{}, RACE_ROUNDS{3},
    m_subQRCode{m_node.subscribe("qrCode", 10, &CPPTOJS::QRCodeCallBack, this)},
    m_pubRound{m_node.advertise<std_msgs::Int32>("qrRound", 1)}
{
}

void CPPTOJS::QRCodeCallBack(const driver_bot_cpp::qrCode::ConstPtr& msg)
{
    /*Receives data from the 'qrCode' topic
    Args:
        msg: contains qr code message/data and als checkQRCode which is 
        true if the qr code data is set, else false
    */
    m_qrCodeData = msg -> qrCode;
    m_checkQRCode = msg -> checkQRCode;
}

bool CPPTOJS::Validate()
{
    /* Checks if incomming data is received. If not false is returned,
        else true 
    */
    if (m_checkQRCode)
    {
        std::cout << "QR Code Received IN QR ROUND" << std::endl;
        return true;
    }
    std::cout << "QR Code NOT Received IN QR ROUND" << std::endl;
    return false;
}

void CPPTOJS::Round()
{
    ros::Rate rate(3);
    while (ros::ok())
    {

        if (!Validate())
        {
            ros::spinOnce();
            rate.sleep(); 
            continue;
        }
        
        for (int i{}; i < m_qrCodeData.size(); i++)
        {
            std::cout <<  m_qrCodeData[i];    
        }
        
        std::string qrText(m_qrCodeData.begin(), m_qrCodeData.end());

        if (qrText == "round") //wait 6 seconds untill able to publish -> drains callback
        {
            m_round += 1;
            break;
        }

        rate.sleep();
        ros::spinOnce(); 
    }

}

void CPPTOJS::Run()
{

    ros::Duration duration(6); 
    ros::Time time = ros::Time::now();

    for (int i{}; i < RACE_ROUNDS; i++) //exit if number of rounds is completed
    {
        Round();
        PublishRound(m_round);
        time = ros::Time::now() + duration;
        while (time > ros::Time::now()) //delay untill next scan can happen
        {
            ros::spinOnce();
            continue;
        }
        Empty();
    }
    system(KILLALLNODES);

}

void CPPTOJS::PublishRound(int round)
{        
    while (ros::ok() && m_pubRound.getNumSubscribers() < 1) {
    // wait for a connection to publisher, since we only publish the message once
        std::cout << "WAITING FOR CONNECTION TO PUBLISHER" << std::endl;
    }

    std_msgs::Int32 msg;
    msg.data = round;
    m_pubRound.publish(msg);
    std::cout << "Publish ROUND MESSAGE IN QRNODE: " << round << std::endl;  

}

void CPPTOJS::Wait()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void CPPTOJS::Empty()
{
    m_qrCodeData.clear();
    m_checkQRCode = false; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrRoundPublisher");
    CPPTOJS cppToJS;
    cppToJS.Run();
    return 0;
}