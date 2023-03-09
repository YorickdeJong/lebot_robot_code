#include "send_powerUp.h"

#define KILLALLNODES "\
#!/bin/bash \n\
echo 'RACE ENDED!' \n\
rosnode kill --all \n\
"

SendPowerUp::SendPowerUp()
: m_checkQRCode{false}, m_ROUND{0},
  m_subQRCode{m_node.subscribe("qrCode", 10, &SendPowerUp::QRCodeCallBack, this)},
  m_subRound{m_node.subscribe("qrRound", 10, &SendPowerUp::RoundCallback, this)},
  m_pubPowerUp{m_node.advertise<std_msgs::Int8>("qrPowerUp", 1)}
{
}

void SendPowerUp::QRCodeCallBack(const driver_bot_cpp::qrCode::ConstPtr& msg)
{
    /*Receives data from the 'qrCode' topic
    Args:
        msg: contains qr code message/data and als checkQRCode which is 
        true if the qr code data is set, else false
    */
    m_qrCodeData = msg -> qrCode;
    m_checkQRCode = msg -> checkQRCode;
}

void SendPowerUp::RoundCallback(const std_msgs::Int8::ConstPtr& msg)
{
    /*Receives data from the 'qrCode' topic
    Args:
        msg: contains qr code message/data and als checkQRCode which is 
        true if the qr code data is set, else false
    */
    m_ROUND = msg -> data;
}


bool SendPowerUp::Validate()
{
    /* Checks if incomming data is received. If not false is returned,
        else true 
    */
    if (m_checkQRCode)
    {
        std::cout << "QR Code Received IN QR POWERUP" << std::endl;
        return true;
    }
    std::cout << "QR Code NOT Received IN QR POWERUP" << std::endl;
    return false;
}

void SendPowerUp::powerUp()
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
            m_ROUND += 1;
        }

        if (qrText == "powerUp") //wait 6 seconds untill able to publish -> drains callback
        {
            PublishPowerUp();
        }       

        if (m_ROUND == 3)
        {
            system(KILLALLNODES);
        }
        rate.sleep();
        ros::spinOnce(); 
    }

}

void SendPowerUp::PublishPowerUp()
{        
    while (ros::ok() && m_pubPowerUp.getNumSubscribers() < 1) {
    // wait for a connection to publisher, since we only publish the message once
        std::cout << "WAITING FOR CONNECTION TO PUBLISHER" << std::endl;
    }

    m_powerUp = powerUpGenerator();
    std_msgs::Int8 msg;
    msg.data = m_powerUp;
    m_pubPowerUp.publish(msg);
    std::cout << "Publish POWERUP MESSAGE: " << m_powerUp << std::endl;  
    Wait();
    Empty();

}

int SendPowerUp::powerUpGenerator()
{
    int max = 3; //max number of power ups
    return floor(random() * max);
}

void SendPowerUp::Wait()
{
    ros::Duration duration(6); 
    ros::Time time = ros::Time::now() + duration;
    while (time > ros::Time::now()) //delay untill next scan can happen
    {
        ros::spinOnce();
        continue;
    }
}

void SendPowerUp::Empty()
{
    m_qrCodeData.clear();
    m_checkQRCode = false; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrPowerPublisher");
    SendPowerUp sendPowerup;
    sendPowerup.powerUp();
    return 0;
}