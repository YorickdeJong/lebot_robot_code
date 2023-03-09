#include "qr_code_detection_subscriber.h"


QRCodeSubscriber::QRCodeSubscriber()
:m_subQRCode{m_node.subscribe("qrCode", 10, &QRCodeSubscriber::QRCodeCallBack, this)},
m_checkQRCode{false}
{
}

void QRCodeSubscriber::QRCodeCallBack(const driver_bot_cpp::qrCode& msg)
{
    /*Receives data from the 'qrCode' topic
    Args:
        msg: contains qr code message/data and als checkQRCode which is 
        true if the qr code data is set, else false
    */
    m_qrCodeData = msg.qrCode;
    m_checkQRCode = msg.checkQRCode;
}

bool QRCodeSubscriber::Validate()
{
    /* Checks if incomming data is received. If not false is returned,
        else true 
    */
    if (m_checkQRCode)
    {
        std::cout << "QR Code Received IN QR CODE SUBSCRIBER" << std::endl;
        return true;
    }
    std::cout << "QR Code NOT Received IN QR CODE SUBSCRIBER" << std::endl;
    return false;
}

 
void QRCodeSubscriber::Run()
{
    /* Runs script to receive data from 'qrCode' node

    */
    ros::Rate rate(10);
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

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrSubscriber");
    QRCodeSubscriber qrCodeSubscriber;
    qrCodeSubscriber.Run();
}