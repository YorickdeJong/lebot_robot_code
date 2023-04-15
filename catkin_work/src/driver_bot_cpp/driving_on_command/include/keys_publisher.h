#include <ros/ros.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "driver_bot_cpp/keyStrokes.h"
#include <csignal>

class KeyPublisher
{
    public:
        KeyPublisher();
        ~KeyPublisher();
        
        void PublishKeys();

    private:
        //subscriber
        ros::NodeHandle m_node;
        ros::Publisher m_pubKeyStrokes;
        struct termios old_tio_, new_tio_;

};