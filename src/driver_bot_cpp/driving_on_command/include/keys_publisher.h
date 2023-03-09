#include <ros/ros.h>
#include "std_msgs/String.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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