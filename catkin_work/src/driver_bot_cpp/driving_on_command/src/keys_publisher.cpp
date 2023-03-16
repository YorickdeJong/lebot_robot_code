#include "keys_publisher.h"

KeyPublisher::KeyPublisher() 
: m_pubKeyStrokes{m_node.advertise<std_msgs::String>("key_strokes", 10)}    
{
    tcgetattr(STDIN_FILENO, &old_tio_);   // get terminal attributes
    new_tio_ = old_tio_;                   // set new terminal attributes to be the same as the old ones
    new_tio_.c_lflag &= ~(ICANON | ECHO);  // set new attributes to raw mode (no canonical mode, no echoing of characters)
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);  // set the new terminal attributes immediately

    // print message to user
    ROS_INFO("Publishing keystrokes. Press Crtl-c to exit");
}

KeyPublisher::~KeyPublisher() 
{
    // restore terminal settings to their original state
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
}


void KeyPublisher::PublishKeys() 
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        // check if there is any input waiting on stdin (the standard input device, i.e., keyboard)
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);  // set file descriptor to standard input device
        timeval timeout{0, 0};

        if (select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout) != -1) 
        {
            if (FD_ISSET(STDIN_FILENO, &read_fds)) 
            {
                char c;
                if (read(STDIN_FILENO, &c, 1) == -1) 
                {
                    break;
                }

                if (c == '`') {
                    ROS_INFO("Exiting KeyPublisher node");
                    std_msgs::String msg;
                    msg.data = std::string(1, c);
                    m_pubKeyStrokes.publish(msg);
                    ros::shutdown();
                    break;
                }
                
                std_msgs::String msg;
                msg.data = std::string(1, c);
                m_pubKeyStrokes.publish(msg);
                std::cout << "published: " << msg << std::endl;
            }
        }

        ros::spinOnce();
        rate.sleep();
    
    }
}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "keys_publisher");
     KeyPublisher keyPublisher;
     keyPublisher.PublishKeys();
     return 0;

}