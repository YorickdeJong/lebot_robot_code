#include "keys_publisher.h"

KeyPublisher::KeyPublisher() 
: m_pubKeyStrokes{m_node.advertise<driver_bot_cpp::keyStrokes>("key_strokes", 10)}    
{
    // tcgetattr(STDIN_FILENO, &old_tio_);   // get terminal attributes
    // new_tio_ = old_tio_;                   // set new terminal attributes to be the same as the old ones
    // new_tio_.c_lflag &= ~(ICANON | ECHO);  // set new attributes to raw mode (no canonical mode, no echoing of characters)
    // tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);  // set the new terminal attributes immediately

    // print message to user
    ROS_INFO("Publishing keystrokes. Press Crtl-c to exit");
}

KeyPublisher::~KeyPublisher() 
{
    // restore terminal settings to their original state
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
}

bool is_space(char ch) {
    return std::isspace(static_cast<unsigned char>(ch));
}

void KeyPublisher::PublishKeys() 
{

    // Save the original terminal settings
    struct termios original_term_attr;
    tcgetattr(STDIN_FILENO, &original_term_attr);

    // Set the new terminal settings
    struct termios new_term_attr = original_term_attr;
    new_term_attr.c_lflag &= ~(ICANON | ECHO);
    new_term_attr.c_cc[VMIN] = 0;
    new_term_attr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term_attr);

    ros::Rate rate(100);
    driver_bot_cpp::keyStrokes msg;
    float velPercentage;
    std::string keyStroke;

    while (ros::ok())
    {
        // Read input if available
        std::string input;
        char c;
        while (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                break;
            }
            input.push_back(c);
        }


        if (input.length() == 0) {
            continue;
        }

        std::istringstream iss(input);
        std::cout << input << std::endl;
        iss >> velPercentage >> keyStroke;

        //Add to seperate c in a keystroke and in a percentage value
        if (keyStroke == "`") {
            ROS_INFO("Exiting KeyPublisher node");
            driver_bot_cpp::keyStrokes msg;
            msg.keyStroke = keyStroke; //keyStroke;
            msg.velPercentage = 0;
            m_pubKeyStrokes.publish(msg);
            ros::shutdown();
            break;
        }
        
        //publish both keystroke and percentage value
        
        msg.velPercentage = velPercentage;
        msg.keyStroke = keyStroke;
        m_pubKeyStrokes.publish(msg);
    
        ros::spinOnce();
        rate.sleep();
    
    }

    // Restore the original terminal settings before exiting
    tcsetattr(STDIN_FILENO, TCSANOW, &original_term_attr);

}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "keys_publisher");
     KeyPublisher keyPublisher;
     keyPublisher.PublishKeys();
     return 0;

}