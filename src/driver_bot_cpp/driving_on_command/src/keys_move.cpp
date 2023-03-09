#include "keys_move.h"


KeysMove::KeysMove()
: m_subKeyStrokes{m_node.subscribe("key_strokes", 10, &KeysMove::KeyStrokesCallback, this)}
{
    m_node.getParam("/keys_move/velMax", m_velMax);
    m_node.getParam("/keys_move/velRamp", m_velRamp);
    std::cout << "PARAM " << m_velMax << std::endl;
    ActionsFactory actionsFactory;
    m_actions = actionsFactory.Create(MOTOR_TYPE);
}

void KeysMove::KeyStrokesCallback(const std_msgs::String::ConstPtr &msg) 
{
    m_key = msg -> data;
}



void KeysMove::VelocityCalc() 
{
    // dv = v_ein - ramp * t
    // v_new = v_old + dv

    ros::Time lastTime = ros::Time::now();
    double v_new, v_prev = 10; //start at 10% motor speed
    ros::Rate rate{100};

    while (ros::ok())
    {
        // set previous velocity to current/new velocity
        v_prev = v_new;

        if (!m_key.empty())
        {
            std::cout << "received key: " << m_key << std::endl;
            if (m_key == "w")
            {
                m_actions -> DriveForward(v_prev);
                std::cout << m_key << " pressed" << std::endl;
            }
            else if (m_key == "a")
            {
                m_actions -> TurnLeft(v_prev);
                std::cout << m_key << " pressed" << std::endl;
            }
            else if (m_key == "s")
            {
                m_actions -> DriveBackward(v_prev);
                std::cout << m_key << " pressed" << std::endl;
            }
            else if (m_key == "d")
            {
                m_actions -> TurnRight(v_prev);
                std::cout << m_key << " pressed" << std::endl;
            }
            else if (m_key == "x")
            {
                m_actions -> Stop();
                std::cout << m_key << " pressed" << std::endl;
            }
            else if (m_key == "`")
            {
                m_actions -> Stop();
                std::cout << m_key << " pressed" << std::endl;
                ros::shutdown();
            }
            else
            {
                std::cout << "please provide the correct key" << std::endl;
            }
        }
        m_key = "";
        //time difference
        ros::Duration dt = ros::Time::now() - lastTime;

        //calculate speed gain for one iteration
        double step = m_velRamp * dt.toSec();

        
        //check if we have overshoot or not
        bool sign;
        
        if (m_velMax > v_new)
        {
            sign = 1.0;
        }  
        else
        {
            sign = -1.0;
        } 


        if (m_velMax - v_new < step)
        {
            v_new = m_velMax;
        }
        else
        {
            v_new = v_prev + sign + step;
        }

        ros::spinOnce();
        rate.sleep();
    }
    m_actions -> Stop();
}

int main(int argc, char **argv)
{

    /*Initialize RPLidar object*/
    ros::init(argc, argv, "keys_move");
    KeysMove keysmove{};
    keysmove.VelocityCalc();
    return 0;
}