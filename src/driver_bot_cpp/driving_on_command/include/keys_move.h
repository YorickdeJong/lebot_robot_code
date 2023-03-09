#pragma once
#include <map>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "../../actions/include/actionsFactory.h"
#include "../../motorImplementation/include/MotorDetails.h"


class KeysMove{
    public:
        KeysMove();
        ~KeysMove() = default;

        void KeyStrokesCallback(const std_msgs::String::ConstPtr &msg);
        void VelocityCalc();

    private:
        //subscriber
        ros::NodeHandle m_node;
        ros::Subscriber m_subKeyStrokes;

        //define input maxSpeed and maxAcceleration
        double m_velMax;
        double m_velRamp;
        
        // Define the key mapping
        std::map<char, std::unique_ptr<ActionsInterface>> m_keyMapping;

        //key
        std::string m_key;

        //actions 
        std::unique_ptr<ActionsInterface> m_actions;
};