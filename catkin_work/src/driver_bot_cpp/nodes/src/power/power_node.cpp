#include "power_node.h"


PowerMonitor::PowerMonitor(ros::NodeHandle &nh) {
    // m_voltagePub = nh.advertise<std_msgs::Float32>("voltage", 10);
    m_currentPub = nh.advertise<std_msgs::Float32>("current", 10);
}

bool PowerMonitor::init() {
    
    if (!m_ina219.init()) {
        std::cout << "failed to init ina219" << std::endl;
        return false;
    }
    m_ina219.setCalibration_16V_400mA();


    return true;
}

void PowerMonitor::start() {
    ros::Rate loop_rate(2); // 1 Hz

    while (ros::ok()) {
        std::cout << "current: " << m_ina219.getCurrent_mA() << " mA" << std::endl;
        std::cout << "power: " << m_ina219.getPower_mW() << " mW" << std::endl;
        publishCurrent();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PowerMonitor::publishCurrent() {
    std_msgs::Float32 current_msg;
    current_msg.data = m_ina219.getCurrent_mA();
    m_currentPub.publish(current_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "power_monitor");
    ros::NodeHandle nh;

    PowerMonitor power_monitor(nh);
    if (!power_monitor.init()) {
        return 1;
    }

    power_monitor.start();
    return 0;
}


