#include "ADS1115.h"
#include "INA219.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

class PowerMonitor {
    public:
        PowerMonitor(ros::NodeHandle &nh);
        bool init();
        void start();

    private:
        void publishCurrent();

        INA219 m_ina219{0x40};
        ros::Publisher m_currentPub;
};
