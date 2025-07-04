#include <string>
#include <vector>
#include <unordered_set>

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

#include "BMS.hpp"
#include "BMSFactory.hpp"

ros::Time last_shutdown_time = ros::Time(0);
std::vector<serial::BMS*> bmses;
ros::NodeHandle* nh = nullptr;

void checkBMSPorts(const ros::TimerEvent&) {
    if (bmses.size() < 2 || !bmses[0]->isOpen() || !bmses[1]->isOpen()) {
        ROS_WARN("One or both ports closed. Rescanning...");
        BMSFactory::scanForBMS(bmses, "/dev", *nh);
    }
}

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (msg->channels.size() < 10) return;
    uint16_t ch10 = msg->channels[9];

    if (ch10 > 1899) {
        ros::Time now = ros::Time::now();
        if ((now - last_shutdown_time).toSec() < 5.0) {
            ROS_INFO("Ignoring command — wait 5 seconds between sends");
            return;
        }
        last_shutdown_time = now;


        for(auto bms : bmses) {
            if(bms->isOpen())
                bms->sendShutdown();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bms_manager_node");
    nh = new ros::NodeHandle();
    BMSFactory::scanForBMS(bmses, "/dev", *nh);

    ros::Subscriber rc_sub = nh->subscribe("/mavros/rc/in", 10, rc_callback);

    ros::Timer bms_check_timer = nh->createTimer(ros::Duration(5.0), checkBMSPorts);


    ros::spin();

    delete nh;
    return 0;
}
