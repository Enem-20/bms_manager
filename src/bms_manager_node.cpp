#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

#include "BMS.hpp"
#include "BMSFactory.hpp"

// Глобальные переменные
ros::Time last_shutdown_time = ros::Time(0);
std::vector<serial::BMS*> bmses;
std::mutex bms_mutex;

// Обработка RC каналов
void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (msg->channels.size() < 10) return;

    uint16_t ch10 = msg->channels[9];
    ros::Time now = ros::Time::now();

    if (ch10 > 1899 && (now - last_shutdown_time).toSec() >= 5.0) {
        last_shutdown_time = now;

        std::lock_guard<std::mutex> lock(bms_mutex);
        for (auto bms : bmses) {
            if (bms && bms->isOpen()) {
                bms->sendShutdown();
                ROS_INFO("Shutdown command sent");
            }
        }
    }
}
ros::NodeHandle* g_nh = nullptr;

void checkBMSPorts(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(bms_mutex);

    if (bmses.size() < 2 || !bmses[0]->isOpen() || !bmses[1]->isOpen()) {
        ROS_WARN("One or both BMS ports closed. Rescanning...");
        if (g_nh) {
            BMSFactory::scanForBMS(bmses, "/dev", *g_nh);
        } else {
            ROS_ERROR("NodeHandle not initialized!");
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bms_manager_node");
    ros::NodeHandle nh;
    g_nh = &nh;

    {
        std::lock_guard<std::mutex> lock(bms_mutex);
        BMSFactory::scanForBMS(bmses, "/dev", nh);
    }

    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);
    ros::Timer bms_check_timer = nh.createTimer(ros::Duration(5.0), checkBMSPorts);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::waitForShutdown();

    {
        std::lock_guard<std::mutex> lock(bms_mutex);
        for (auto bms : bmses) {
            delete bms;
        }
        bmses.clear();
    }

    return 0;
}
