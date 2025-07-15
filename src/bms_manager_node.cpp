#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <future>
#include <signal.h>

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

#include "BMS.hpp"
#include "BMSFactory.hpp"
#include "MavToPublisherSingleton.hpp"

ros::Time last_shutdown_time = ros::Time(0);
std::vector<serial::BMS*> bmses;
std::mutex bms_mutex;
ros::Time lastRcTime = ros::Time(0);
ros::Timer watchdog;

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
   lastRcTime = ros::Time::now();    
if (msg->channels.size() < 10) return;

    uint16_t ch10 = msg->channels[9];
    ros::Time now = ros::Time::now();
    

    if (ch10 > 1899 && (now - last_shutdown_time).toSec() >= 0.0) {
        last_shutdown_time = now;
        std::vector<std::future<void>> futures;
        {
            std::lock_guard<std::mutex> lock(bms_mutex);
            ROS_ERROR("bms count: %i", bmses.size());
            size_t disconnectedCount = 0;
            for (auto bms : bmses) {
                if (bms && bms->isOpen()) {
                    futures.push_back(std::async(std::launch::async, [bms]() {
                        bms->sendShutdown();
                        ROS_INFO("Shutdown command sent");
                    }));
                    ++disconnectedCount;
                }
                else {
                    ROS_ERROR("bms didn't open: %i", disconnectedCount);
                }
            }
            ROS_ERROR("bms disconnected count: %i", disconnectedCount);
        }
        for (auto& f : futures) {
            f.get();
        }
    }
}
ros::NodeHandle* g_nh = nullptr;

void checkAndShutdownRc(const ros::TimerEvent&) {
    ros::Duration delta = ros::Time::now()- lastRcTime;
    if(delta.toSec() > 5.0) {
        std::exit(1);
    }
}

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
    MavToPublisher::getInstance(&nh);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);
    ros::Timer bms_check_timer = nh.createTimer(ros::Duration(5.0), checkBMSPorts);
    watchdog = nh.createTimer(ros::Duration(0.5), checkAndShutdownRc);

    ros::AsyncSpinner spinner(6);
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
