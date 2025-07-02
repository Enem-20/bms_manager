#include "BMSFactory.hpp"

#include <filesystem>
#include <regex>
#include <iostream>

#include <ros/console.h>

#include "BMS.hpp"

void BMSFactory::closeBMSes(std::vector<serial::BMS*>& bmses) {
    for(serial::BMS* bms : bmses) {
        delete bms;
    }
    bmses.clear();
}

std::vector<serial::BMS*> BMSFactory::scanForBMS(std::vector<serial::BMS*>& bmses, const std::string& path, ros::NodeHandle& nh) {
    std::regex tty_regex(R"(ttyUSB\d+)");
    bool shouldClear = false;
    for(auto bms : bmses) {
        if(bms->isOpen()) {
            bms->checkAnswerable();
            if(!bms->isAnswerable()) {
                shouldClear = true;
            }
                
        }
        else {
            shouldClear = true;
        }
    }
    if(shouldClear) {
        for(auto bms : bmses) {
            delete bms;
        }
        bmses.clear();
    }

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        //ROS_INFO("Entry %s bytes", entry.path().c_str());
        if (!entry.is_character_file()) continue;
        //ROS_INFO("After: if (!entry.is_character_file()) continue;");
        const std::string filename = entry.path().filename().string();
        ROS_INFO("filename: %s", filename.c_str());
        if (!std::regex_match(filename, tty_regex)) continue;
        ROS_INFO("After: if (!std::regex_match(filename, tty_regex)) continue;");
        const std::string devicePath = entry.path().string();
        ROS_INFO("devicePath = %s", devicePath.c_str());
        try {
            auto* bms = new serial::BMS(5, &nh,
                devicePath,
                9600,
                serial::Timeout::simpleTimeout(1000),
                serial::eightbits,
                serial::parity_none,
                serial::stopbits_one,
                serial::flowcontrol_none
            );

            if (bms->isAccessed() && bms->isAnswerable()) {
                ROS_INFO("[BMS] OK: %s", devicePath.c_str());
                bmses.push_back(bms);
            } else {
                ROS_INFO("[BMS] Skipped: %s", devicePath.c_str());
                delete bms;
            }

        } catch (const std::exception& e) {
            ROS_ERROR("[BMS] Error opening: %s: %s", devicePath.c_str(), e.what());
        }
    }

    return bmses;
}