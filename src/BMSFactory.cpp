#include "BMSFactory.hpp"
#include <filesystem>
#include <regex>
#include <ros/console.h>
#include "BMS.hpp"

void BMSFactory::closeBMSes(std::vector<serial::BMS*>& bmses) {
    for(serial::BMS* bms : bmses) {
        bms->stopTimers();
        delete bms;
    }
    bmses.clear();
}

std::vector<serial::BMS*> BMSFactory::scanForBMS(std::vector<serial::BMS*>& bmses, 
                                                const std::string& path, 
                                                ros::NodeHandle& nh) {
    std::regex tty_regex(R"(ttyUSB\d+)");
    std::vector<serial::BMS*> active_bmses;
    std::vector<std::string> current_devices;
    std::vector<std::string> processed_devices;

    for (auto it = bmses.begin(); it != bmses.end(); ) {
        serial::BMS* bms = *it;
        const std::string dev_path = bms->getPath();
        processed_devices.push_back(dev_path);
        bool device_exists = std::filesystem::exists(dev_path);

        if (device_exists && bms->isAnswerable()) {
            active_bmses.push_back(bms);
            current_devices.push_back(dev_path);
            ++it;
            continue;
        }

        if (device_exists) {
            ROS_WARN("Reconnecting to existing BMS at %s", dev_path.c_str());
            bms->stopTimers();
            bms->reconnect(dev_path);
            
            if (bms->isAnswerable()) {
                ROS_INFO("Reconnected to BMS at %s", dev_path.c_str());
                active_bmses.push_back(bms);
                current_devices.push_back(dev_path);
                ++it;
            } else {
                ROS_ERROR("Failed to reconnect BMS at %s. Removing.", dev_path.c_str());
                bms->stopTimers();
                delete bms;
                it = bmses.erase(it);
            }
        } else {
            ROS_ERROR("BMS device %s no longer exists. Removing.", dev_path.c_str());
            bms->stopTimers();
            delete bms;
            it = bmses.erase(it);
        }
    }

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (!entry.is_character_file()) 
            continue;

        const std::string filename = entry.path().filename().string();
        if (!std::regex_match(filename, tty_regex)) 
            continue;

        const std::string devicePath = entry.path().string();
        
        if (std::find(processed_devices.begin(), processed_devices.end(), devicePath) != processed_devices.end()) {
            continue;
        }
        
        if (std::find(current_devices.begin(), current_devices.end(), devicePath) != current_devices.end()) {
            continue;
        }

        try {
            ROS_INFO("Trying to initialize new BMS at %s", devicePath.c_str());
            auto* bms = new serial::BMS(&nh, devicePath, 9600,
                                        serial::Timeout::simpleTimeout(1000),
                                        serial::eightbits,
                                        serial::parity_none,
                                        serial::stopbits_one,
                                        serial::flowcontrol_none);

            if (bms->isAccessed() && bms->isAnswerable()) {
                ROS_INFO("Successfully initialized new BMS at %s", devicePath.c_str());
                active_bmses.push_back(bms);
                bmses.push_back(bms);
            } else {
                ROS_WARN("New BMS at %s is not accessible or not responding", devicePath.c_str());
                bms->stopTimers();
                delete bms;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error initializing BMS at %s: %s", devicePath.c_str(), e.what());
        }
    }

    return active_bmses;
}

bool BMSFactory::alreadyHas(const std::vector<serial::BMS*>& bmses, 
                           const std::string& path) {
    for (auto bms : bmses) {
        if (bms->getPath() == path)
            return true;
    }
    return false;
}