#include "BMSFactory.hpp"
#include <filesystem>
#include <regex>
#include <ros/console.h>
#include "BMS.hpp"

void BMSFactory::closeBMSes(std::vector<serial::BMS*>& bmses) {
    for(serial::BMS* bms : bmses) {
        delete bms;
    }
    bmses.clear();
}

std::vector<serial::BMS*> BMSFactory::scanForBMS(std::vector<serial::BMS*>& bmses, 
                                                const std::string& path, 
                                                ros::NodeHandle& nh) {
    std::regex tty_regex(R"(ttyUSB\d+)");
    std::vector<serial::BMS*> new_bmses;
    std::vector<std::string> current_devices;

    // Шаг 1: Проверка существующих BMS
    for (auto it = bmses.begin(); it != bmses.end();) {
        serial::BMS* bms = *it;
        const std::string dev_path = bms->getPath();
        
        // Проверяем физическое существование устройства
        bool device_exists = std::filesystem::exists(dev_path);
        
        if (!device_exists || !bms->isAnswerable()) {
            ROS_WARN("BMS %s disconnected or not responding. Reconnecting...", dev_path.c_str());
            
            // Попытка переподключения к тому же порту
            bms->reconnect(device_exists ? dev_path : "");
            
            if (!bms->isAnswerable() || !bms->isOpen()) {
                ROS_ERROR("Failed to reconnect BMS %s. Removing.", dev_path.c_str());
                delete bms;
                it = bmses.erase(it);
                continue;
            }
        }
        
        // Успешно переподключенный BMS сохраняется
        new_bmses.push_back(bms);
        current_devices.push_back(dev_path);
        ++it;
    }

    // Шаг 2: Сканирование новых устройств
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (!entry.is_character_file()) continue;
        
        const std::string filename = entry.path().filename().string();
        const std::string devicePath = entry.path().string();
        
        // Пропускаем неподходящие файлы
        if (!std::regex_match(filename, tty_regex)) continue;
        
        // Проверяем, не добавлено ли уже устройство
        bool already_exists = std::find(current_devices.begin(), 
                                      current_devices.end(), 
                                      devicePath) != current_devices.end();
        
        if (already_exists) continue;

        try {
            ROS_INFO("Trying to initialize BMS at %s", devicePath.c_str());
            auto* bms = new serial::BMS(&nh, devicePath, 9600,
                                        serial::Timeout::simpleTimeout(1000),
                                        serial::eightbits,
                                        serial::parity_none,
                                        serial::stopbits_one,
                                        serial::flowcontrol_none);

            if (bms->isAccessed() && bms->isAnswerable()) {
                ROS_INFO("[BMS] Successfully initialized: %s", devicePath.c_str());
                new_bmses.push_back(bms);
                current_devices.push_back(devicePath);
            } else {
                ROS_WARN("[BMS] Device not accessible or not responding: %s", devicePath.c_str());
                delete bms;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("[BMS] Initialization error %s: %s", 
                     devicePath.c_str(), e.what());
        }
    }

    return new_bmses;
}

bool BMSFactory::alreadyHas(const std::vector<serial::BMS*>& bmses, 
                           const std::string& path) {
    for (auto bms : bmses) {
        if (bms->getPath() == path)
            return true;
    }
    return false;
}