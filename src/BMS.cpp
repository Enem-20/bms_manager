#include "BMS.hpp"

#include <cstdint>
#include <algorithm>

#include <numeric>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>


#include <sensor_msgs/BatteryState.h>

namespace serial {

void printHexROS(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (uint8_t byte : data) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(byte) << " ";
    }
    ROS_INFO_STREAM("Data in HEX: " << oss.str());
}

BMS::BMS(size_t id, ros::NodeHandle* nodeHandle, const std::string &port,
          uint32_t baudrate,
          Timeout timeout,
          bytesize_t bytesize,
          parity_t parity,
          stopbits_t stopbits,
          flowcontrol_t flowcontrol) 
    : Serial(port, baudrate, timeout, bytesize, parity, stopbits, flowcontrol)
    , _id(id)
    , _nodeHandle(nodeHandle)
    , _publisher(nodeHandle->advertise<sensor_msgs::BatteryState>("/mavros/battery", 10))
{
    ROS_INFO("Before: if (access(port.c_str(), R_OK | W_OK) != 0) {");
    if (access(port.c_str(), R_OK | W_OK) != 0) {
        ROS_ERROR_STREAM("Cannot access port " << port << " — permission denied.");
        _accessed = false;
    }
    ROS_INFO("After: if (access(port.c_str(), R_OK | W_OK) != 0) {");
    if (!isOpen()) {
        ROS_ERROR_STREAM("Port " << port << " did not open (no exception thrown)");
        return;
    }
    ROS_INFO("After: if (!isOpen()) {");
    checkAnswerable();
    ROS_INFO("After: checkAnswerable();");
    updateCallback({});
    _publishTimer = _nodeHandle->createTimer(ros::Duration(0.3), &BMS::publishCallback, this);
    _updateTimer = _nodeHandle->createTimer(ros::Duration(2.0), &BMS::updateCallback, this);
}

BMS::~BMS() {
    close();
}

float ntcToCelsiusFloat(uint16_t raw) {
    return (static_cast<int32_t>(raw) - 2731) * 0.1f;
}

float averageTemperatureFloat(const std::vector<uint16_t>& ntc_values) {
    if (ntc_values.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    
    float sum = 0.0f;
    for (uint16_t value : ntc_values) {
        sum += ntcToCelsiusFloat(value);
    }
    return sum / ntc_values.size();
}

void BMS::sendBatterries() {
    sensor_msgs::BatteryState bms = {};
    
    bms.temperature = averageTemperatureFloat(_ntcs);
    bms.current = _battInfo->current;
    bms.percentage = _battInfo->RSOC;
    bms.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    bms.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    bms.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    bms.cell_voltage.resize(_voltages.size());
    std::memcpy(bms.cell_voltage.data(), _voltages.data(), _voltages.size()*sizeof(float));
    ROS_INFO("publlishing...");
    _publisher.publish(bms);
}

size_t BMS::sendShutdown()  {
    if (!isOpen()) return 0;

    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    return write(shutdown_cmd, sizeof(shutdown_cmd));
}

BMSBatteriesInfo* BMS::getBMSBatteriesInfo() {
    uint8_t probe[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    size_t sentByteCount = write(probe, sizeof(probe));
    ROS_INFO("sentByteCount: %i", sentByteCount);
    std::vector<uint8_t> response;
    size_t readByteCount = read(response, 200);
    ROS_INFO("getBMSBatteriesInfo readByteCount: %i", readByteCount);
    flush();
    printHexROS(response);
    if(response.size() > 4) {
        _battInfo = reinterpret_cast<BMSBatteriesInfo*>(response.data()+4);
        auto ntcsStart = reinterpret_cast<uint16_t*>(_battInfo + sizeof(BMSBatteriesInfo) + 1);
        _ntcs = std::vector<uint16_t>(ntcsStart, ntcsStart + _battInfo->NTCCount);
    }
    
    return nullptr;
}

std::vector<float> BMS::getVoltages() {
    uint8_t probe[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    size_t sentByteCount = write(probe, sizeof(probe));
    ROS_INFO("sentByteCount: %zu", sentByteCount);
    
    std::vector<uint8_t> response;
    size_t readByteCount = read(response, 200);
    ROS_INFO("getVoltages readByteCount: %zu", readByteCount);
    flush();
    printHexROS(response);
    if(readByteCount < 5) {
        ROS_ERROR("Invalid response size: %zu", readByteCount);
        return {};
    }

    uint8_t cell_count = response[3];
    size_t expected_size = 4 + cell_count * 2;
    
    if(readByteCount < expected_size) {
        ROS_ERROR("Response too small: %zu < %zu", readByteCount, expected_size);
        return {};
    }

    std::vector<float> voltages;
    voltages.reserve(cell_count);
    
    for(size_t i = 0; i < cell_count; ++i) {
        uint16_t raw = static_cast<uint16_t>(response[4 + i*2]) | 
                      (static_cast<uint16_t>(response[5 + i*2]) << 8);
        

        voltages.push_back(static_cast<float>(raw) / 1000.0f);
    }

    return voltages;
}

bool BMS::isAccessed() const {
    return _accessed;
}

bool BMS::isAnswerable() const {
    return _answerable;
}

void BMS::checkAnswerable() {
    uint8_t probe[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    size_t sentByteCount = write(probe, sizeof(probe));
    ROS_INFO("sentByteCount: %i", sentByteCount);
    std::vector<uint8_t> response;
    size_t readByteCount = read(response, 200);
    ROS_INFO("readByteCount: %i", readByteCount);
    flush();
    _answerable = readByteCount > 0;
}

void BMS::publishCallback(const ros::TimerEvent&) {
    sendBatterries();
}

void BMS::updateCallback(const ros::TimerEvent&) {
    getBMSBatteriesInfo();
    _voltages = getVoltages();
}

}