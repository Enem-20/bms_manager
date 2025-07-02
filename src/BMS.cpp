#include "BMS.hpp"

#include <cstdint>
#include <algorithm>

#include <numeric>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>


#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

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
{
    _publisher = nodeHandle->advertise<mavros_msgs::Mavlink>("/mavlink/to", 10);
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

void BMS::sendBatterries() {
    if (!_battInfo) return;

    mavlink_message_t msg;
    mavlink_battery_status_t bat{};

    bat.id = static_cast<uint8_t>(_id);
    bat.battery_function = MAV_BATTERY_FUNCTION_AVIONICS;
    bat.type = MAV_BATTERY_TYPE_LIPO;
    bat.temperature = calculateAverageCentiCelsius(_ntcs);

    for (size_t i = 0; i < 10; ++i) {
        bat.voltages[i] = i < _voltages.size() ? _voltages[i] : UINT16_MAX;
    }

    for (size_t i = 0; i < 4; ++i) {
        size_t idx = 10 + i;
        bat.voltages_ext[i] = idx < _voltages.size() ? _voltages[idx] : UINT16_MAX;
    }

    bat.current_battery = static_cast<int16_t>(_battInfo->current);
    bat.battery_remaining = static_cast<int8_t>(_battInfo->RSOC);

    mavlink_msg_battery_status_encode(1, MAV_COMP_ID_BATTERY, &msg, &bat);

    mavros_msgs::Mavlink ros_msg;
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.sysid = msg.sysid;
    ros_msg.compid = msg.compid;
    ros_msg.msgid = msg.msgid;
    ros_msg.len = msg.len;
    ros_msg.seq = _seq++;
    ros_msg.checksum = msg.checksum;
    ros_msg.magic = msg.magic;
    ros_msg.payload64.resize((msg.len + 7) / 8);
    std::memcpy(ros_msg.payload64.data(), msg.payload64, ros_msg.payload64.size() * sizeof(uint64_t));

    ROS_INFO("publishing...");
    _publisher.publish(ros_msg);
}

size_t BMS::sendShutdown()  {
    if (!isOpen()) return 0;

    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    return write(shutdown_cmd, sizeof(shutdown_cmd));
}

BMSBatteriesInfo* BMS::getBMSBatteriesInfo() {
    static BMSBatteriesInfo battInfoCopy;
    
    uint8_t probe[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    size_t sentByteCount = write(probe, sizeof(probe));
    ROS_INFO("sentByteCount: %zu", sentByteCount);

    std::vector<uint8_t> response;
    size_t readByteCount = read(response, 200);
    ROS_INFO("getBMSBatteriesInfo readByteCount: %zu", readByteCount);
    flush();
    printHexROS(response);

    if (readByteCount < 5 + sizeof(BMSBatteriesInfo)) {
        ROS_ERROR("Too short for BMSBatteriesInfo");
        return nullptr;
    }

    const uint8_t* dataPtr = &response[4];

    memcpy(&battInfoCopy, dataPtr, sizeof(BMSBatteriesInfo));
    _battInfo = &battInfoCopy;

    uint8_t ntcCount = _battInfo->NTCCount;
    _ntcs.clear();
    _ntcs.reserve(ntcCount);
    ROS_INFO("ntcCount: %i", ntcCount);
    const uint8_t* ntcData = dataPtr + sizeof(BMSBatteriesInfo);
    parseNTCsToCentiCelsius(ntcData, ntcCount*2);

    return _battInfo;
}

std::vector<uint16_t> BMS::getVoltages() {
    uint8_t probe[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
    size_t sentByteCount = write(probe, sizeof(probe));
    ROS_INFO("sentByteCount: %zu", sentByteCount);
    
    std::vector<uint8_t> response;
    size_t readByteCount = read(response, 200);
    ROS_INFO("getVoltages readByteCount: %zu", readByteCount);
    flush();
    printHexROS(response);

    if (readByteCount < 5) {
        ROS_ERROR("Invalid response size: %zu", readByteCount);
        return {};
    }

    uint8_t dataLength = response[3];
    size_t expectedSize = dataLength + 6;

    if (readByteCount < expectedSize) {
        ROS_ERROR("Response too small: %zu < %zu", readByteCount, expectedSize);
        return {};
    }

    const uint8_t* dataPtr = &response[4];
    size_t cellCount = dataLength / 2;

    _voltages.resize(cellCount);
    for (size_t i = 0; i < cellCount; ++i) {
        uint16_t temp;
        memcpy(&temp, dataPtr + i * 2, 2);
        _voltages[i] = (temp >> 8) | (temp << 8);
    }

    return _voltages;
}

bool BMS::isAccessed() const {
    return _accessed;
}

bool BMS::isAnswerable() const {
    return _answerable;
}


int16_t BMS::calculateAverageCentiCelsius(const std::vector<int16_t>& temps)
{
    if (temps.empty())
        return -271;

    int64_t sum = 0;

    for (int16_t temp : temps)
        sum += temp;

    int16_t average = static_cast<int16_t>(sum / temps.size());
    
    return *std::max_element(temps.begin(), temps.end());;
}

std::vector<int16_t> BMS::parseNTCsToCentiCelsius(const uint8_t* dataPtr, size_t byteCount)
{
    if (byteCount % 2 != 0)
        return {};

    std::vector<int16_t> result;
    result.reserve(byteCount / 2);

    for (size_t i = 0; i < byteCount; i += 2)
    {
        uint16_t raw_be = static_cast<uint16_t>(dataPtr[i]) |
                          static_cast<uint16_t>(dataPtr[i + 1] << 8);

        std::cout << "0.1K: " << raw_be << '\n';
        int16_t centiC = static_cast<int16_t>(raw_be*10 - 27315);
        std::cout << "centiC: " << centiC << '\n';
        result.push_back(centiC);
    }

    _ntcs = result;
    return _ntcs;
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