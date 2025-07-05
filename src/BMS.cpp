#include "BMS.hpp"

#include <cstdint>
#include <algorithm>

#include <numeric>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>



#include "MavToPublisherSingleton.hpp"

namespace serial {

size_t BMS::id_counter = 2;
std::unordered_set<size_t> BMS::has;

void printHexROS(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (uint8_t byte : data) {
        oss << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(byte) << " ";
    }
    ROS_INFO_STREAM("Data in HEX: " << oss.str());
}

BMS::BMS(ros::NodeHandle* nodeHandle, const std::string &port,
          uint32_t baudrate,
          Timeout timeout,
          bytesize_t bytesize,
          parity_t parity,
          stopbits_t stopbits,
          flowcontrol_t flowcontrol
        ) 
    : Serial(port, baudrate, timeout, bytesize, parity, stopbits, flowcontrol)
    , _nodeHandle(nodeHandle)
    , _gen(_rd())
    , _dist(0, 6303)
{
    auto hasIt = has.find(id_counter);
    if(hasIt != has.end()) {
        if(id_counter == 2) {
            ++id_counter;
        }
        else if(id_counter == 3){
            --id_counter;
        }
    }

    has.emplace(id_counter);
    _id = id_counter;
    //_publisher = nodeHandle->advertise<mavros_msgs::Mavlink>("/mavlink/to", 10);
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
    _publishTimer = _nodeHandle->createTimer(ros::Duration(1.0), &BMS::publishCallback, this);
    _updateTimer = _nodeHandle->createTimer(ros::Duration(2.0), &BMS::updateCallback, this);
}

BMS::BMS(const std::string &port, ros::NodeHandle* nodeHandle) 
    : Serial()
    , _nodeHandle(nodeHandle)
{
    auto hasIt = has.find(id_counter);
    if(hasIt != has.end()) {
        if(id_counter == 2) {
            ++id_counter;
        }
        else if(id_counter == 3){
            --id_counter;
        }
    }

    has.emplace(id_counter);
    _id = id_counter;
    _accessed = true;
    _answerable = true;

    _battInfo = std::make_shared<BMSBatteriesInfo>();
    _battInfo->current = 34;
    _battInfo->batteryStringsCount = 14;
    _battInfo->NTCCount = 4;
    _battInfo->RSOC = 54;
    _battInfo->cycles = 3456;
    _battInfo->totalVoltage = 14;
    _ntcs.resize(5);
    for(auto& ntc : _ntcs) {
        ntc = _dist(_gen);
    }
    _voltages.resize(14);
    for(auto& voltage : _voltages) {
        voltage = _dist(_gen);
    }

    _publishTimer = _nodeHandle->createTimer(ros::Duration(3.0f), &BMS::publishCallback, this);
    _updateTimer = _nodeHandle->createTimer(ros::Duration(2.0), &BMS::updateCallback, this);
}

BMS::~BMS() {
    has.erase(_id);
    _publishTimer.stop();
    _updateTimer.stop();
    _publisher.shutdown();
    close();
}

void BMS::stopTimers() {
    _publishTimer.stop();
    _updateTimer.stop();
}

void BMS::reconnect() {
    _publishTimer.stop();
    _updateTimer.stop();
    close();
    open();
    ROS_INFO("Before: if (access(port.c_str(), R_OK | W_OK) != 0) {");
    if (access(getPort().c_str(), R_OK | W_OK) != 0) {
        ROS_ERROR_STREAM("Cannot access port " << getPort().c_str() << " — permission denied.");
        _accessed = false;
    }
    ROS_INFO("After: if (access(port.c_str(), R_OK | W_OK) != 0) {");
    if (!isOpen()) {
        ROS_ERROR_STREAM("Port " << getPort().c_str() << " did not open (no exception thrown)");
        return;
    }
    ROS_INFO("After: if (!isOpen()) {");
    checkAnswerable();
    ROS_INFO("After: checkAnswerable();");
    updateCallback({});
    _publishTimer.start();
    _updateTimer.start();
}

void BMS::reconnect(const std::string& new_port) {
    _publishTimer.stop();
    _updateTimer.stop();
    close();
    
    if (!new_port.empty() && new_port != getPort()) {
        setPort(new_port);
    }

    try {
        open();
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Reconnect open error: " << e.what());
    }

    if (access(getPort().c_str(), R_OK | W_OK) != 0) {
        ROS_ERROR_STREAM("Cannot access port " << getPort() << " — permission denied.");
        _accessed = false;
    } else {
        _accessed = true;
    }

    if (!isOpen()) {
        ROS_ERROR_STREAM("Port " << getPort() << " did not open.");
        _answerable = false;
        return;
    }

    checkAnswerable();

    updateCallback({});
    _publishTimer.start();
    _updateTimer.start();
}

void BMS::sendBatterries() {
    ROS_INFO("publish: %i", _ros_msg.seq);
    ROS_INFO("publishing...");
    //_publisher.publish(_ros_msg);
    MavToPublisher::getInstance(_nodeHandle)->getPub().publish(_ros_msg);
    //_publisher.publish(ros_msg);
}

size_t BMS::sendShutdown()  {
    if (!isOpen()) return 0;

    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    size_t byteCount = 0;
    size_t readByteCount = 0;
    std::vector<uint8_t> response;
    try {
        //std::lock_guard<std::mutex>(&mut);
        byteCount = write(shutdown_cmd, sizeof(shutdown_cmd));
        readByteCount = read(response, 200);
    }
    catch(...) {
        //std::lock_guard<std::mutex>(&mut);
        close();
    }
    return byteCount;
}

std::shared_ptr<BMSBatteriesInfo> BMS::getBMSBatteriesInfo() {
    BMSBatteriesInfo* battInfoCopy = new BMSBatteriesInfo;
    uint8_t probe[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    size_t sentByteCount = 0;
    size_t readByteCount = 0;
    std::vector<uint8_t> response;
    try { 
        sentByteCount = write(probe, sizeof(probe));
        ROS_INFO("sentByteCount: %zu", sentByteCount);

        
        readByteCount = read(response, 200);
        ROS_INFO("getBMSBatteriesInfo readByteCount: %zu", readByteCount);
        flush();
        printHexROS(response);
    }
    catch(...) {
        close();
        return _battInfo;
    }
    if (readByteCount < 5 + sizeof(BMSBatteriesInfo)) {
        ROS_ERROR("Too short for BMSBatteriesInfo");
        return _battInfo;
    }

    const uint8_t* dataPtr = &response[4];

    memcpy(battInfoCopy, dataPtr, sizeof(BMSBatteriesInfo));
    _battInfo.reset(battInfoCopy);
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
    size_t sentByteCount = 0;
    std::vector<uint8_t> response;
    size_t readByteCount = 0;
    try { 
        sentByteCount = write(probe, sizeof(probe));
        ROS_INFO("sentByteCount: %zu", sentByteCount);
        
        
        readByteCount = read(response, 200);
        ROS_INFO("getVoltages readByteCount: %zu", readByteCount);
        flush();
        printHexROS(response);
    }
    catch(...) {
        close();
        return _voltages;
    }
    if (readByteCount < 5) {
        ROS_ERROR("Invalid response size: %zu", readByteCount);
        return _voltages;
    }

    uint8_t dataLength = response[3];
    size_t expectedSize = dataLength + 6;

    if (readByteCount < expectedSize) {
        ROS_ERROR("Response too small: %zu < %zu", readByteCount, expectedSize);
        return _voltages;
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

void BMS::checkAnswerable() {
    try {
        uint8_t probe[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
        size_t sentByteCount = write(probe, sizeof(probe));
        ROS_INFO("sentByteCount: %i", sentByteCount);
        std::vector<uint8_t> response;
        size_t readByteCount = read(response, 200);
        ROS_INFO("readByteCount: %i", readByteCount);
        flush();
        _answerable = readByteCount > 0;
    }
    catch(...) {
        close();
        _answerable = false;
        return;
    }
    
}

const std::string BMS::getPath() const {
    return getPort();
}

void BMS::prepareTestFrame() {
    if (!_battInfo) return;

    mavlink_message_t msg;
    mavlink_battery_status_t bat{};
    static size_t counter = 0;
    bat.id = static_cast<uint8_t>(_id + (counter % 2));
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
    ROS_INFO("_battInfo->current before : %i", _battInfo->current);
    bat.current_battery = static_cast<int16_t>(((_battInfo->current & 0xFF) << 8) | ((_battInfo->current >> 8) & 0xFF));
    bat.battery_remaining = static_cast<int8_t>(_battInfo->RSOC);

    mavlink_msg_battery_status_encode(1, MAV_COMP_ID_BATTERY, &msg, &bat);

    _ros_msg.header.stamp = ros::Time::now();
    _ros_msg.sysid = msg.sysid;
    _ros_msg.compid = 180;
    _ros_msg.msgid = msg.msgid;
    _ros_msg.len = msg.len;
    _ros_msg.seq = _seq++;
    _ros_msg.checksum = msg.checksum;
    _ros_msg.magic = msg.magic;
    _ros_msg.payload64.resize((msg.len + 7) / 8);
    std::memcpy(_ros_msg.payload64.data(), msg.payload64, _ros_msg.payload64.size() * sizeof(uint64_t));
    ++counter;
}

void BMS::prepareFrame() {
    if (!_battInfo) return;

    mavlink_message_t msg;
    msg.sysid = 1;
    static size_t counter = 0;
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
    ROS_INFO("_battInfo->current before : %i", _battInfo->current);
    bat.current_battery = static_cast<int16_t>(((_battInfo->current & 0xFF) << 8) | ((_battInfo->current >> 8) & 0xFF));
    bat.battery_remaining = static_cast<int8_t>(_battInfo->RSOC);

    mavlink_msg_battery_status_encode(1, MAV_COMP_ID_BATTERY, &msg, &bat);

    _ros_msg.header.stamp = ros::Time::now();
    _ros_msg.sysid = msg.sysid;
    _ros_msg.compid = 180 + _id -2;
    _ros_msg.msgid = msg.msgid;
    _ros_msg.len = msg.len;
    _ros_msg.seq = _seq++;
    _ros_msg.checksum = msg.checksum;
    _ros_msg.magic = msg.magic;
    _ros_msg.payload64.resize((msg.len + 7) / 8);
    std::memcpy(_ros_msg.payload64.data(), msg.payload64, _ros_msg.payload64.size() * sizeof(uint64_t));
    ++counter;
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
        uint16_t raw_be = (static_cast<uint16_t>(dataPtr[i]) << 8) | static_cast<uint16_t>(dataPtr[i + 1]);

        std::cout << "0.1K: " << raw_be << '\n';
        int16_t centiC = static_cast<int16_t>(raw_be*10 - 27315);
        std::cout << "centiC: " << centiC << '\n';
        result.push_back(centiC);
    }

    _ntcs = result;
    return _ntcs;
}

void BMS::publishCallback(const ros::TimerEvent&) {
    sendBatterries();
}

void BMS::updateCallback(const ros::TimerEvent&) {
    getBMSBatteriesInfo();
    _voltages = getVoltages();
    prepareFrame();
}

}