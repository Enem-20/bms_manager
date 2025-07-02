#ifndef C_BMS_HPP
#define C_BMS_HPP

#include <string>
#include <memory>

#include <ros/ros.h>
#include <serial/serial.h>

namespace serial {
    class Serial;

struct BMSBatteriesInfo {
    uint16_t totalVoltage;
    uint16_t current;
    uint16_t remainingCapacity;
    uint16_t nominalCapacity;
    uint16_t cycles;
    uint16_t productionDate;
    uint16_t equilibrium;
    uint16_t equilibriumHigh;
    uint16_t protectionStatus;
    uint8_t softwareVersion;
    uint8_t RSOC;
    uint8_t FET;
    uint8_t batteryStringsCount;
    uint8_t NTCCount;
};

class BMS : public Serial {
public:
    BMS(size_t id, ros::NodeHandle* nodeHandle, const std::string &port = "",
          uint32_t baudrate = 9600,
          Timeout timeout = Timeout::simpleTimeout(1000),
          bytesize_t bytesize = eightbits,
          parity_t parity = parity_none,
          stopbits_t stopbits = stopbits_one,
          flowcontrol_t flowcontrol = flowcontrol_none);
    ~BMS() override;

    void sendBatterries();
    size_t sendShutdown();
    BMSBatteriesInfo* getBMSBatteriesInfo();
    std::vector<uint16_t> getVoltages();

    bool isAccessed() const;
    bool isAnswerable() const;
private:
    int16_t calculateAverageCentiCelsius(const std::vector<int16_t>& temps);
    std::vector<int16_t> parseNTCsToCentiCelsius(const uint8_t* dataPtr, size_t byteCount);
    void checkAnswerable();
    void publishCallback(const ros::TimerEvent&);
    void updateCallback(const ros::TimerEvent&);

    BMSBatteriesInfo* _battInfo;
    size_t _id;
    ros::NodeHandle* _nodeHandle;
    ros::Publisher _publisher;
    ros::Timer _publishTimer;
    ros::Timer _updateTimer;
    std::vector<int16_t> _ntcs;
    std::vector<uint16_t> _voltages;
    bool _accessed = true;
    bool _answerable = true;
    uint8_t _seq = 0;
};

}

#endif