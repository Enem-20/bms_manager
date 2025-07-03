#ifndef C_BMS_FACTORY_HPP
#define C_BMS_FACTORY_HPP

#include <vector>
#include <string>


namespace serial {
    class BMS;
}

namespace ros {
    class NodeHandle;
}

class BMSFactory {
public:
    static void closeBMSes(std::vector<serial::BMS*>& bmses);
    static std::vector<serial::BMS*> scanForBMS(std::vector<serial::BMS*>& bmses, const std::string& path, ros::NodeHandle& nh);
private:
    static bool alreadyHas(std::vector<serial::BMS*>& bmses, const std::string& path);
};

#endif