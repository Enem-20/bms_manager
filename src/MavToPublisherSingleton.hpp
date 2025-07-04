#ifndef C_MAV_TO_PUBLISHER_HPP
#define C_MAV_TO_PUBLISHER_HPP

#include <ros/ros.h>

class MavToPublisher {
public:
    static MavToPublisher* getInstance(ros::NodeHandle* nodeHandle = nullptr);
    ros::Publisher& getPub();
private:
    MavToPublisher(ros::NodeHandle* nodeHandle);
    static MavToPublisher* instance;
    
    void send_heartbeat();
    ros::NodeHandle* _nodeHandle;
    ros::Publisher _publisher;
    ros::Timer heartbeat;
};

#endif