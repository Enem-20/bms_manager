#include "MavToPublisherSingleton.hpp"

#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>

MavToPublisher* MavToPublisher::instance;

MavToPublisher* MavToPublisher::getInstance(ros::NodeHandle* nodeHandle) {
    if(instance)
        return instance;
    return instance = new MavToPublisher(nodeHandle);
}

ros::Publisher& MavToPublisher::getPub() {
    return _publisher;
}

MavToPublisher::MavToPublisher(ros::NodeHandle* nodeHandle) {
    _publisher = nodeHandle->advertise<mavros_msgs::Mavlink>("/mavlink/to", 10);
}