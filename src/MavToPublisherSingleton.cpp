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

void MavToPublisher::send_heartbeat(const ros::TimerEvent&) {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        1,             
        180,           
        &msg,
        MAV_TYPE_GENERIC,        
        MAV_AUTOPILOT_INVALID,   
        0,                       
        0,                       
        MAV_STATE_ACTIVE         
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    mavros_msgs::Mavlink ros_msg;
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.len = msg.len;
    ros_msg.seq = msg.seq;
    ros_msg.sysid = msg.sysid;
    ros_msg.compid = msg.compid;
    ros_msg.msgid = msg.msgid;
    ros_msg.checksum = msg.checksum;
    ros_msg.magic = msg.magic;

    size_t payload_size = sizeof(msg.payload64);
    ros_msg.payload64.resize(payload_size / sizeof(uint64_t));
    memcpy(ros_msg.payload64.data(), msg.payload64, payload_size);

    _publisher.publish(ros_msg);
}

MavToPublisher::MavToPublisher(ros::NodeHandle* nodeHandle) {
    _nodeHandle = nodeHandle;
    _publisher = nodeHandle->advertise<mavros_msgs::Mavlink>("/mavlink/to", 100);
    heartbeat = nodeHandle->createTimer(ros::Duration(0.1), &MavToPublisher::send_heartbeat, this);
}