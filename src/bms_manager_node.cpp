#include <string>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <serial/serial.h>
#include <dirent.h>
#include <fcntl.h>

serial::Serial usb_port1;
serial::Serial usb_port2;
ros::Time last_shutdown_time = ros::Time(0);

bool testPort(serial::Serial& port, const std::string& name) {
    try {
        port.setPort(name);
        port.setBaudrate(9600);
        serial::Timeout t = serial::Timeout::simpleTimeout(300);
        port.setTimeout(t);
        port.open();

        if (!port.isOpen()) return false;

        uint8_t probe[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
        port.write(probe, sizeof(probe));
        std::vector<uint8_t> response(64);
        size_t len = port.read(response, response.size());

        port.flush();
        return len > 0;
    } catch (...) {
        return false;
    }
}

void detectWorkingPorts() {
    std::vector<std::string> candidates = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
    int found = 0;

    for (const auto& name : candidates) {
        if (found == 0 && testPort(usb_port1, name)) {
            ROS_INFO_STREAM("Found working port1: " << name);
            found++;
        } else if (found == 1 && testPort(usb_port2, name)) {
            ROS_INFO_STREAM("Found working port2: " << name);
            found++;
        }
        if (found == 2) break;
    }

    if (found < 2) {
        if (!usb_port1.isOpen()) ROS_ERROR("usb_port1 not found or failed to open");
        if (!usb_port2.isOpen()) ROS_ERROR("usb_port2 not found or failed to open");
    }
}

size_t sendShutdown(serial::Serial& port) {
    if (!port.isOpen()) return 0;

    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    return port.write(shutdown_cmd, sizeof(shutdown_cmd));
}

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (msg->channels.size() < 10) return;
    uint16_t ch10 = msg->channels[9];

    if (ch10 > 1899) {
        ros::Time now = ros::Time::now();
        if ((now - last_shutdown_time).toSec() < 5.0) {
            ROS_INFO("Ignoring command — wait 5 seconds between sends");
            return;
        }
        last_shutdown_time = now;

        if (!usb_port1.isOpen() || !usb_port2.isOpen()) {
            ROS_WARN("One or both ports closed. Rescanning...");
            detectWorkingPorts();
        }

        ROS_INFO("Sending shutdown command...");
        size_t sent = sendShutdown(usb_port1) + sendShutdown(usb_port2);
        ROS_INFO("Sent %zu bytes", sent);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bms_manager_node");
    ros::NodeHandle nh;

    detectWorkingPorts();

    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);
    ros::spin();

    usb_port1.close();
    usb_port2.close();
    return 0;
}
