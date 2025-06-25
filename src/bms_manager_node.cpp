#include <string>
#include <vector>
#include <unistd.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <serial/serial.h>

serial::Serial usb_port;
serial::Serial usb_port2;

ros::Time last_shutdown_time = ros::Time(0);

const uint8_t test_cmd[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

bool testBMS(serial::Serial& port, const std::string& name) {
    try {
        port.setPort(name);
        port.setBaudrate(9600);
        port.setTimeout(serial::Timeout::simpleTimeout(200));
        port.open();

        if (!port.isOpen()) return false;

        port.flushInput();
        port.write(test_cmd, sizeof(test_cmd));
        usleep(100000); // 100 ms

        std::string response = port.read(64);
        if (!response.empty()) {
            ROS_INFO("Device %s responded with %zu bytes", name.c_str(), response.size());
            return true;
        } else {
            port.close();
            return false;
        }
    } catch (...) {
        return false;
    }
}

void detectPorts(serial::Serial& port1, serial::Serial& port2) {
    std::vector<std::string> candidates;
    for (int i = 0; i < 10; ++i) {
        candidates.push_back("/dev/ttyUSB" + std::to_string(i));
    }

    int found = 0;
    for (const auto& dev : candidates) {
        serial::Serial test;
        if (testBMS(test, dev)) {
            if (found == 0) {
                port1 = std::move(test);
            } else if (found == 1) {
                port2 = std::move(test);
            }
            found++;
            if (found == 2) break;
        }
    }

    if (found < 2) {
        ROS_ERROR("Could not find two valid BMS devices!");
        ros::shutdown();
    }
}

size_t switchOFF(serial::Serial& port, const std::string& name) {
    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    if (port.isOpen()) {
        size_t sent = port.write(shutdown_cmd, sizeof(shutdown_cmd));
        ROS_INFO("Sent shutdown to %s: %zu bytes", name.c_str(), sent);
        return sent;
    }
    return 0;
}

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (msg->channels.size() < 10) return;

    uint16_t ch10 = msg->channels[9];
    ROS_INFO("ch10: %u", ch10);

    if (ch10 > 1899) {
        ros::Time now = ros::Time::now();
        if ((now - last_shutdown_time).toSec() < 5.0) {
            ROS_INFO("Command ignored — wait 5s between sends");
            return;
        }

        last_shutdown_time = now;
        ROS_INFO("Sending shutdown to both BMS");

        switchOFF(usb_port, usb_port.getPort());
        switchOFF(usb_port2, usb_port2.getPort());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_to_bms_node");
    ros::NodeHandle nh;

    detectPorts(usb_port, usb_port2);

    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);

    ros::spin();

    usb_port.close();
    usb_port2.close();
    return 0;
}
