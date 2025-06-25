#include <string>

#include <unistd.h>

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <serial/serial.h>

serial::Serial usb_port;
serial::Serial usb_port2;



void tryOpen(serial::Serial& usb_port, const std::string& port) {
    try {
        usb_port.setPort(port.c_str());
        usb_port.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        usb_port.setTimeout(to);
        usb_port.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open the port: " << e.what());
    }
}

size_t switchOFF(serial::Serial& usb_port, const std::string& port) {
    uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    size_t sent = 0;
    if (usb_port.isOpen()) {
        sent = usb_port.write(shutdown_cmd, sizeof(shutdown_cmd));
    }
    else {
        tryOpen(usb_port, port);
    }
    ROS_INFO("sent bytes for %s: %i", sent, port.c_str());
}

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    
    if (msg->channels.size() < 10) return;
    uint16_t ch10 = msg->channels[9];
    ROS_INFO("ch10 is: %i", ch10);
    
    if (ch10 > 1899) {
        ROS_INFO_STREAM("RC Switch ON detected — sending BMS shutdown command");
        size_t sentGeneral = 0;
        sentGeneral += switchOFF(usb_port, "/dev/ttyUSB0");
        sentGeneral += switchOFF(usb_port2, "/dev/ttyUSB2");
        if(sentGeneral > 0) sleep(2);
    } 
    //else if (ch10 == 0) {
    //    ROS_INFO_STREAM("RC Switch OFF detected — not sending command");
    //}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_to_bms_node");
    ros::NodeHandle nh;

    tryOpen(usb_port, "/dev/ttyUSB0");
    tryOpen(usb_port2, "/dev/ttyUSB2");

    if (!usb_port.isOpen()) {
        ROS_ERROR("Port wasn't open");
    }
    else {
        ROS_INFO("Port /dev/ttyUSB0 is open");
    }

    if (!usb_port2.isOpen()) {
        ROS_ERROR("Port wasn't open");
        //return -1;
    }
    else {
        ROS_INFO("Port /dev/ttyUSB2 is open");
    }
    
    

    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);

    ros::spin();

    usb_port.close();
    return 0;
}
