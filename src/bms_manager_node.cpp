#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <serial/serial.h>

serial::Serial usb_port;
serial::Serial usb_port2;

void tryOpen(const serial::Serial& usb_port, std::string_view port) {
    try {
        usb_port.setPort(port.data());
        usb_port.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        usb_port.setTimeout(to);
        usb_port.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open the port: " << e.what());
    }
}

void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    
    if (msg->channels.size() < 10) return;
    uint16_t ch10 = msg->channels[9];
    ROS_INFO("ch10 is: %i", ch10);
    
    if (ch10 > 1899) {
        ROS_INFO_STREAM("RC Switch ON detected — sending BMS shutdown command");
        uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
        if (usb_port.isOpen()) {
            size_t sent = usb_port.write(shutdown_cmd, sizeof(shutdown_cmd));
            ROS_INFO("sent bytes for /dev/ttyUSB0: %i", sent);
        }
        else {
            ROS_INFO("sent bytes for /dev/ttyUSB0: %i", 0);
            tryOpen(usbPort, "/dev/ttyUSB0");
        }
        if (usb_port2.isOpen()) {
            sent = usb_port2.write(shutdown_cmd, sizeof(shutdown_cmd));
            ROS_INFO("sent bytes for /dev/ttyUSB2: %i", sent);
        }
        else {
            ROS_INFO("sent bytes for /dev/ttyUSB2: %i", 0);
            tryOpen(usbPort2, "/dev/ttyUSB2");
        }
        

    } 
    //else if (ch10 == 0) {
    //    ROS_INFO_STREAM("RC Switch OFF detected — not sending command");
    //}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_to_bms_node");
    ros::NodeHandle nh;

    tryOpen(usbPort, "/dev/ttyUSB0");
    tryOpen(usbPort2, "/dev/ttyUSB2");

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
