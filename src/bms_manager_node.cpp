#include <string>
#include <vector>
#include <unistd.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <serial/serial.h>
#include <dirent.h>

std::unique_ptr<serial::Serial> usb_port1;
std::unique_ptr<serial::Serial> usb_port2;

ros::Time last_shutdown_time = ros::Time(0);

// Тестовая команда и ожидаемый ответ
const uint8_t TEST_CMD[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

// Проверка порта
bool testBMS(const std::string& port_name) {
    try {
        serial::Serial port;
        port.setPort(port_name);
        port.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
        port.setTimeout(timeout);
        port.open();
        if (!port.isOpen()) return false;

        const uint8_t TEST_CMD[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
        port.write(TEST_CMD, sizeof(TEST_CMD));
        usleep(100 * 1000); // 100 мс

        size_t available = port.available();
        if (available == 35) {
            std::vector<uint8_t> buffer(35);
            port.read(buffer, 35);
            // Можно дополнительно сверять начальный и конечный байты:
            if (buffer.front() == 0xDD && buffer.back() == 0x77) {
                return true;
            }
        }

        port.close();
    } catch (...) {
        return false;
    }
    return false;
}

// Получение списка /dev/ttyUSB*
std::vector<std::string> find_usb_ports() {
    std::vector<std::string> ports;
    DIR* dev = opendir("/dev");
    if (!dev) return ports;

    struct dirent* ent;
    while ((ent = readdir(dev)) != NULL) {
        std::string name(ent->d_name);
        if (name.find("ttyUSB") == 0) {
            ports.push_back("/dev/" + name);
        }
    }
    closedir(dev);
    return ports;
}

// Поиск двух рабочих портов
void detectPorts() {
    std::vector<std::string> ports = find_usb_ports();
    int found = 0;
    for (const auto& port : ports) {
        if (testBMS(port)) {
            if (found == 0) usb_port1 = std::make_unique<serial::Serial>(port, 9600, serial::Timeout::simpleTimeout(1000));
            else if (found == 1) usb_port2 = std::make_unique<serial::Serial>(port, 9600, serial::Timeout::simpleTimeout(1000));
            found++;
        }
        if (found == 2) break;
    }

    if (!usb_port1 || !usb_port1->isOpen()) {
        ROS_ERROR("usb_port1 not found or failed to open");
    } else {
        ROS_INFO("usb_port1 is %s", usb_port1->getPort().c_str());
    }

    if (!usb_port2 || !usb_port2->isOpen()) {
        ROS_ERROR("usb_port2 not found or failed to open");
    } else {
        ROS_INFO("usb_port2 is %s", usb_port2->getPort().c_str());
    }
}

// Отправка команды выключения
void sendShutdown(serial::Serial& port) {
    const uint8_t shutdown_cmd[] = {0xDD, 0x5A, 0xE1, 0x02, 0x00, 0x02, 0xFF, 0x1B, 0x77};
    if (port.isOpen()) {
        port.write(shutdown_cmd, sizeof(shutdown_cmd));
        ROS_INFO("Sent shutdown to %s", port.getPort().c_str());
    }
}

// RC callback
void rc_callback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (msg->channels.size() < 10) return;
    uint16_t ch10 = msg->channels[9];
    if (ch10 > 1899) {
        ros::Time now = ros::Time::now();
        if ((now - last_shutdown_time).toSec() < 5.0) {
            ROS_INFO("Too soon, ignoring shutdown request");
            return;
        }
        last_shutdown_time = now;
        ROS_INFO("RC switch triggered, sending shutdown");

        if (usb_port1) sendShutdown(*usb_port1);
        if (usb_port2) sendShutdown(*usb_port2);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_to_bms_node");
    ros::NodeHandle nh;

    detectPorts();

    ros::Subscriber sub = nh.subscribe("/mavros/rc/in", 10, rc_callback);
    ros::spin();

    if (usb_port1) usb_port1->close();
    if (usb_port2) usb_port2->close();

    return 0;
}
