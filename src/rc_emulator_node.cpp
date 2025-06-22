#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "rc_emulator");
    ros::NodeHandle nh;
    
    // Публикатор для переопределения RC
    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>(
        "/mavros/rc/override", 10);

    // Настройка каналов (пример для квадрокоптера)
    const int ROLL_CH = 0;     // Канал крена
    const int PITCH_CH = 1;    // Канал тангажа
    const int THROTTLE_CH = 2; // Канал газа
    const int YAW_CH = 3;      // Канал рысканья

    ros::Rate rate(10);
    ROS_INFO("rc emulator started");
    while (ros::ok()) {
        mavros_msgs::OverrideRCIn rc_msg;

        for (auto& channel : rc_msg.channels) {
            channel = mavros_msgs::OverrideRCIn::CHAN_NOCHANGE;
        }

        rc_msg.channels[ROLL_CH] = 1500;    // Нейтраль
        rc_msg.channels[PITCH_CH] = 1500;   // Нейтраль
        rc_msg.channels[THROTTLE_CH] = 1200; // Низкий газ
        rc_msg.channels[YAW_CH] = 1500;     // Нейтраль
        
        // Или динамическое управление:
        // rc_msg.channels[ROLL_CH] = 1500 + 200 * sin(ros::Time::now().toSec());
        
        // Публикация сообщения
        rc_override_pub.publish(rc_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}