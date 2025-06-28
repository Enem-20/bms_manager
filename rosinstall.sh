apt-get install ros-noetic-serial                                                   -y
cd /home/ubuntu/catkin_ws

echo MAKE
source /opt/ros/noetic/setup.bash
catkin_make

cp -f /home/ubuntu/catkin_ws/src/bms_manager/bms_manager.service /etc/systemd/system/bms_manager.service

systemctl daemon-reload
systemctl enable bms_manager.service
systemctl start bms_manager.service
