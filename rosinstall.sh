cd /home/ubuntu/catkin_ws/src
git clone https://github.com/wjwwood/serial.git
cd ..
rm -rf build devel

echo MAKE
source /opt/ros/noetic/setup.bash
catkin_make

cp -f /home/ubuntu/catkin_ws/src/bms_manager/bms_manager.service /etc/systemd/system/bms_manager.service

systemctl daemon-reload
systemctl enable bms_manager.service
systemctl start bms_manager.service
