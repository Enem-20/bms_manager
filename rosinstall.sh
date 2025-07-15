cd /home/ubuntu/catkin_ws/src
git clone https://github.com/wjwwood/serial.git
cd ..
rm -rf build devel

echo MAKE
source /opt/ros/noetic/setup.bash
catkin_make install

chmod +x /home/ubuntu/catkin_ws/src/bms_manager_setup.bash

rosrun robot_upstart install bms_manager/launch/bms_manager_node.launch \
        --job bms_manager \
        --user ubuntu \
        --setup /home/ubuntu/catkin_ws/install/setup.bash \
        --wait

systemctl daemon-reexec
systemctl daemon-reload

systemctl stop bms_manager

systemctl enable bms_manager
systemctl start bms_manager
