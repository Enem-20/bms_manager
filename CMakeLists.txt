cmake_minimum_required(VERSION 3.0.2)
project(bms_manager)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  mavros_msgs
  serial
  mavros
)

find_package(mavlink REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bms_manager_node src/bms_manager_node.cpp src/BMS.cpp src/BMSFactory.cpp src/MavToPublisherSingleton.cpp)

set_property(TARGET bms_manager_node PROPERTY CXX_STANDARD 17)

target_link_libraries(bms_manager_node
  ${catkin_LIBRARIES}
)

add_executable(rc_emulator_node src/rc_emulator_node.cpp)

target_link_libraries(rc_emulator_node
  ${catkin_LIBRARIES}
)