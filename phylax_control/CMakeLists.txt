cmake_minimum_required(VERSION 2.8.3)
project(phylax_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/teleop.launch)

install(DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

