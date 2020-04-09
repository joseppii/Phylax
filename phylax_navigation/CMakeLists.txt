cmake_minimum_required(VERSION 2.8.3)
project(phylax_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/include/gmapping.launch)

install(
DIRECTORY launch maps params
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
