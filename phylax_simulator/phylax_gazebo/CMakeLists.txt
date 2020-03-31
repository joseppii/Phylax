cmake_minimum_required(VERSION 2.8.3)
project(phylax_gazebo)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/phylax_world.launch)

install(DIRECTORY launch media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
