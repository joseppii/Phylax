cmake_minimum_required(VERSION 2.8.3)
project(phylax_viz)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roslaunch)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

roslaunch_add_file_check(launch/view_model.launch)
roslaunch_add_file_check(launch/view_robot.launch)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
