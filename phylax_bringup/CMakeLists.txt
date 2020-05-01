cmake_minimum_required(VERSION 2.8.3)
project(phylax_bringup)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS roslaunch)

catkin_package()

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

