cmake_minimum_required(VERSION 3.0.2)
project(sample_ros_rtc)

find_package(catkin REQUIRED COMPONENTS rtmbuild)

rtmbuild_init()

catkin_package()

add_subdirectory(rtc)
