cmake_minimum_required(VERSION 3.0.2)
project(sample_openhrp3_model)

find_package(catkin REQUIRED COMPONENTS rtmbuild)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

add_subdirectory(rtc)
