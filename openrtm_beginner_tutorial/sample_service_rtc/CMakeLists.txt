cmake_minimum_required(VERSION 3.0.2)
project(sample_service_rtc)

find_package(catkin REQUIRED COMPONENTS rtmbuild)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# generate idl
rtmbuild_genidl()

add_subdirectory(rtc)
