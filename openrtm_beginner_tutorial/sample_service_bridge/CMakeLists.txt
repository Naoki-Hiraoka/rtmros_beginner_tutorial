cmake_minimum_required(VERSION 3.0.2)
project(sample_service_bridge)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

add_subdirectory(rtc)
