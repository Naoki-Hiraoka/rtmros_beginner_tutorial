cmake_minimum_required(VERSION 3.0.2)
project(sample_idl)

find_package(catkin REQUIRED COMPONENTS rtmbuild)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# add_custom_command to compile idl/*.idl file into c++
rtmbuild_genidl()

add_custom_target(genidl ALL DEPENDS RTMBUILD_${PROJECT_NAME}_genrpc)
