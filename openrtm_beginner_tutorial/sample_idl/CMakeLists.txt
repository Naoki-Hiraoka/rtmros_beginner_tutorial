cmake_minimum_required(VERSION 3.0.2)
project(sample_idl)

find_package(catkin REQUIRED COMPONENTS rtmbuild)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# generate idl
rtmbuild_genidl()
