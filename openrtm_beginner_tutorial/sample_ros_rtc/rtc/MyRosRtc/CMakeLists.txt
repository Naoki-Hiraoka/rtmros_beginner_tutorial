find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp geometry_msgs tf2)

include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(MyRosRtc SHARED MyRosRtc.cpp)
target_link_libraries(MyRosRtc ${catkin_LIBRARIES})
set_target_properties(MyRosRtc PROPERTIES PREFIX "") # libMyRosRtc.so -> MyRosRtc.so

rtmbuild_add_executable(MyRosRtcComp MyRosRtcComp.cpp)
target_link_libraries(MyRosRtcComp MyRosRtc)
