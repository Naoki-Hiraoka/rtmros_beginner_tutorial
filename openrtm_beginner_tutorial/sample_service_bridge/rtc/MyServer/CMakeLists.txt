include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(MyServer SHARED MyServer.cpp MyServerService_impl.cpp)
target_link_libraries(MyServer ${catkin_LIBRARIES})
set_target_properties(MyServer PROPERTIES PREFIX "") # libMyServer.so -> MyServer.so
add_dependencies(MyServer RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(MyServerComp MyServerComp.cpp)
target_link_libraries(MyServerComp MyServer)
