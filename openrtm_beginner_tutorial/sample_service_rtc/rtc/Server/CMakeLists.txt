include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Server SHARED Server.cpp ServerService_impl.cpp)
target_link_libraries(Server ${catkin_LIBRARIES})
set_target_properties(Server PROPERTIES PREFIX "") # libServer.so -> Server.so
add_dependencies(Server RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(ServerComp ServerComp.cpp)
target_link_libraries(ServerComp Server)
