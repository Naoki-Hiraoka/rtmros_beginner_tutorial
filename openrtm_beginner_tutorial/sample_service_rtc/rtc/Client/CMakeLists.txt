include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Client SHARED Client.cpp)
target_link_libraries(Client ${catkin_LIBRARIES})
set_target_properties(Client PROPERTIES PREFIX "") # libClient.so -> Client.so
add_dependencies(Client RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(ClientComp ClientComp.cpp)
target_link_libraries(ClientComp Client)
