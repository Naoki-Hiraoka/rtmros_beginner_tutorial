include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Publisher SHARED Publisher.cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})
set_target_properties(Publisher PROPERTIES PREFIX "") # libPublisher.so -> Publisher.so

rtmbuild_add_executable(PublisherComp PublisherComp.cpp)
target_link_libraries(PublisherComp Publisher)
