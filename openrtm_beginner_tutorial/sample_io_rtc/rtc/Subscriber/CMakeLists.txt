include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Subscriber SHARED Subscriber.cpp)
target_link_libraries(Subscriber ${catkin_LIBRARIES})
set_target_properties(Subscriber PROPERTIES PREFIX "") # libSubscriber.so -> Subscriber.so

rtmbuild_add_executable(SubscriberComp SubscriberComp.cpp)
target_link_libraries(SubscriberComp Subscriber)
