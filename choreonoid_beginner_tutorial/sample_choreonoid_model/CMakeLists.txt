cmake_minimum_required(VERSION 3.0.2)
project(sample_choreonoid_model)

find_package(catkin REQUIRED)
find_package(choreonoid REQUIRED)
catkin_package()

include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})
add_executable(sample src/sample.cpp)
target_link_libraries(sample ${CHOREONOID_BODY_LIBRARIES})

install(TARGETS sample
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
