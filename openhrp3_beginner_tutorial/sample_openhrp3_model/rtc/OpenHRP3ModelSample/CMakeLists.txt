include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(OpenHRP3ModelSample SHARED OpenHRP3ModelSample.cpp)
target_link_libraries(OpenHRP3ModelSample ${catkin_LIBRARIES})
set_target_properties(OpenHRP3ModelSample PROPERTIES PREFIX "") # libOpenHRP3ModelSample.so -> OpenHRP3ModelSample.so

rtmbuild_add_executable(OpenHRP3ModelSampleComp OpenHRP3ModelSampleComp.cpp)
target_link_libraries(OpenHRP3ModelSampleComp OpenHRP3ModelSample)
