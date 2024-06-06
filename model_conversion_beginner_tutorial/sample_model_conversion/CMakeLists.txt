cmake_minimum_required(VERSION 3.0.2)
project(sample_model_conversion)

find_package(catkin REQUIRED COMPONENTS
  hrpsys_ros_bridge)

# get openhrp3 path
execute_process(COMMAND pkg-config openhrp3.1 --variable=idl_dir
  OUTPUT_VARIABLE _OPENHRP3_IDL_DIR
  RESULT_VARIABLE _OPENHRP3_RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(_OPENHRP3_MODEL_DIR ${_OPENHRP3_IDL_DIR}/../sample/model)

catkin_package()

####################
## convert models ##
####################

compile_openhrp_model(${_OPENHRP3_MODEL_DIR}/sample1.wrl SampleRobot)
