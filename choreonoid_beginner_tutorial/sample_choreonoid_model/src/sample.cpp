#include <iostream>

#include <cnoid/Body>
#include <cnoid/BasicSensors>

#include <cnoid/BodyLoader>

int main(void){
  // load robot
  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl");
  if (!robot){
    std::cerr << "failed to load model" << std::endl;
    return 1;
  }

  // print robot names
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name() << std::endl;
  }

  return 0;
}
