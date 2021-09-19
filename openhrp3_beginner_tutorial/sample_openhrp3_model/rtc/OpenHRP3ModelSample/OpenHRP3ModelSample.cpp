#include "OpenHRP3ModelSample.h"

#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>

#include <iostream>

OpenHRP3ModelSample::OpenHRP3ModelSample(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t OpenHRP3ModelSample::onInitialize(){
  hrp::BodyPtr robot = hrp::BodyPtr(new hrp::Body());

  // load model from modelloader
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0) comPos = nameServer.length();
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!hrp::loadBodyFromModelLoader(robot, "/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl",
                                    CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                    )){
    std::cerr << "failed to load model" << std::endl;
    return RTC::RTC_ERROR;
  }

  // print robot names
  std::cout << "loaded: " << robot->name() << std::endl;
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name << std::endl;
  }

  return RTC::RTC_OK;
}


static const char* OpenHRP3ModelSample_spec[] = {
  "implementation_id", "OpenHRP3ModelSample",
  "type_name",         "OpenHRP3ModelSample",
  "description",       "OpenHRP3ModelSample component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void OpenHRP3ModelSampleInit(RTC::Manager* manager) {
        RTC::Properties profile(OpenHRP3ModelSample_spec);
        manager->registerFactory(profile, RTC::Create<OpenHRP3ModelSample>, RTC::Delete<OpenHRP3ModelSample>);
    }
};
