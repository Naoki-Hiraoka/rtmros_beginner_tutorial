#include "MyServer.h"
#include <iostream>

MyServer::MyServer(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_MyServerServicePort("service0")
{
  m_service0.setComponent(this);
}

RTC::ReturnCode_t MyServer::onInitialize(){
  m_MyServerServicePort.registerProvider("service0", "MyBridgeService", m_service0);
  addPort(m_MyServerServicePort);
  return RTC::RTC_OK;
}

CORBA::Boolean MyServer::addTwoInts(CORBA::Long a, CORBA::Long b, CORBA::Long& sum) {
  std::cout << "Called addTwoInts: " << a << "+" << b << "=" << a+b << std::endl;
  sum = a+b;
  return true;
}

static const char* MyServer_spec[] = {
  "implementation_id", "MyServer",
  "type_name",         "MyServer",
  "description",       "MyServer component",
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
    void MyServerInit(RTC::Manager* manager) {
        RTC::Properties profile(MyServer_spec);
        manager->registerFactory(profile, RTC::Create<MyServer>, RTC::Delete<MyServer>);
    }
};
