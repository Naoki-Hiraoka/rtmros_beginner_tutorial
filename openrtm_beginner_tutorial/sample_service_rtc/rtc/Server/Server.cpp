#include "Server.h"
#include <iostream>

Server::Server(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ServerServicePort("service0")
{
  m_service0.setComponent(this);
}

RTC::ReturnCode_t Server::onInitialize(){
  m_ServerServicePort.registerProvider("service0", "MyOriginalService", m_service0);
  addPort(m_ServerServicePort);
  return RTC::RTC_OK;
}

CORBA::Long Server::addTwoInts(CORBA::Long a, CORBA::Long b) {
  std::cout << "Called addTwoInts: " << a << "+" << b << "=" << a+b << std::endl;
  return a+b;
}

CORBA::Boolean Server::addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time& sum) {
  sum.sec = a.sec + b.sec + (a.nsec + b.nsec)/1000000000;
  sum.nsec = (a.nsec + b.nsec)%1000000000;
  return true;
}

CORBA::Boolean Server::addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq& sum) {
  if(a.data.length() != b.data.length()) return false;
  sum.data.length(a.data.length());
  for(int i=0;i<a.data.length();i++) sum.data[i] = a.data[i] + b.data[i];
  return true;
}

static const char* Server_spec[] = {
  "implementation_id", "Server",
  "type_name",         "Server",
  "description",       "Server component",
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
    void ServerInit(RTC::Manager* manager) {
        RTC::Properties profile(Server_spec);
        manager->registerFactory(profile, RTC::Create<Server>, RTC::Delete<Server>);
    }
};
