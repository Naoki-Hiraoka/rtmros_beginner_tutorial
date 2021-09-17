#include "Publisher.h"
#include <iostream>

Publisher::Publisher(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataOut("chatter", m_data)
{
}

RTC::ReturnCode_t Publisher::onInitialize(){
  addOutPort("chatter", m_dataOut);
  m_data.data.length(3);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Publisher::onExecute(RTC::UniqueId ec_id){
  loop++;

  coil::TimeValue coiltm(coil::gettimeofday());
  m_data.tm.sec  = coiltm.sec();
  m_data.tm.nsec = coiltm.usec() * 1000;

  for(int i=0;i<m_data.data.length();i++){
    m_data.data[i] = loop;
  }

  m_dataOut.write();

  std::cout << "Publish: " << loop << std::endl;
  return RTC::RTC_OK;
}

static const char* Publisher_spec[] = {
  "implementation_id", "Publisher",
  "type_name",         "Publisher",
  "description",       "Publisher component",
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
    void PublisherInit(RTC::Manager* manager) {
        RTC::Properties profile(Publisher_spec);
        manager->registerFactory(profile, RTC::Create<Publisher>, RTC::Delete<Publisher>);
    }
};
