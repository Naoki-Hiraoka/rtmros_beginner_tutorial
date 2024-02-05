#include "Client.h"
#include <iostream>

Client::Client(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ClientServicePort("service0")
{
}

RTC::ReturnCode_t Client::onInitialize(){
  m_ClientServicePort.registerConsumer("service0", "MyOriginalService", m_consumer);
  addPort(m_ClientServicePort);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Client::onExecute(RTC::UniqueId ec_id){
  if(m_consumer._ptr()){

    int a1=1;
    int b1=2;
    int sum1 = m_consumer->addTwoInts(1,2);
    std::cout << "Call addTwoInts: " << a1 << "+" << b1 << "=" << sum1 << std::endl;

    RTC::Time a2; a2.sec=1; a2.nsec=0;
    RTC::Time b2; b2.sec=2; b2.nsec=0;
    RTC::Time sum2;
    m_consumer->addTwoTime(a2,b2,sum2);

    RTC::TimedDoubleSeq a3; a3.data.length(3); for(size_t i=0;i<a3.data.length();i++) a3.data[i]=1;
    RTC::TimedDoubleSeq b3; b3.data.length(3); for(size_t i=0;i<b3.data.length();i++) b3.data[i]=1;
    RTC::TimedDoubleSeq* sum3;
    m_consumer->addTwoTimedDoubleSeq(a3,b3,sum3);
    delete sum3;
  }

  return RTC::RTC_OK;
}

static const char* Client_spec[] = {
  "implementation_id", "Client",
  "type_name",         "Client",
  "description",       "Client component",
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
    void ClientInit(RTC::Manager* manager) {
        RTC::Properties profile(Client_spec);
        manager->registerFactory(profile, RTC::Create<Client>, RTC::Delete<Client>);
    }
};
