#include "Subscriber.h"

Subscriber::Subscriber(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataIn("listener", m_data)
{
}

RTC::ReturnCode_t Subscriber::onInitialize(){
  addInPort("listener", m_dataIn);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Subscriber::onExecute(RTC::UniqueId ec_id){
  if(m_dataIn.isNew()){
    m_dataIn.read();

    std::cout << "Subscribe: ";
    for(int i=0;i<m_data.data.length();i++){
      std::cout << m_data.data[i] << " ";
    }
    std::cout << std::endl;
  }

  return RTC::RTC_OK;
}

static const char* Subscriber_spec[] = {
  "implementation_id", "Subscriber",
  "type_name",         "Subscriber",
  "description",       "Subscriber component",
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
    void SubscriberInit(RTC::Manager* manager) {
        RTC::Properties profile(Subscriber_spec);
        manager->registerFactory(profile, RTC::Create<Subscriber>, RTC::Delete<Subscriber>);
    }
};
