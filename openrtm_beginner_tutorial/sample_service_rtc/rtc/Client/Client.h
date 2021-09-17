#ifndef Client_H
#define Client_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>

#include <rtm/idl/BasicDataType.hh>

#include "sample_service_rtc/idl/MyServiceSample.hh"

class Client : public RTC::DataFlowComponentBase{
protected:
  RTC::CorbaConsumer<MySample::MyOriginalService> m_consumer;
  RTC::CorbaPort m_ClientServicePort;

public:
  Client(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};


extern "C"
{
  void ClientInit(RTC::Manager* manager);
};

#endif // Client_H
