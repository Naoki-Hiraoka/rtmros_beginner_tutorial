#ifndef Subscriber_H
#define Subscriber_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh> // for RTC::TimedDoubleSeq

class Subscriber : public RTC::DataFlowComponentBase{
protected:
  RTC::TimedDoubleSeq m_data;
  RTC::InPort<RTC::TimedDoubleSeq> m_dataIn;

public:
  Subscriber(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};


extern "C"
{
  void SubscriberInit(RTC::Manager* manager);
};

#endif // Subscriber_H
