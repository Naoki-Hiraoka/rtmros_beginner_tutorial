#ifndef Publisher_H
#define Publisher_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>

#include <rtm/idl/BasicDataType.hh> // for RTC::TimedDoubleSeq

class Publisher : public RTC::DataFlowComponentBase{
protected:
  unsigned int loop;
  RTC::TimedDoubleSeq m_data;
  RTC::OutPort<RTC::TimedDoubleSeq> m_dataOut;

public:
  Publisher(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};


extern "C"
{
  void PublisherInit(RTC::Manager* manager);
};

#endif // Publisher_H
