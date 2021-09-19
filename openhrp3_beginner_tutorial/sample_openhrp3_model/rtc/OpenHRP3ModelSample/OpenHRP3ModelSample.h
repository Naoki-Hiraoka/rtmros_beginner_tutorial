#ifndef OpenHRP3ModelSample_H
#define OpenHRP3ModelSample_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>

class OpenHRP3ModelSample : public RTC::DataFlowComponentBase{
protected:

public:
  OpenHRP3ModelSample(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
};


extern "C"
{
  void OpenHRP3ModelSampleInit(RTC::Manager* manager);
};

#endif // OpenHRP3ModelSample_H
