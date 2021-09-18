#ifndef MyServer_H
#define MyServer_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>

#include <rtm/idl/BasicDataType.hh>

#include "MyServerService_impl.h"

class MyServer : public RTC::DataFlowComponentBase{
protected:
  MyServerService_impl m_service0;
  RTC::CorbaPort m_MyServerServicePort;

public:
  MyServer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();

  CORBA::Boolean addTwoInts(CORBA::Long a, CORBA::Long b, CORBA::Long& sum);
};


extern "C"
{
  void MyServerInit(RTC::Manager* manager);
};

#endif // MyServer_H
