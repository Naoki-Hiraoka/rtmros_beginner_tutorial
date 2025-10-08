#ifndef Server_H
#define Server_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>

#include <rtm/idl/BasicDataType.hh>

#include "ServerService_impl.h"

class Server : public RTC::DataFlowComponentBase{
protected:
  ServerService_impl m_service0;
  RTC::CorbaPort m_ServerServicePort;

public:
  Server(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();

  CORBA::Long addTwoInts(CORBA::Long a, CORBA::Long b);
  CORBA::Boolean addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time& sum);
  CORBA::Boolean addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq& sum);
  CORBA::Boolean addTwoString(const char*& a, const char*& b, char*& sum);
  CORBA::Boolean addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString& sum);
};


extern "C"
{
  void ServerInit(RTC::Manager* manager);
};

#endif // Server_H
