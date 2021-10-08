#ifndef ServerSERVICESVC_IMPL_H
#define ServerSERVICESVC_IMPL_H

#include "sample_service_rtc/idl/MyServiceSample.hh"

class Server;

class ServerService_impl
  : public virtual POA_sample_service_rtc::MyOriginalService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ServerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  virtual ~ServerService_impl();
  CORBA::Long addTwoInts(CORBA::Long a, CORBA::Long b);
  CORBA::Boolean addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time_out sum);
  CORBA::Boolean addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq_out sum);

  void setComponent(Server *i_component);
private:
  Server *component;
};

#endif
