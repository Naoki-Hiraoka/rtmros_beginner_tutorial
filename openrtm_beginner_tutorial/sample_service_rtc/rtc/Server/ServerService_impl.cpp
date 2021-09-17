#include "ServerService_impl.h"
#include "Server.h"

ServerService_impl::ServerService_impl()
{
}

ServerService_impl::~ServerService_impl()
{
}

void ServerService_impl::setComponent(Server *i_component)
{
  component = i_component;
}

CORBA::Long ServerService_impl::addTwoInts(CORBA::Long a, CORBA::Long b) {
  return component->addTwoInts(a,b);
}

CORBA::Boolean ServerService_impl::addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time_out sum) {
  sum = RTC::Time();
  return component->addTwoTime(a,b,sum);
}
CORBA::Boolean ServerService_impl::addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq_out sum) {
  sum = new RTC::TimedDoubleSeq();
  return component->addTwoTimedDoubleSeq(a,b,*sum);
}

