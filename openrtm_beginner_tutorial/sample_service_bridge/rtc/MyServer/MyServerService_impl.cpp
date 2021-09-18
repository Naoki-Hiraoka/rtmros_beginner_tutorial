#include "MyServerService_impl.h"
#include "MyServer.h"

MyServerService_impl::MyServerService_impl()
{
}

MyServerService_impl::~MyServerService_impl()
{
}

void MyServerService_impl::setComponent(MyServer *i_component)
{
  component = i_component;
}

CORBA::Boolean MyServerService_impl::addTwoInts(CORBA::Long a, CORBA::Long b, CORBA::Long_out sum) {
  sum = CORBA::Long();
  return component->addTwoInts(a,b,sum);
}
