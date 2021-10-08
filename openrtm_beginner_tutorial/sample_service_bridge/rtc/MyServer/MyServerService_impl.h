#ifndef MyServerSERVICESVC_IMPL_H
#define MyServerSERVICESVC_IMPL_H

#include "sample_service_bridge/idl/MyBridgeService.hh"

class MyServer;

class MyServerService_impl
  : public virtual POA_sample_service_bridge::MyBridgeService,
    public virtual PortableServer::RefCountServantBase
{
public:
  MyServerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  virtual ~MyServerService_impl();
  CORBA::Boolean addTwoInts(CORBA::Long a, CORBA::Long b, CORBA::Long_out sum) override;

  void setComponent(MyServer *i_component);
private:
  MyServer *component;
};

#endif
