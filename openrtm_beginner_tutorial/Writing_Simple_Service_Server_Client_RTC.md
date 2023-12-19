# Writing Simple Service Server Client RTC

[Creating OpenRTM idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Creating_OpenRTM_idl.md)で作成したサービスと同じものを使用する. サンプルコードは[sample_service_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc)にある.
```
#include "BasicDataType.idl"

module sample_service_rtc
{
  interface MyOriginalService
  {
    long addTwoInts(in long a, in long b);

    boolean addTwoTime(in RTC::Time a, in RTC::Time b, out RTC::Time sum);

    boolean addTwoTimedDoubleSeq(in RTC::TimedDoubleSeq a, in RTC::TimedDoubleSeq b, out RTC::TimedDoubleSeq sum);

  };
};
```

## 1. Wrinting Server RTC

### 1.1 The Code

[Server.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/Server.h)
```c++
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
};


extern "C"
{
  void ServerInit(RTC::Manager* manager);
};

#endif // Server_H
```

[Server.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/Server.cpp)
```c++
#include "Server.h"
#include <iostream>

Server::Server(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ServerServicePort("service0")
{
  m_service0.setComponent(this);
}

RTC::ReturnCode_t Server::onInitialize(){
  m_ServerServicePort.registerProvider("service0", "MyOriginalService", m_service0);
  addPort(m_ServerServicePort);
  return RTC::RTC_OK;
}

CORBA::Long Server::addTwoInts(CORBA::Long a, CORBA::Long b) {
  std::cout << "Called addTwoInts: " << a << "+" << b << "=" << a+b << std::endl;
  return a+b;
}

CORBA::Boolean Server::addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time& sum) {
  sum.sec = a.sec + b.sec + (a.nsec + b.nsec)/1000000000;
  sum.nsec = (a.nsec + b.nsec)%1000000000;
  return true;
}

CORBA::Boolean Server::addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq& sum) {
  if(a.data.length() != b.data.length()) return false;
  sum.data.length(a.data.length());
  for(int i=0;i<a.data.length();i++) sum.data[i] = a.data[i] + b.data[i];
  return true;
}

static const char* Server_spec[] = {
  "implementation_id", "Server",
  "type_name",         "Server",
  "description",       "Server component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ServerInit(RTC::Manager* manager) {
        RTC::Properties profile(Server_spec);
        manager->registerFactory(profile, RTC::Create<Server>, RTC::Delete<Server>);
    }
};
```

[ServerService_impl.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerService_impl.h)
```c++
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
```

[ServerService_impl.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerService_impl.cpp)
```c++
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
```

### 1.2 The Code Explained

```c++
class Server : public RTC::DataFlowComponentBase{
```
RTコンポーネントは`RTC::DataFlowComponentBase`クラスを継承する

```c++
protected:
  ServerService_impl m_service0;
  RTC::CorbaPort m_ServerServicePort;
```
`ServerService_impl`は[idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/idl/MyServiceSample.idl)で定義されたインターフェースである. [ServerService_impl.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerService_impl.h)で定義する.

`RTC::CorbaPort`は、rosのServiceServerと同様の役割を果たす.

```c++
extern "C"
{
  void ServerInit(RTC::Manager* manager);
};
```
このプラグインをロードしたときにfactory関数を登録するための関数である

```c++
Server::Server(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ServerServicePort("service0")
{
  m_service0.setComponent(this);
}
```
`m_ServerServicePort`のコンストラクタでポート名を"service0"に指定している.

```c++
RTC::ReturnCode_t Server::onInitialize(){
  m_ServerServicePort.registerProvider("service0", "MyOriginalService", m_service0);
  addPort(m_ServerServicePort);
  return RTC::RTC_OK;
}
```
`registerProvider`で`m_ServerServicePort`を`m_service0`と対応付けしている. これによって、portからサービスが呼ばれると, `m_ServerServicePort`は`m_service0`の該当するメンバ関数を呼ぶようになる. `registerProvider`の第一引数`service0`は単なる名前であり、何を与えても実用上あまり重要ではない. `registerProvider`の第二引数にはインターフェースの型名を与える.

`addPort`でこのRTCに実際にサービスポートを生成している.

```c++
CORBA::Long Server::addTwoInts(CORBA::Long a, CORBA::Long b) {
  std::cout << "Called addTwoInts: " << a << "+" << b << "=" << a+b << std::endl;
  return a+b;
}

CORBA::Boolean Server::addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time& sum) {
  sum.sec = a.sec + b.sec + (a.nsec + b.nsec)/1000000000;
  sum.nsec = (a.nsec + b.nsec)%1000000000;
  return true;
}

CORBA::Boolean Server::addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq& sum) {
  if(a.data.length() != b.data.length()) return false;
  sum.data.length(a.data.length());
  for(int i=0;i<a.data.length();i++) sum.data[i] = a.data[i] + b.data[i];
  return true;
}
```
サービスの処理. これらの関数は[ServerService_impl.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerService_impl.h)を通じて呼ばれる.

```c++
static const char* Server_spec[] = {
  "implementation_id", "Server",
  "type_name",         "Server",
  "description",       "Server component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ServerInit(RTC::Manager* manager) {
        RTC::Properties profile(Server_spec);
        manager->registerFactory(profile, RTC::Create<Server>, RTC::Delete<Server>);
    }
};
```
ファクトリー関数を設定している. また、このRTコンポーネントのプロパティを設定している.

```c++
class ServerService_impl
  : public virtual POA_sample_service_rtc::MyOriginalService,
    public virtual PortableServer::RefCountServantBase
```
インターフェースは、idlで定義されたインターフェースのクラスと、`PortableServer::RefCountServantBase`の2つのクラスを継承する.

```c++
public:
  ServerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  virtual ~ServerService_impl();
  CORBA::Long addTwoInts(CORBA::Long a, CORBA::Long b);
  CORBA::Boolean addTwoTime(const RTC::Time& a, const RTC::Time& b, RTC::Time_out sum);
  CORBA::Boolean addTwoTimedDoubleSeq(const RTC::TimedDoubleSeq& a, const RTC::TimedDoubleSeq& b, RTC::TimedDoubleSeq_out sum);
```
`addTwoInts`, `addTwoTime`, `addTwoTimedDoubleSeq`は、idlで定義されたサービス関数をオーバーライドしている. 基本型は値渡しだが、ユーザー定義型は参照渡しとなる点に注意.

```c++
  void setComponent(Server *i_component);
private:
  Server *component;
};
```
RTコンポーネントのポインタを保持する.

```c++
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
```

```c++
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
```
portから各サービスが呼ばれると、これらの関数が実行される. ここでは、実際に処理を担うRTコンポーネントのメンバ関数を呼び出している.

返り値は、ユーザ定義型の中でも、可変長配列が含まれない場合は参照渡しとなり、含まれる場合はポインタ渡しとなる.

### 1.3 Build

CMakeLists.txtに以下を書くことで、共有ライブラリ`Server.so`が生成される.
```
include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Server SHARED Server.cpp ServerService_impl.cpp)
target_link_libraries(Server ${catkin_LIBRARIES})
set_target_properties(Server PROPERTIES PREFIX "") # libServer.so -> Server.so
add_dependencies(Server RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl
```

### 1.4 The Code (executable)
[ServerComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerComp.cpp)
```c++
#include <rtm/Manager.h>
#include "Server.h"

void MyModuleInit(RTC::Manager* manager)
{
  ServerInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("Server");

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
```
おまじないだと思っておけばよい.

### 1.5 Build (executable)
CMakeLists.txtに以下を書くことで、`ServerComp`が生成される
```
rtmbuild_add_executable(ServerComp ServerComp.cpp)
target_link_libraries(ServerComp Server)
```
先に生成した`Server.so`をリンクしている

## 2. Writing Client RTC

### 2.1 The Code
[Client.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Client/Client.h)
```c++
#ifndef Client_H
#define Client_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>

#include <rtm/idl/BasicDataType.hh>

#include "sample_service_rtc/idl/MyServiceSample.hh"

class Client : public RTC::DataFlowComponentBase{
protected:
  RTC::CorbaConsumer<sample_service_rtc::MyOriginalService> m_consumer;
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
```

[Client.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Client/Client.cpp)
```c++
#include "Client.h"
#include <iostream>

Client::Client(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ClientServicePort("service0")
{
}

RTC::ReturnCode_t Client::onInitialize(){
  m_ClientServicePort.registerConsumer("service0", "MyOriginalService", m_consumer);
  addPort(m_ClientServicePort);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Client::onExecute(RTC::UniqueId ec_id){
  if(m_consumer._ptr()){

    int a1=1;
    int b1=2;
    int sum1 = m_consumer->addTwoInts(1,2);
    std::cout << "Call addTwoInts: " << a1 << "+" << b1 << "=" << sum1 << std::endl;

    RTC::Time a2; a2.sec=1; a2.nsec=0;
    RTC::Time b2; b2.sec=2; b2.nsec=0;
    RTC::Time sum2;
    m_consumer->addTwoTime(a2,b2,sum2);

    RTC::TimedDoubleSeq a3; a3.data.length(3); for(size_t i=0;i<a3.data.length();i++) a3.data[i]=1;
    RTC::TimedDoubleSeq b3; b3.data.length(3); for(size_t i=0;i<b3.data.length();i++) b3.data[i]=1;
    RTC::TimedDoubleSeq* sum3;
    m_consumer->addTwoTimedDoubleSeq(a3,b3,sum3);
  }

  return RTC::RTC_OK;
}

static const char* Client_spec[] = {
  "implementation_id", "Client",
  "type_name",         "Client",
  "description",       "Client component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ClientInit(RTC::Manager* manager) {
        RTC::Properties profile(Client_spec);
        manager->registerFactory(profile, RTC::Create<Client>, RTC::Delete<Client>);
    }
};
```

### 2.2 The Code Explained

```c++
class Client : public RTC::DataFlowComponentBase{
```
RTコンポーネントは`RTC::DataFlowComponentBase`クラスを継承する

```c++
protected:
  RTC::CorbaConsumer<sample_service_rtc::MyOriginalService> m_consumer;
  RTC::CorbaPort m_ClientServicePort;
```
`RTC::CorbaConsumer`はインターフェースのコンシューマである.

`RTC::CorbaPort`はROSのServiceClientと同様の役割を果たす.

```c++
public:
  Client(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};
```
`RTC::DataFlowComponentBase`クラスのメンバ関数をオーバーライドすることで、独自の処理を実装できる. ここでは、コンポーネント生成時に一度だけ呼ばれる`onInitialize`と、アクティブ状態時に周期的に呼ばれる`onExecute`をオーバーライドしている.

```c++
extern "C"
{
  void ClientInit(RTC::Manager* manager);
};
```
`RTC::DataFlowComponentBase`クラスのメンバ関数をオーバーライドすることで、独自の処理を実装できる. ここでは、コンポーネント生成時に一度だけ呼ばれる`onInitialize`と、アクティブ状態時に周期的に呼ばれる`onExecute`をオーバーライドしている.

```c++
Client::Client(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_ClientServicePort("service0")
{
}
```
`m_ClientServicePort`のコンストラクタでポート名を"service0"に指定している.

```c++
RTC::ReturnCode_t Client::onInitialize(){
  m_ClientServicePort.registerConsumer("service0", "MyOriginalService", m_consumer);
  addPort(m_ClientServicePort);
  return RTC::RTC_OK;
}
```
`registerConsumer`で`m_ClientServicePort`を`m_consumer`と対応付けしている. これによって、`m_consumer`のメンバ関数を呼ぶと, portからサーバーにサービスコールするようになる. `registerConsumer`の第一引数`service0`は単なる名前であり、何を与えても実用上あまり重要ではない. `registerProvider`の第二引数にはインターフェースの型名を与える.

`addPort`で実際にサービスポートを生成している.

```c++
RTC::ReturnCode_t Client::onExecute(RTC::UniqueId ec_id){
  if(m_consumer._ptr()){
```
サーバーと接続しているかどうかを判定している

```c++
    int a1=1;
    int b1=2;
    int sum1 = m_consumer->addTwoInts(1,2);
    std::cout << "Call addTwoInts: " << a1 << "+" << b1 << "=" << sum1 << std::endl;

    RTC::Time a2; a2.sec=1; a2.nsec=0;
    RTC::Time b2; b2.sec=2; b2.nsec=0;
    RTC::Time sum2;
    m_consumer->addTwoTime(a2,b2,sum2);

    RTC::TimedDoubleSeq a3; a3.data.length(3); for(size_t i=0;i<a3.data.length();i++) a3.data[i]=1;
    RTC::TimedDoubleSeq b3; b3.data.length(3); for(size_t i=0;i<b3.data.length();i++) b3.data[i]=1;
    RTC::TimedDoubleSeq* sum3;
    m_consumer->addTwoTimedDoubleSeq(a3,b3,sum3);
  }

  return RTC::RTC_OK;
}
```
`m_consumer`のメンバ関数を呼ぶ形で、サーバーにサービスコールしている. 返り値は、ユーザ定義型の中でも、可変長配列が含まれない場合は参照渡しとなり、含まれる場合はポインタ渡しとなることに注意.

```c++
static const char* Client_spec[] = {
  "implementation_id", "Client",
  "type_name",         "Client",
  "description",       "Client component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ClientInit(RTC::Manager* manager) {
        RTC::Properties profile(Client_spec);
        manager->registerFactory(profile, RTC::Create<Client>, RTC::Delete<Client>);
    }
};
```
ファクトリー関数を設定している. また、このRTコンポーネントのプロパティを設定している.

### 2.3 Build

CMakeLists.txtに以下を書くことで、共有ライブラリ`Client.so`が生成される.
```
include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Client SHARED Client.cpp)
target_link_libraries(Client ${catkin_LIBRARIES})
set_target_properties(Client PROPERTIES PREFIX "") # libClient.so -> Client.so
add_dependencies(Client RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl
```

### 2.4 The Code (executable)
[ClientComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Client/ClientComp.cpp)
```c++
#include <rtm/Manager.h>
#include "Client.h"

void MyModuleInit(RTC::Manager* manager)
{
  ClientInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("Client");

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
```
おまじないだと思っておけばよい.

### 2.5 Build (executable)
CMakeLists.txtに以下を書くことで、`ClientComp`が生成される
```
rtmbuild_add_executable(ClientComp ClientComp.cpp)
target_link_libraries(ClientComp Client)
```
先に生成した`Client.so`をリンクしている

## 3 Build and Execute

### 3.1 Build
上記を行ったものが[sample_service_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc)にある.

以下のスクリプトでビルドできる
```
catkin build sample_service_rtc
```

### 3.2 Execute
ターミナルで次を実行
```
rtmlaunch sample_service_rtc server.launch
```
別のターミナルで次を実行
```
rtmlaunch sample_service_rtc client.launch
```
