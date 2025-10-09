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

    boolean addTwoString(in string a, in string b, out string sum);

    boolean addTwoTimedString(in RTC::TimedString a, in RTC::TimedString b, out RTC::TimedString sum);
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
  CORBA::Boolean addTwoString(const char*& a, const char*& b, char*& sum);
  CORBA::Boolean addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString& sum);
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

CORBA::Boolean Server::addTwoString(const char*& a, const char*& b, char*& sum) {
  sum = CORBA::string_alloc(strlen(a)+strlen(b));
  strcpy(sum,a);
  strcpy(sum+strlen(a),b);
  return true;
}

int loop = 0;
CORBA::Boolean Server::addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString& sum) {
  // どちらの方法も可. 交互に実行するサンプル
  if(loop % 2 == 0){
    sum.data = CORBA::string_alloc(strlen(a.data)+strlen(b.data));
    strcpy(sum.data,a.data);
    strcpy(sum.data+strlen(a.data),b.data);
  }else{
    std::string a_(a.data);
    std::string b_(b.data);
    std::string sum_ = a_ + b_;
    sum.data = sum_.c_str(); // const char*からのコピー. (char*からのコピーは不可)
  }
  loop++;
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
  CORBA::Boolean addTwoString(const char* a, const char* b, CORBA::String_out sum);
  CORBA::Boolean addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString_out sum);

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

CORBA::Boolean ServerService_impl::addTwoString(const char* a, const char* b, CORBA::String_out sum) {
  return component->addTwoString(a,b,sum);
}

CORBA::Boolean ServerService_impl::addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString_out sum) {
  sum = new RTC::TimedString();
  return component->addTwoTimedString(a,b,*sum);
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
`registerProvider`で`m_ServerServicePort`を`m_service0`と対応付けしている. これによって、portからサービスが呼ばれると, `m_ServerServicePort`は`m_service0`の該当するメンバ関数を呼ぶようになる. `registerProvider`の第一引数`service0`はインスタンス名、第二引数にはサービスのインターフェースの型名であり、これらはサーバー側とクライアント側で同じである必要がある。

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

CORBA::Boolean Server::addTwoString(const char*& a, const char*& b, char*& sum) {
  sum = CORBA::string_alloc(strlen(a)+strlen(b));
  strcpy(sum,a);
  strcpy(sum+strlen(a),b);
  return true;
}

int loop = 0;
CORBA::Boolean Server::addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString& sum) {
  // どちらの方法も可. 交互に実行するサンプル
  if(loop % 2 == 0){
    sum.data = CORBA::string_alloc(strlen(a.data)+strlen(b.data));
    strcpy(sum.data,a.data);
    strcpy(sum.data+strlen(a.data),b.data);
  }else{
    std::string a_(a.data);
    std::string b_(b.data);
    std::string sum_ = a_ + b_;
    sum.data = sum_.c_str(); // const char*からのコピー. (char*からのコピーは不可)
  }
  loop++;
  return true;
}
```
サービスの処理. これらの関数は[ServerService_impl.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_rtc/rtc/Server/ServerService_impl.h)を通じて呼ばれる.

文字列型は、構造体のメンバである場合に限り`const char*`からのコピーが可能である.

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
  CORBA::Boolean addTwoString(const char* a, const char* b, CORBA::String_out sum);
  CORBA::Boolean addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString_out sum);
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

CORBA::Boolean ServerService_impl::addTwoString(const char* a, const char* b, CORBA::String_out sum) {
  return component->addTwoString(a,b,sum);
}

CORBA::Boolean ServerService_impl::addTwoTimedString(const RTC::TimedString& a, const RTC::TimedString& b, RTC::TimedString_out sum) {
  sum = new RTC::TimedString();
  return component->addTwoTimedString(a,b,*sum);
}
```
portから各サービスが呼ばれると、これらの関数が実行される. ここでは、実際に処理を担うRTコンポーネントのメンバ関数を呼び出している.

返り値は、ユーザ定義型の中でも、可変長配列が含まれない場合は参照渡しとなり、含まれる場合はポインタ渡しとなる.

ポインタ渡しの場合、領域は自動的に解放されるため、`new`で獲得したポインタの領域を`delete`で開放したり文字列の領域を開放したりする必要はない.

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
  if(!m_consumer._ptr()->_is_nil()){

    int a1=1;
    int b1=2;
    int sum1 = m_consumer->addTwoInts(1,2);
    std::cout << "Call addTwoInts: " << a1 << "+" << b1 << "=" << sum1 << std::endl;

    RTC::Time a2; a2.sec=1; a2.nsec=0;
    RTC::Time b2; b2.sec=2; b2.nsec=0;
    RTC::Time sum2;
    m_consumer->addTwoTime(a2,b2,sum2);
    std::cout << "Call addTwoTime: " << a2.sec << "+" << b2.sec << "=" << sum2.sec << std::endl;

    RTC::TimedDoubleSeq a3; a3.data.length(3); for(size_t i=0;i<a3.data.length();i++) a3.data[i]=1;
    RTC::TimedDoubleSeq b3; b3.data.length(3); for(size_t i=0;i<b3.data.length();i++) b3.data[i]=1;
    RTC::TimedDoubleSeq* sum3;
    m_consumer->addTwoTimedDoubleSeq(a3,b3,sum3);
    std::cout << "Call addTwoTimedDoubleSeq: " << a3.data[0] << "," << a3.data[1] << "," << a3.data[2] << " + " << b3.data[0] << "," << b3.data[1] << "," << b3.data[2] << " = "  << sum3->data[0] << "," << sum3->data[1] << "," << sum3->data[2] << std::endl;
    delete sum3;

    std::string a4 = "abcde";
    std::string b4 = "FGHIJ";
    char* sum4;
    m_consumer->addTwoString(a4.c_str(),b4.c_str(),sum4);
    std::cout << "Call addTwoString: " << a4 << " + " << b4 << " = " << sum4 << std::endl;
    CORBA::string_free(sum4);

    RTC::TimedString a5; a5.data = "abcde";
    RTC::TimedString b5; b5.data = "FGHIJ";
    RTC::TimedString* sum5;
    m_consumer->addTwoTimedString(a5,b5,sum5);
    std::cout << "Call addTwoTimedString: " << a5.data << " + " << b5.data << " = " << sum5->data << std::endl;
    delete sum5;
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
`registerConsumer`で`m_ClientServicePort`を`m_consumer`と対応付けしている. これによって、`m_consumer`のメンバ関数を呼ぶと, portからサーバーにサービスコールするようになる. `registerConsumer`の第一引数`service0`はインスタンス名、第二引数にはサービスのインターフェースの型名であり、これらはサーバー側とクライアント側で同じである必要がある。

`addPort`で実際にサービスポートを生成している.

```c++
RTC::ReturnCode_t Client::onExecute(RTC::UniqueId ec_id){
  if(!m_consumer._ptr()->_is_nil()){
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
    std::cout << "Call addTwoTime: " << a2.sec << "+" << b2.sec << "=" << sum2.sec << std::endl;

    RTC::TimedDoubleSeq a3; a3.data.length(3); for(size_t i=0;i<a3.data.length();i++) a3.data[i]=1;
    RTC::TimedDoubleSeq b3; b3.data.length(3); for(size_t i=0;i<b3.data.length();i++) b3.data[i]=1;
    RTC::TimedDoubleSeq* sum3;
    m_consumer->addTwoTimedDoubleSeq(a3,b3,sum3);
    std::cout << "Call addTwoTimedDoubleSeq: " << a3.data[0] << "," << a3.data[1] << "," << a3.data[2] << " + " << b3.data[0] << "," << b3.data[1] << "," << b3.data[2] << " = "  << sum3->data[0] << "," << sum3->data[1] << "," << sum3->data[2] << std::endl;
    delete sum3;

    std::string a4 = "abcde";
    std::string b4 = "FGHIJ";
    char* sum4;
    m_consumer->addTwoString(a4.c_str(),b4.c_str(),sum4);
    std::cout << "Call addTwoString: " << a4 << " + " << b4 << " = " << sum4 << std::endl;
    CORBA::string_free(sum4);

    RTC::TimedString a5; a5.data = "abcde";
    RTC::TimedString b5; b5.data = "FGHIJ";
    RTC::TimedString* sum5;
    m_consumer->addTwoTimedString(a5,b5,sum5);
    std::cout << "Call addTwoTimedString: " << a5.data << " + " << b5.data << " = " << sum5->data << std::endl;
    delete sum5;
  }

  return RTC::RTC_OK;
}
```
`m_consumer`のメンバ関数を呼ぶ形で、サーバーにサービスコールしている. 返り値は、ユーザ定義型の中でも、可変長配列が含まれない場合は参照渡しとなり、含まれる場合はポインタ渡しとなることに注意.

ポインタ渡しの場合所有権が移るため、`delete`等を用いて領域を開放する必要がある.

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
