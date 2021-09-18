# Writing Simple Publisher Subscriber RTC

サンプルコードは[sample_io_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc)にある.

## 1 Writing Publisher RTC

出力ポート"chatter"をもち、毎周期配列を出力するRTコンポーネントを作成する.

### 1.1 The Code

[Publisher.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Publisher/Publisher.h)
```c++
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
```

[Publisher.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Publisher/Publisher.cpp)
```c++
#include "Publisher.h"
#include <iostream>

Publisher::Publisher(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataOut("chatter", m_data)
{
}

RTC::ReturnCode_t Publisher::onInitialize(){
  addOutPort("chatter", m_dataOut);
  m_data.data.length(3);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Publisher::onExecute(RTC::UniqueId ec_id){
  loop++;

  coil::TimeValue coiltm(coil::gettimeofday());
  m_data.tm.sec  = coiltm.sec();
  m_data.tm.nsec = coiltm.usec() * 1000;

  for(int i=0;i<m_data.data.length();i++){
    m_data.data[i] = loop;
  }

  m_dataOut.write();

  std::cout << "Publish: " << loop << std::endl;
  return RTC::RTC_OK;
}

static const char* Publisher_spec[] = {
  "implementation_id", "Publisher",
  "type_name",         "Publisher",
  "description",       "Publisher component",
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
    void PublisherInit(RTC::Manager* manager) {
        RTC::Properties profile(Publisher_spec);
        manager->registerFactory(profile, RTC::Create<Publisher>, RTC::Delete<Publisher>);
    }
};
```

### 1.2 The Code Explained

ROSとは異なり、RTコンポーネントのプログラムにはmain関数が無く、クラスとして定義する. ([ROS2](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)と似ている)

```c++
class Publisher : public RTC::DataFlowComponentBase{
```
RTコンポーネントは`RTC::DataFlowComponentBase`クラスを継承する

```c++
protected:
  unsigned int loop;
  RTC::TimedDoubleSeq m_data;
  RTC::OutPort<RTC::TimedDoubleSeq> m_dataOut;
```
`RTC::TimedDoubleSeq`は[BasicDataType.idl](https://github.com/OpenRTM/OpenRTM-aist/blob/master/src/lib/rtm/idl/BasicDataType.idl)で定義されたデータ型である. この型を使うために、`rtm/idl/BasicDataType.hh`をincludeしている. `RTC::TimedDoubleSeq`の定義を記す.
```
struct TimedDoubleSeq
  {
     Time tm;
     sequence<double> data;
  };
```

`RTC::OutPort`はROSのPublisherと同様の役割を果たす. 送るデータ型のtemplateになっている.

```
public:
  Publisher(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};
```

`RTC::DataFlowComponentBase`クラスの下記のメンバ関数をオーバーライドすることで、独自の処理を実装できる.

[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/rtcdevelflow)より引用

| 関数名 | 説明 |
| ---- | ---- |
| onInitialize | 初期化処理、コンポーネントライフサイクルの開始時に一度だけ呼ばれる。 |
| onActivated | 非アクティブ状態からアクティブ化されるとき1度だけ呼ばれる。 |
| onExecute | アクティブ状態時に周期的に呼ばれる。 |
| onDeactivated | アクティブ状態から非アクティブ化されるとき1度だけ呼ばれる。 |
| onAborting | ERROR 状態に入る前に1度だけ呼ばれる。 |
| onReset | エラー状態からリセットされ非アクティブ状態に移行するときに1度だけ呼ばれる。 |
| onError | エラー状態にいる間周期的に呼ばれる。 |
| onFinalize | コンポーネントライフサイクルの終了時に1度だけ呼ばれる。 |
| onStateUpdate | onExecute の後毎回呼ばれる。 |
| onRateChanged | ExecutionContext の rate が変更されたとき呼ばれる。 |
| onStartup | ExecutionContext が実行を開始するとき1度だけ呼ばれる。 |
| onShutdown | ExecutionContext が実行を停止するとき1度だけ呼ばれる。 |

ここでは、コンポーネント生成時に一度だけ呼ばれる`onInitialize`と、アクティブ状態時に周期的に呼ばれる`onExecute`をオーバーライドしている.

```c++
extern "C"
{
  void PublisherInit(RTC::Manager* manager);
};
```
このプラグインをロードしたときにfactory関数を登録するための関数である

```c++
Publisher::Publisher(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataOut("chatter", m_data)
{
}
```
`m_dataOut`のコンストラクタで`m_data`に対応付けしている. これによって、`m_dataOut`は`m_data`の中身を読んでportから出力するようになる. また、ポート名を"chatter"に指定している.

```c++
RTC::ReturnCode_t Publisher::onInitialize(){
  addOutPort("chatter", m_dataOut);
  m_data.data.length(3);
  return RTC::RTC_OK;
}
```
このRTコンポーネントに実際に出力ポートを生成し、`m_dataOut`と対応付けしている.

シーケンス型は`length`関数でサイズを変更できる.

```c++
RTC::ReturnCode_t Publisher::onExecute(RTC::UniqueId ec_id){
  loop++;

  coil::TimeValue coiltm(coil::gettimeofday());
  m_data.tm.sec  = coiltm.sec();
  m_data.tm.nsec = coiltm.usec() * 1000;

  for(int i=0;i<m_data.data.length();i++){
    m_data.data[i] = loop;
  }
```
`m_data`に値をセットしている.

```c++
  m_dataOut.write();

  std::cout << "Publish: " << loop << std::endl;
  return RTC::RTC_OK;
}
```
`write`によって`m_data`の値を実際にportから出力する

```
static const char* Publisher_spec[] = {
  "implementation_id", "Publisher",
  "type_name",         "Publisher",
  "description",       "Publisher component",
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
    void PublisherInit(RTC::Manager* manager) {
        RTC::Properties profile(Publisher_spec);
        manager->registerFactory(profile, RTC::Create<Publisher>, RTC::Delete<Publisher>);
    }
};
```
ファクトリー関数を設定している. また、このRTコンポーネントのプロパティを設定している.

### 1.3 Build

CMakeLists.txtに以下を書くことで、共有ライブラリ`Publisher.so`が生成される.
```
find_package(catkin REQUIRED COMPONENTS rtmbuild)
include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Publisher SHARED Publisher.cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})
set_target_properties(Publisher PROPERTIES PREFIX "") # libPublisher.so -> Publisher.so
```

### 1.4 The Code (executable)
[PublisherComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Publisher/PublisherComp.cpp)
```
#include <rtm/Manager.h>
#include "Publisher.h"

void MyModuleInit(RTC::Manager* manager)
{
  PublisherInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("Publisher");

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
CMakeLists.txtに以下を書くことで、`PublisherComp`が生成される
```
rtmbuild_add_executable(PublisherComp PublisherComp.cpp)
target_link_libraries(PublisherComp Publisher)
```
先に生成した`Publisher.so`をリンクしている

## 2. Writing Subscriber RTC

入力ポート"listener"をもち、毎周期配列を受け取るRTコンポーネントを作成する.

### 2.1 The Code
[Subscriber.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Subscriber/Subscriber.h)
```c++
#ifndef Subscriber_H
#define Subscriber_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh> // for RTC::TimedDoubleSeq

class Subscriber : public RTC::DataFlowComponentBase{
protected:
  RTC::TimedDoubleSeq m_data;
  RTC::InPort<RTC::TimedDoubleSeq> m_dataIn;

public:
  Subscriber(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};


extern "C"
{
  void SubscriberInit(RTC::Manager* manager);
};

#endif // Subscriber_H
```

[Subscriber.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Subscriber/Subscriber.cpp)
```c++
#include "Subscriber.h"

Subscriber::Subscriber(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataIn("listener", m_data)
{
}

RTC::ReturnCode_t Subscriber::onInitialize(){
  addInPort("listener", m_dataIn);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Subscriber::onExecute(RTC::UniqueId ec_id){
  if(m_dataIn.isNew()){
    m_dataIn.read();

    std::cout << "Subscribe: ";
    for(int i=0;i<m_data.data.length();i++){
      std::cout << m_data.data[i] << " ";
    }
    std::cout << std::endl;
  }

  return RTC::RTC_OK;
}

static const char* Subscriber_spec[] = {
  "implementation_id", "Subscriber",
  "type_name",         "Subscriber",
  "description",       "Subscriber component",
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
    void SubscriberInit(RTC::Manager* manager) {
        RTC::Properties profile(Subscriber_spec);
        manager->registerFactory(profile, RTC::Create<Subscriber>, RTC::Delete<Subscriber>);
    }
};
```

### 2.2 The Code Explained

Publisherと同様である.

```c++
class Subscriber : public RTC::DataFlowComponentBase{
```
RTコンポーネントは`RTC::DataFlowComponentBase`クラスを継承する

```c++
protected:
  RTC::TimedDoubleSeq m_data;
  RTC::InPort<RTC::TimedDoubleSeq> m_dataIn;
```
`RTC::InPort`はROSのSubsciberと同様の役割を果たす. 受け取るデータ型のtemplateになっている.

```
public:
  Subscriber(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};
```

`RTC::DataFlowComponentBase`クラスのメンバ関数をオーバーライドすることで、独自の処理を実装できる. ここでは、コンポーネント生成時に一度だけ呼ばれる`onInitialize`と、アクティブ状態時に周期的に呼ばれる`onExecute`をオーバーライドしている.

```c++
extern "C"
{
  void SubscriberInit(RTC::Manager* manager);
};
```
このプラグインをロードしたときにfactory関数を登録するための関数であるw

```c++
Subscriber::Subscriber(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataIn("listener", m_data)
{
}
```
`m_dataIn`のコンストラクタで`m_data`に対応付けしている. これによって、`m_dataIn`はportから受け取ったデータを`m_data`に書き込むようになる.

```c++
RTC::ReturnCode_t Subscriber::onInitialize(){
  addInPort("listener", m_dataIn);
  return RTC::RTC_OK;
}
```
このRTコンポーネントに"listener"という名前の入力ポートを生成し、`m_dataIn`と対応付けしている.

```c++
RTC::ReturnCode_t Subscriber::onExecute(RTC::UniqueId ec_id){
  if(m_dataIn.isNew()){
    m_dataIn.read();
```
`isNew`はポートに新しいデータが届いているかどうかを返す. `read`を呼ぶとポートに届いたデータを実際に`m_data`に書き込む.

```c++
    std::cout << "Subscribe: ";
    for(int i=0;i<m_data.data.length();i++){
      std::cout << m_data.data[i] << " ";
    }
    std::cout << std::endl;
  }

  return RTC::RTC_OK;
}
```
届いた`m_data`の値を読んでいる

```
static const char* Subscriber_spec[] = {
  "implementation_id", "Subscriber",
  "type_name",         "Subscriber",
  "description",       "Subscriber component",
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
    void SubscriberInit(RTC::Manager* manager) {
        RTC::Properties profile(Subscriber_spec);
        manager->registerFactory(profile, RTC::Create<Subscriber>, RTC::Delete<Subscriber>);
    }
};
```
ファクトリー関数を設定している. また、このRTコンポーネントのプロパティを設定している.

### 2.3 Build

CMakeLists.txtに以下を書くことで、共有ライブラリ`Subscriber.so`が生成される.
```
find_package(catkin REQUIRED COMPONENTS rtmbuild)
include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(Subscriber SHARED Subscriber.cpp)
target_link_libraries(Subscriber ${catkin_LIBRARIES})
set_target_properties(Subscriber PROPERTIES PREFIX "") # libSubscriber.so -> Subscriber.so
```

### 2.4 The Code (executable)
[SubscriberComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc/rtc/Publisher/SubscriberComp.cpp)
```c++
#include <rtm/Manager.h>
#include "Subscriber.h"

void MyModuleInit(RTC::Manager* manager)
{
  SubscriberInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("Subscriber");

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
CMakeLists.txtに以下を書くことで、`SubscriberComp`が生成される
```
rtmbuild_add_executable(SubscriberComp SubscriberComp.cpp)
target_link_libraries(SubscriberComp Subscriber)
```
先に生成した`Subscriber.so`をリンクしている

## 3 Build and Execute

### 3.1 Build
上記を行ったものが[sample_io_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc)にある.

以下のスクリプトでビルドできる
```
catkin build sample_io_rtc
```

### 3.2 Execute
ターミナルで次を実行
```
rtmlaunch sample_io_rtc subscriber.launch
```
別のターミナルで次を実行
```
rtmlaunch sample_io_rtc publisher.launch
```