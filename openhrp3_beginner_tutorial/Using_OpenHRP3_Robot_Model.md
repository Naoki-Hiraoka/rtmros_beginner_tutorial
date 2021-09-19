# Using OpenHRP3 Robot Model

## 1. OpenHRP3 Robot Model

OpenHRP3は、VRMLモデルを読み込んで、運動学・動力学計算を行うことができる.

C++のプログラムで, VRMLモデルを読み込む方法を説明する.

サンプルコードは[sample_openhrp3_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model)にある

## 2. ModelLoader

あらかじめ`ModelLoader`と呼ばれるモデルを読み込む専用のプロセスを立ち上げておき, 各プログラムはモデルを読み込みたいときに`ModelLoader`にOpenRTMで要求を送り,実際の読み込みは`ModelLoader`が行い,読み込み結果を受け取る.

```bash
rosrun openhrp3 openhrp-model-loader -ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService
```

上記コマンドでModelLoaderを起動できる.

ネームサーバーのportを引数で指定する(この例では15005). 事前にネームサーバーを立ち上げておくこと.

## 2. The Code

ここでは、`/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl`にあるSampleRobotモデルを読み込むコードで説明する. ROSのDISTROが違う場合にはパスを修正すること.

[OpenHRP3ModelSample.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model/rtc/OpenHRP3ModelSample/OpenHRP3ModelSample.h)
```c++
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
```

[OpenHRP3ModelSample.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model/rtc/OpenHRP3ModelSample/OpenHRP3ModelSample.cpp)
```c++
#include "OpenHRP3ModelSample.h"

#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

#include <iostream>

OpenHRP3ModelSample::OpenHRP3ModelSample(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t OpenHRP3ModelSample::onInitialize(){
  hrp::BodyPtr robot = hrp::BodyPtr(new hrp::Body());

  // load model from modelloader
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0) comPos = nameServer.length();
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!hrp::loadBodyFromModelLoader(robot, "/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl",
                                    CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                    )){
    std::cerr << "failed to load model" << std::endl;
    return RTC::RTC_ERROR;
  }

  // print robot names
  std::cout << "loaded: " << robot->name() << std::endl;
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name << std::endl;
  }

  return RTC::RTC_OK;
}


static const char* OpenHRP3ModelSample_spec[] = {
  "implementation_id", "OpenHRP3ModelSample",
  "type_name",         "OpenHRP3ModelSample",
  "description",       "OpenHRP3ModelSample component",
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
    void OpenHRP3ModelSampleInit(RTC::Manager* manager) {
        RTC::Properties profile(OpenHRP3ModelSample_spec);
        manager->registerFactory(profile, RTC::Create<OpenHRP3ModelSample>, RTC::Delete<OpenHRP3ModelSample>);
    }
};
```

[OpenHRP3ModelSampleComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model/rtc/OpenHRP3ModelSample/OpenHRP3ModelSampleComp.cpp)
```c++
#include <rtm/Manager.h>
#include "OpenHRP3ModelSample.h"

void MyModuleInit(RTC::Manager* manager)
{
  OpenHRP3ModelSampleInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("OpenHRP3ModelSample");

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

## 3. The Code Explained

`ModelLoader`との通信にOpenRTMを使用するため、RTコンポーネントとしてプログラムを書く必要がある.

```c++
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
```
`ModelLoader`と通信するために必要なファイル郡

```c++
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
```
ロボットモデルのクラスが定義された必要なファイル郡

```c++
RTC::ReturnCode_t OpenHRP3ModelSample::onInitialize(){
  hrp::BodyPtr robot = hrp::BodyPtr(new hrp::Body());
```
`hrp::Body`クラスがロボットモデルに対応したクラスである. `hrp::BodyPtr`クラスはc++のsharedポインタのような役割を果たすが、c++のsharedポインタと比べて、生のポインタで渡してもオブジェクトカウントが正しく増減するという性質を持つ.

```c++
  // load model from modelloader
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0) comPos = nameServer.length();
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!hrp::loadBodyFromModelLoader(robot, "/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl",
                                    CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                    )){
    std::cerr << "failed to load model" << std::endl;
    return RTC::RTC_ERROR;
  }
```
この部分で`ModelLoader`に要求を送ってロボットモデル(`/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl`)を読み込み、変数`robot`にセットしている.

```c++
  // print robot names
  std::cout << "loaded: " << robot->name() << std::endl;
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name << std::endl;
  }
```
読み込んだロボットの名前と、関節名を表示させている.

## 4. Build Code

CMakeLists.txtに以下を記述する.
```
find_package(catkin REQUIRED COMPONENTS rtmbuild)
rtmbuild_init()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(OpenHRP3ModelSample SHARED OpenHRP3ModelSample.cpp)
target_link_libraries(OpenHRP3ModelSample ${catkin_LIBRARIES})
set_target_properties(OpenHRP3ModelSample PROPERTIES PREFIX "") # libOpenHRP3ModelSample.so -> OpenHRP3ModelSample.so

rtmbuild_add_executable(OpenHRP3ModelSampleComp OpenHRP3ModelSampleComp.cpp)
target_link_libraries(OpenHRP3ModelSampleComp OpenHRP3ModelSample)
```

OpenHRP3のライブラリ群は、`rtmbuild`の中で自動的にリンクされる.

## 5. Run Example

[sample.launch](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model/launch/sample.launch)
```xml
<launch>
  <node name="modelloader" pkg="openhrp3" type="openhrp-model-loader"
        args="-ORBInitRef NameService=corbaloc:iiop:localhost:15005/NameService" output="screen"/>

  <node name="openhrp3_model_sample" pkg="sample_openhrp3_model" type="OpenHRP3ModelSampleComp" output="screen"
        args='-o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" '/>

  <!-- BEGIN:openrtm connection -->
  <node name="rtmlaunch_py" pkg="openrtm_tools" type="rtmlaunch.py"
        args="$(find sample_openhrp3_model)/launch/sample.launch" />
  <rtactivate component="OpenHRP3ModelSample0.rtc" />
  <!-- END:openrtm connection -->
</launch>
```

上記を作成したものが[sample_openhrp3_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openhrp3_beginner_tutorial/sample_openhrp3_model)にある。

ビルド
```bash
catkin build sample_openhrp3_model
```

実行
```bash
rtmlaunch sample_openhrp3_model sample.launch
```

出力(抜粋)
```
loaded: SampleRobot
RLEG_HIP_R
RLEG_HIP_P
RLEG_HIP_Y
RLEG_KNEE
RLEG_ANKLE_P
RLEG_ANKLE_R
RARM_SHOULDER_P
RARM_SHOULDER_R
RARM_SHOULDER_Y
RARM_ELBOW
RARM_WRIST_Y
RARM_WRIST_P
RARM_WRIST_R
LLEG_HIP_R
LLEG_HIP_P
LLEG_HIP_Y
LLEG_KNEE
LLEG_ANKLE_P
LLEG_ANKLE_R
LARM_SHOULDER_P
LARM_SHOULDER_R
LARM_SHOULDER_Y
LARM_ELBOW
LARM_WRIST_Y
LARM_WRIST_P
LARM_WRIST_R
WAIST_P
WAIST_R
CHEST
```
