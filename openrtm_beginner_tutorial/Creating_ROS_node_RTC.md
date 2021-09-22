# Creating ROS node RTC

## 1. ROS node RTC

ROSのノードとしても機能するRTコンポーネントの作成法を述べる.

RTコンポーネントの作法に従ってROSのnodeを書けば良いだけで、特に特別なことは無い.

サンプルコードは[sample_ros_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc)にある.

## 2. The Code

ROSのトピック"/pose"を受け取り、その値を出力ポート"pose"に出力するRTコンポーネントを作成する.

[MyRosRtc.h](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc/rtc/MyRosRtc/MyRosRtc.h)
```c++
#ifndef MyRosRtc_H
#define MyRosRtc_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>

#include <rtm/idl/ExtendedDataTypes.hh> // for RTC::TimedPose3D

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class MyRosRtc : public RTC::DataFlowComponentBase{
protected:
  ros::Subscriber sub;
  RTC::TimedPose3D m_data;
  RTC::OutPort<RTC::TimedPose3D> m_dataOut;
public:
  MyRosRtc(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};


extern "C"
{
  void MyRosRtcInit(RTC::Manager* manager);
};

#endif // MyRosRtc_H
```

[MyRosRtc.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc/rtc/MyRosRtc/MyRosRtc.cpp)
```c++
#include "MyRosRtc.h"
#include <tf2/utils.h>

MyRosRtc::MyRosRtc(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_dataOut("pose", m_data)
{
}

RTC::ReturnCode_t MyRosRtc::onInitialize(){
  addOutPort("pose", m_dataOut);

  ros::NodeHandle n;
  sub = n.subscribe("pose", 1, &MyRosRtc::poseCallback, this);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t MyRosRtc::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  return RTC::RTC_OK;
}

void MyRosRtc::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  m_data.tm.sec = msg->header.stamp.sec;
  m_data.tm.nsec = msg->header.stamp.nsec;
  m_data.data.position.x = msg->pose.position.x;
  m_data.data.position.y = msg->pose.position.y;
  m_data.data.position.z = msg->pose.position.z;
  tf2::getEulerYPR(msg->pose.orientation,m_data.data.orientation.y,m_data.data.orientation.p,m_data.data.orientation.r);
  m_dataOut.write();
}

static const char* MyRosRtc_spec[] = {
  "implementation_id", "MyRosRtc",
  "type_name",         "MyRosRtc",
  "description",       "MyRosRtc component",
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
    void MyRosRtcInit(RTC::Manager* manager) {
        RTC::Properties profile(MyRosRtc_spec);
        manager->registerFactory(profile, RTC::Create<MyRosRtc>, RTC::Delete<MyRosRtc>);
    }
};
```

[MyRosRtcComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc/rtc/MyRosRtc/MyRosRtcComp.cpp)
```c++
#include <rtm/Manager.h>
#include "MyRosRtc.h"

void MyModuleInit(RTC::Manager* manager)
{
  MyRosRtcInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("MyRosRtc");

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);
  ros::init(argc, argv, "my_ros_rtc", ros::init_options::NoSigintHandler);

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

[MyRosRtc.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc/rtc/MyRosRtc/MyRosRtc.cpp)
```c++
RTC::ReturnCode_t MyRosRtc::onInitialize(){
  addOutPort("pose", m_dataOut);

  ros::NodeHandle n;
  sub = n.subscribe("pose", 1, &MyRosRtc::poseCallback, this);

  return RTC::RTC_OK;
}
```

`pose`という名前のOpenRTMの出力ポートを作成している.

ROSのノードハンドルを作成し、ROSのトピック`/pose`をサブスクライブしている.

```c++
void MyRosRtc::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  m_data.tm.sec = msg->header.stamp.sec;
  m_data.tm.nsec = msg->header.stamp.nsec;
  m_data.data.position.x = msg->pose.position.x;
  m_data.data.position.y = msg->pose.position.y;
  m_data.data.position.z = msg->pose.position.z;
  tf2::getEulerYPR(msg->pose.orientation,m_data.data.orientation.y,m_data.data.orientation.p,m_data.data.orientation.r);
  m_dataOut.write();
}
```
ROSのトピックが届いたら, それをOpenRTMのportに出力している.

[MyRosRtcComp.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc/rtc/MyRosRtc/MyRosRtcComp.cpp)
```c++
int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);
  ros::init(argc, argv, "my_ros_rtc", ros::init_options::NoSigintHandler);
```
`ros::init`はコンポーネントの外で呼んでいる. これは、`argc` `argv`が取得しやすいからという理由に加えて、`ros::init`はプロセス中で1回しか呼んでは行けないため, 複数のRTCのモジュールを同一プロセスにロードして走らせる場合に備えてRTコンポーネントの外で`ros::init`を呼んでおいた方が安全である、という理由がある.

## Build

CMakeLists.txtに以下のように書くことで、`MyRosRtc.so`と`MyRosRtcComp`が生成される.
```c++
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp geometry_msgs tf2)

include_directories(${catkin_INCLUDE_DIRS})
rtmbuild_add_library(MyRosRtc SHARED MyRosRtc.cpp)
target_link_libraries(MyRosRtc ${catkin_LIBRARIES})
set_target_properties(MyRosRtc PROPERTIES PREFIX "") # libMyRosRtc.so -> MyRosRtc.so

rtmbuild_add_executable(MyRosRtcComp MyRosRtcComp.cpp)
target_link_libraries(MyRosRtcComp MyRosRtc)
```

## 4. Run Sample

以上を作成したものが[sample_ros_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ros_rtc)にある.

次でビルドできる.
```bash
catkin build sample_ros_rtc
```

ターミナルで次を実行する. 作成したMyRosRtcが立ち上がる.
```bash
rtmlaunch sample_ros_rtc myrosrtc.launch
```

別のターミナルで,ROSのトピック`/pose`にデータをPublishしてみる.
```bash
rostopic pub /pose geometry_msgs/PoseStamped "[0, 0, \"\"]" "[[1, 2, 3], [0, 0, 0, 1]]" -r 1
```

MyRosRtcの出力port`pose`に、ROSのトピックのデータが出力されていることが分かる
```bash
$ rtprint localhost:15005/MyRosRtc0.rtc:pose -t 10
comp_args: rtprint_reader0?exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=100.0
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
[0.000000000] RTC.Pose3D(position=RTC.Point3D(x=1.0, y=2.0, z=3.0), orientation=RTC.Orientation3D(r=0.0, p=-0.0, y=0.0))
```
