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
  tf2::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tf2::Matrix3x3(quat).getRPY(m_data.data.orientation.r,m_data.data.orientation.p,m_data.data.orientation.y);
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
