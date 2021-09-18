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
