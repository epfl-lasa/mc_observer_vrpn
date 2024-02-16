#pragma once

#include <VrpnSensorObserver.h>

#include <mc_rtc_ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <thread>

namespace vrpn_sensor_obs
{

struct VrpnSensorObserverROS : public VrpnSensorObserver
{
  VrpnSensorObserverROS(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void callbackVrpn(geometry_msgs::Pose msg);

protected:
  std::string rosTopic_ = "/ft_sensor_ns/netft_data";

  mc_rtc::NodeHandlePtr nh_ = nullptr;
  ros::Subscriber rosSubscriber_;
  std::thread thread_;

  void rosSpinner();

  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d orientation_ = Eigen::Vector4d::Zero();
};

} // namespace vrpn_sensor_obs
