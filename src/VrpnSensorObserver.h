#pragma once

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>

namespace vrpn_sensor_obs
{

struct VrpnSensorObserver : public mc_observers::Observer
{

  VrpnSensorObserver(const std::string &type, double dt);

  void configure (const mc_control::MCController &ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController &ctl) override;

  bool run(const mc_control::MCController &ctl) override;

  void update(mc_control::MCController &ctl) override;

  void poseUpdate(Eigen::Vector3d position, Eigen::Vector4d orientation);


protected:
  std::string robotName_ = "";
  std::string robotBodyName_ = "";

  Eigen::Quaterniond q_;
  sva::PTransformd robotPos_ = sva::PTransformd(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
};

} // namespace vrpn_sensor_obs
