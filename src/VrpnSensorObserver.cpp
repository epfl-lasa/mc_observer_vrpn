#include <mc_observers/ObserverMacros.h>
#include <VrpnSensorObserver.h>

namespace vrpn_sensor_obs
{

VrpnSensorObserver::VrpnSensorObserver(const std::string &type, double dt) : mc_observers::Observer(type, dt) {}

void VrpnSensorObserver::configure (const mc_control::MCController &ctl, const mc_rtc::Configuration &config)
{
  mc_rtc::log::info("[VrpnSensorObserver] - Configure ");
  robotName_ = config("updateRobot", ctl.robot().name());
  // robotBodyName_ = config("robot_body_name", robot(ctl).mb().body(0).name());
}

void VrpnSensorObserver::reset(const mc_control::MCController &ctl)
{
  mc_rtc::log::info("[VrpnSensorObserver] - reset ");
  run(ctl);
}

bool VrpnSensorObserver::run(const mc_control::MCController &ctl)
{
  return true;
}

void VrpnSensorObserver::update(mc_control::MCController &ctl)
{
  // mc_rtc::log::info("[VrpnSensorObserver] - update ");
  ctl.realRobot(robotName_).posW(robotPos_);
}

void VrpnSensorObserver::poseUpdate(Eigen::Vector3d position, Eigen::Vector4d orientation){
  robotPos_.translation() = position;
  q_.w() = orientation[0];
  q_.x() = orientation[1];
  q_.y() = orientation[2];
  q_.z() = orientation[3];
  robotPos_.rotation() = q_.normalized().toRotationMatrix(); //TODO CHECK IF CORRECT
}

} // namespace vrpn_sensor_obs

EXPORT_OBSERVER_MODULE("VrpnSensorObserver", vrpn_sensor_obs::VrpnSensorObserver)
