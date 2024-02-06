

#include <mc_observers/ObserverMacros.h>
#include <VrpnSensorObserverROS.h>

namespace vrpn_sensor_obs
{

VrpnSensorObserverROS::VrpnSensorObserverROS(const std::string & type, double dt)
: VrpnSensorObserver(type, dt), nh_(mc_rtc::ROSBridge::get_node_handle())
{
}

void VrpnSensorObserverROS::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  VrpnSensorObserver::configure(ctl, config);
  config("rostopic", rosTopic_);
  desc_ =
      fmt::format("rostopic: {} ", rosTopic_);

  rosSubscriber_ = nh_->subscribe<geometry_msgs::PoseStamped>(rosTopic_, 10, &VrpnSensorObserverROS::callbackVrpn, this);
  thread_ = std::thread(std::bind(&VrpnSensorObserverROS::rosSpinner, this));

}

void VrpnSensorObserverROS::reset(const mc_control::MCController & ctl)
{
  VrpnSensorObserver::reset(ctl);
}

void VrpnSensorObserverROS::callbackVrpn(geometry_msgs::PoseStamped msg)
{


  position_ = (Eigen::Vector3d() << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z ).finished();
  orientation_ = (Eigen::Vector4d() << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z ).finished();

  VrpnSensorObserver::poseUpdate(position_, orientation_);
}

bool VrpnSensorObserverROS::run(const mc_control::MCController & ctl)
{
  return VrpnSensorObserver::run(ctl);
}

void VrpnSensorObserverROS::rosSpinner()
{
  ros::Rate rate(200);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}


} // namespace vrpn_sensor_obs

EXPORT_OBSERVER_MODULE("VrpnSensorObserverROS", vrpn_sensor_obs::VrpnSensorObserverROS)
