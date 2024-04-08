Simple custom VRPN observer (using ROS)
==

Please refer to the [state observation tutorial](https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html) for futher details.

This simple observer listen to the specified ROS topic and **update the base pose of the specified robot/object** in mc_rtc.

Dependencies
------------

This package requires:
- [mc_rtc]
- ROS (tested on noetic)

(can be used with [real_pose])

  
And the following ros packages: 
- ros-${ROS_DISTRO}-mc-rtc-plugin 
- ros-${ROS_DISTRO}-mc-rtc-tools 
- ros-${ROS_DISTRO}-mc-state-observation
- ros-${ROS_DISTRO}-vrpn-client-ros



Usage
==

1. Install this project's dependencies
2. Clone this repo (oustide of the catkin WS) and install it (`cmake .. && make && make install`)

In your controller's configuration (Controller.in.yaml), add

```yaml
ObserverPipelines:
- name: ObserverName
  gui: true
  observers:
    - type: Encoder
    - type: VrpnSensorObserverROS
      update: true
      config:
        updateRobot: RobotName
        rostopic: /ros_topic_name

```

Launch the vrpn client:

```bash
roslaunch vrpn_client_ros sample.launch server:=IP_ADDRESS
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
[real_pose]: https://github.com/epfl-lasa/real_pose.git