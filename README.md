Simple custom VRPN observer (ROS)
==

Please refer to the [state observation tutorial](https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html) for futher details.

This simple observer listen to the specified ROS topic and update the base pose of the specified robot in mc_rtc.

Dependencies
------------

This package requires:
- [mc_rtc]
- [mc_state_observation] (--recurse-submodules when git clone)
  - [gram_savitzky_golay] (--recurse-submodules when git clone)

  
And the following ros packages: 
- ros-ROS_DISTRO-mc-rtc-plugin 
- ros-ROS_DISTRO-mc-rtc-tools 
- ros-ROS_DISTRO-mc-state-observation
- ros-${ROS_DISTRO}-vrpn-client-ros



Usage
==

1. Install this project's dependencies
2. Install this project (`cmake .. `/`make`/`make install`)

In your controller's configuration, add

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


[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
[mc_state_observation]: https://github.com/arntanguy/gram_savitzky_golay.git
[gram_savitzky_golay]: https://github.com/arntanguy/gram_savitzky_golay.git
