macro(add_observer observer_name observer_SRC observer_HDR)
  add_library(${observer_name} SHARED ${observer_SRC} ${observer_HDR})
  set_target_properties(${observer_name} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX "")
  set_target_properties(${observer_name} PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_INSTALL_PREFIX})
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_observers)
  install(TARGETS ${observer_name} DESTINATION "${MC_OBSERVERS_INSTALL_PREFIX}")
endmacro()

add_observer(VrpnSensorObserver VrpnSensorObserver.cpp VrpnSensorObserver.h)
target_link_libraries(VrpnSensorObserver PUBLIC mc_rtc::mc_control)
target_include_directories(VrpnSensorObserver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)

add_observer(VrpnSensorObserverROS VrpnSensorObserverROS.cpp VrpnSensorObserverROS.h)
target_link_libraries(VrpnSensorObserverROS PUBLIC VrpnSensorObserver)
target_link_libraries(VrpnSensorObserverROS PUBLIC mc_rtc::mc_control
                                                 mc_rtc::mc_rtc_ros)
