# ROS2 package for EM2RS Series stepper motor drivers

[![License](./apache20.svg)](./LICENSE.txt)

This packages controls stepper drivers using Modbus RTU over RS485.
It is expected to support the following products:

- Leadshine
  - EM2RS-522
  - EM2RS-556
  - EM2RS-870
  - EM2RS-A882
- STEPPERONLINE
  - DM556RS
  - DM882RS

## Ready for ros2\_control

This package works with
[ros2\_control](https://github.com/ros-controls/ros2_control) via
[remote\_hardware\_interface](https://github.com/openvmp/remote_hardware_interface).

## Generic actuator interface

This package implements the `velocity` command (in terms of `ros2\_control`)
using [the generic actuator interface](https://github.com/openvmp/actuator).

```shell
$ ros2 service list
...
/openvmp/robot_EE3U/actuator/front_body_joint/set_velocity
...
```

### Generic stepper driver interface

This package implements
[the generic stepper driver interface](https://github.com/openvmp/stepper_driver/).

```shell
$ ros2 service list
...
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/param/ppr/get
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/param/ppr/set
...
```

### Custom interface

This package also exposes some device specific Modbus registers as ROS2 services.

```shell
$ ros2 service list
...
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_brake_delay_lock
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_brake_delay_release
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_brake_locking_velocity_threshold
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_control
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_cur_alarm
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_dir
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_do1
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_firmware
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_jog_acc_dec
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_jog_interval
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_jog_velocity
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_model
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_peak_current
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_ppr
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/get_version
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_brake_delay_lock
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_brake_delay_release
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_brake_locking_velocity_threshold
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_control
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_dir
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_do1
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_jog_acc_dec
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_jog_interval
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_jog_velocity
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_peak_current
/openvmp/robot_EE3U/actuator/front_body_joint/stepper/modbus/set_ppr
...
```

## Disclaimers

It has only been tested with STEPPERONLINE DM556RS so far.
