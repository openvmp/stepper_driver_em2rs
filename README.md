# OpenVMP

[![License](./license.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## ROS2 package for EM2RS Series stepper motor drivers

This packages controls stepper drivers using Modbus RTU over RS485. It is expected to support the following products:

  - Leadshine
    - EM2RS-522
    - EM2RS-556
    - EM2RS-870
    - EM2RS-A882
  - STEPPERONLINE
    - DM556RS
    - DM882RS
    

 ### Generic stepper driver interface

 This package implement [the generic stepper driver interface](https://github.com/stepper_driver/).

 ### Custom interface

 This package also exposes custom Modbus registers supported by these devices as ROS2 services.


 ### Notes

Disclaimer: it has only been tested with STEPPERONLINE DM556RS so far.

