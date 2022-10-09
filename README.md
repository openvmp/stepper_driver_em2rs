# OpenVMP

[![License](./license.svg)](./LICENSE.txt)

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

![ROS/ROS2 index package for OpenVMP module: stepper driver for RS485-enabled STEPPERONLINE (DM556RS,DM882RS)](https://www.google-analytics.com/collect?v=1&tid=UA-242596187-2&cid=555&aip=1&t=event&ec=github&ea=md&dp=%2FREADME.md&dt=ROS2%20package%20for%20stepper%20driver%20RS485%20STEPPERONLINE)
