# OpenVMP

[![License](./apache20.svg)](./LICENSE.txt)

This package is a part of
[the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal.
It's usable independently from the rest of OpenVMP if needed.

## ROS2 package for EM2RS Series stepper motor drivers

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

### Generic stepper driver interface

This package implements [the generic stepper driver interface](https://github.com/openvmp/stepper_driver/).

### Custom interface

This package also exposes some Modbus registers directly as ROS2 services.

## Disclaimers

It has only been tested with STEPPERONLINE DM556RS so far.
