/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
 */

#ifndef OPENVMP_SD_RS485_SO_NODE_H
#define OPENVMP_SD_RS485_SO_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/port.hpp"
#include "stepper_driver_rs485_so/interface.hpp"

namespace stepper_driver_rs485_so {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  // node parameters
  rclcpp::Parameter prefix_;
  // Modbus RTU option #1: ROS2 interface config
  rclcpp::Parameter rtu_interface_prefix_;
  // Modbus RTU option #2: Native API config
  serial::PortSettings rtu_port_settings_;

  std::shared_ptr<StepperDriverRS485SOInterface> intf_;
};

}  // namespace stepper_driver_rs485_so

#endif  // OPENVMP_SD_RS485_SO_NODE_H
