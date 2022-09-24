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

#include "modbus_rtu/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stepper_driver_rs485_so/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto rtu_node = std::make_shared<modbus_rtu::Node>();
  auto node = std::make_shared<stepper_driver_rs485_so::Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
