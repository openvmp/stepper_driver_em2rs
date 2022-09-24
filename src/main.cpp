/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
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
