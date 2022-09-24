/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
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
  std::shared_ptr<StepperDriverRS485SOInterface> intf_;
};

}  // namespace stepper_driver_rs485_so

#endif  // OPENVMP_SD_RS485_SO_NODE_H
