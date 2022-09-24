/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "stepper_driver_rs485_so/node.hpp"

namespace stepper_driver_rs485_so {

Node::Node() : rclcpp::Node::Node("stepper_driver_rs485_so") {
  intf_ = std::shared_ptr<StepperDriverRS485SOInterface>(
      new StepperDriverRS485SOInterface(this));
}

}  // namespace stepper_driver_rs485_so
