/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "stepper_driver_em2rs/node.hpp"

namespace stepper_driver_em2rs {

Node::Node() : rclcpp::Node::Node("stepper_driver_em2rs") {
  intf_ = std::make_shared<StepperDriverRS485SOInterface>(this);
}

}  // namespace stepper_driver_em2rs
