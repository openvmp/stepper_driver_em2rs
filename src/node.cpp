/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "em2rs_driver/node.hpp"

namespace em2rs_driver {

Node::Node() : rclcpp::Node::Node("stepper_driver_em2rs") {
  intf_ = std::make_shared<Interface>(this);
}

}  // namespace em2rs_driver
