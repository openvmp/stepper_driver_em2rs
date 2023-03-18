/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "stepper_driver_em2rs/factory.hpp"

#include <exception>

#include "remote_actuator/interface_remote.hpp"
#include "stepper_driver_em2rs/interface.hpp"

namespace stepper_driver_em2rs {

std::shared_ptr<remote_actuator::Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  if (!node->has_parameter("actuator_is_remote")) {
    node->declare_parameter("actuator_is_remote", use_remote.as_bool());
  }
  node->get_parameter("actuator_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<remote_actuator::RemoteInterface>(node);
  } else {
    return std::make_shared<Interface>(node);
  }
}

}  // namespace stepper_driver_em2rs
