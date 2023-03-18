/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_STEPPER_DRIVER_EM2RS_FACTORY_H
#define OPENVMP_STEPPER_DRIVER_EM2RS_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/interface.hpp"

namespace stepper_driver_em2rs {

class Factory {
 public:
  static std::shared_ptr<remote_actuator::Interface> New(rclcpp::Node *node);
};

}  // namespace stepper_driver_em2rs

#endif  // OPENVMP_STEPPER_DRIVER_EM2RS_FACTORY_H