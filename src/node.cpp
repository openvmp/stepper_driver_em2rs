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

#include "stepper_driver_rs485_so/node.hpp"

namespace stepper_driver_rs485_so {

Node::Node() : rclcpp::Node::Node("stepper_driver_rs485_so") {
  this->declare_parameter("prefix", "/stepper_driver_rs485_so");
  this->get_parameter("prefix", prefix_);

  // TODO(clairbee): instantiate the provider: Modbus RTU instance

  intf_ = std::shared_ptr<StepperDriverRS485SOInterface>(
      new StepperDriverRS485SOInterface(this, prefix_.as_string()));
}

}  // namespace stepper_driver_rs485_so
