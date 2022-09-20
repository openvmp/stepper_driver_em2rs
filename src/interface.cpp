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

#include "stepper_driver_rs485_so/interface.hpp"

namespace stepper_driver_rs485_so {

StepperDriverRS485SOInterface::StepperDriverRS485SOInterface(
    Node *node, const std::string &interface_prefix)
    : StepperDriverInterface((rclcpp::Node *)node, interface_prefix) {}

void StepperDriverRS485SOInterface::param_ppr_get_handler_(
    const std::shared_ptr<stepper_driver::srv::ParamPprGet::Request> request,
    std::shared_ptr<stepper_driver::srv::ParamPprGet::Response> response) {
  (void)request;
  response->status_code = 1;
}

void StepperDriverRS485SOInterface::param_ppr_set_handler_(
    const std::shared_ptr<stepper_driver::srv::ParamPprSet::Request> request,
    std::shared_ptr<stepper_driver::srv::ParamPprSet::Response> response)

{
  (void)request;
  response->status_code = 1;
}

}  // namespace stepper_driver_rs485_so