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

#include <locale>

#include "stepper_driver_rs485_so/node.hpp"

namespace stepper_driver_rs485_so {

StepperDriverRS485SOInterface::StepperDriverRS485SOInterface(rclcpp::Node *node)
    : StepperDriverInterface(node), prov_(node) {
  node->declare_parameter("stepper_driver_model", "dm556rs");
  node->get_parameter("stepper_driver_model", param_model);
  std::string model = param_model.as_string();
  for (auto &c : model) {
    c = tolower(c);
  }

  node->declare_parameter("stepper_driver_rs485_leaf_id", 1);
  node->get_parameter("stepper_driver_rs485_leaf_id", param_leaf_id);
  uint16_t leaf_id = param_leaf_id.as_int();

  prov_.generate_modbus_mappings(leaf_id, interface_prefix_.as_string(),
                                 "config/modbus.yaml");
  if (model == "dm556rs") {
    prov_.generate_modbus_mappings(leaf_id, interface_prefix_.as_string(),
                                   "config/modbus556.yaml");
  } else if (model == "dm882rs") {
    prov_.generate_modbus_mappings(leaf_id, interface_prefix_.as_string(),
                                   "config/modbus882.yaml");
  } else {
    throw std::invalid_argument("unsupported stepper driver model");
  }
}

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