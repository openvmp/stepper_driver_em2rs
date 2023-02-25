/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "stepper_driver_em2rs/interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>

#include "modbus_rtu/factory.hpp"
#include "stepper_driver_em2rs/node.hpp"

namespace stepper_driver_em2rs {

Interface::Interface(rclcpp::Node *node) : stepper_driver::Interface(node) {
  auto prefix = get_prefix_();

  prov_ = modbus_rtu::Factory::New(node);

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface():" " started");

  node->declare_parameter("stepper_driver_model", "dm556rs");
  node->get_parameter("stepper_driver_model", param_model);
  std::string model = param_model.as_string();
  for (auto &c : model) {
    c = tolower(c);
  }

  std::string share_dir =
      ament_index_cpp::get_package_share_directory("stepper_driver_em2rs");
  prov_->generate_modbus_mappings(prefix, share_dir + "/config/modbus.yaml");
  if (model == "dm556rs") {
    prov_->generate_modbus_mappings(prefix,
                                    share_dir + "/config/modbus556.yaml");
  } else if (model == "dm882rs") {
    prov_->generate_modbus_mappings(prefix,
                                    share_dir + "/config/modbus882.yaml");
  } else {
    throw std::invalid_argument("unsupported stepper driver model");
  }

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

void Interface::param_ppr_get_handler_(
    const std::shared_ptr<stepper_driver::srv::ParamPprGet::Request> request,
    std::shared_ptr<stepper_driver::srv::ParamPprGet::Response> response) {
  (void)request;
  auto req = std::make_shared<modbus::srv::HoldingRegisterRead::Request>();
  auto resp = std::make_shared<modbus::srv::HoldingRegisterRead::Response>();

  // req->leaf_id will be auto-filled
  req->addr = 0x0001;
  req->count = 1;

  prov_->holding_register_read(req, resp);
  response->exception_code = resp->exception_code;
  if (!resp->exception_code && resp->len) {
    response->ppr = resp->values[0];
  }
}

void Interface::param_ppr_set_handler_(
    const std::shared_ptr<stepper_driver::srv::ParamPprSet::Request> request,
    std::shared_ptr<stepper_driver::srv::ParamPprSet::Response> response)

{
  auto req = std::make_shared<modbus::srv::HoldingRegisterWrite::Request>();
  auto resp = std::make_shared<modbus::srv::HoldingRegisterWrite::Response>();

  // req->leaf_id will be auto-filled
  req->addr = 0x0001;
  req->value = request->ppr;

  prov_->holding_register_write(req, resp);
  response->exception_code = resp->exception_code;
}

}  // namespace stepper_driver_em2rs
