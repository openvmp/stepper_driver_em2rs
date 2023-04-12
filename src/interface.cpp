/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_em2rs/interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>

#include "ros2_modbus_rtu/factory.hpp"

#ifndef DEBUG
#undef RCLCPP_DEBUG
#define RCLCPP_DEBUG(...)
#endif

namespace ros2_em2rs {

Interface::Interface(rclcpp::Node *node)
    : remote_stepper_driver::Implementation(node) {
  auto prefix = get_prefix_();

  prov_ = ros2_modbus_rtu::Factory::New(node);

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): started");

  node->declare_parameter("stepper_driver_model", "dm556rs");
  node->get_parameter("stepper_driver_model", param_model);
  std::string model = param_model.as_string();
  for (auto &c : model) {
    c = tolower(c);
  }

  std::string share_dir =
      ament_index_cpp::get_package_share_directory("ros2_em2rs");
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

  init_actuator();
  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

rclcpp::FutureReturnCode Interface::param_ppr_get_handler_(
    const std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Request>
        request,
    std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Response>
        response) {
  (void)request;
  auto req = std::make_shared<ros2_modbus::srv::HoldingRegisterRead::Request>();
  auto resp =
      std::make_shared<ros2_modbus::srv::HoldingRegisterRead::Response>();

  // req->leaf_id will be auto-filled
  req->leaf_id = 0;
  req->addr = 0x0001;
  req->count = 1;

  prov_->holding_register_read(req, resp);
  response->exception_code = resp->exception_code;
  if (!resp->exception_code && resp->len) {
    response->ppr = resp->values[0];
    return rclcpp::FutureReturnCode::SUCCESS;
  }

  return rclcpp::FutureReturnCode::INTERRUPTED;
}

rclcpp::FutureReturnCode Interface::param_ppr_set_handler_(
    const std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Request>
        request,
    std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Response> response)

{
  auto req =
      std::make_shared<ros2_modbus::srv::HoldingRegisterWrite::Request>();
  auto resp =
      std::make_shared<ros2_modbus::srv::HoldingRegisterWrite::Response>();

  // req->leaf_id will be auto-filled
  req->leaf_id = 0;
  req->addr = 0x0001;
  req->value = request->ppr;

  prov_->holding_register_write(req, resp);
  response->exception_code = resp->exception_code;

  if (!resp->exception_code) {
    return rclcpp::FutureReturnCode::SUCCESS;
  }

  return rclcpp::FutureReturnCode::INTERRUPTED;
}

void Interface::velocity_set_real_(double velocity) {
  auto req =
      std::make_shared<ros2_modbus::srv::HoldingRegisterWrite::Request>();
  auto resp =
      std::make_shared<ros2_modbus::srv::HoldingRegisterWrite::Response>();

  uint16_t velocity_val = ::abs(velocity * 60.0 / (2.0 * M_PI));

  // Adjust the jog speed if necessary
  if (velocity_last_ != velocity_val && velocity_val != 0) {
    req->leaf_id = 0;    // it will be auto-filled
    req->addr = 0x01E1;  // Jog speed
    req->value = velocity_val;

    RCLCPP_DEBUG(node_->get_logger(),
                 "Interface::velocity_set_real_(): setting the jog speed: %04x",
                 velocity_val);
    prov_->holding_register_write(req, resp);
    if (resp->exception_code) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Interface::velocity_set_real_(): failed setting the jog speed");
    }
    velocity_last_ = velocity_val;
  }

  // Send the jog command if it is supposed to keep moving
  if (velocity_val != 0) {
    req->leaf_id = 0;    // it will be auto-filled
    req->addr = 0x1801;  // Trigger jog
    if (velocity > 0) {
      req->value = 0x4001;  // clockwise
    } else {
      req->value = 0x4002;  // counter-clockwise
    }
    RCLCPP_DEBUG(node_->get_logger(),
                 "Interface::velocity_set_real_(): triggering the jog");
    prov_->holding_register_write(req, resp);
    if (resp->exception_code) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Interface::velocity_set_real_(): failed triggering the jog");
    }

    double velocity_real = ((double)velocity_val) * 2.0 * M_PI / 60.0;
    if (velocity < 0) {
      velocity_real *= -1.0;
    }
    velocity_did_set_(velocity_real);
  } else {
    velocity_did_set_(0.0);
  }
}

}  // namespace ros2_em2rs
