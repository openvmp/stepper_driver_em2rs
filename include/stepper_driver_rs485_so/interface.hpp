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

#ifndef OPENVMP_SD_RS485_SO_INTERFACE_H
#define OPENVMP_SD_RS485_SO_INTERFACE_H

#include <memory>
#include <string>

#include "modbus_rtu/interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "stepper_driver/interface.hpp"

namespace stepper_driver_rs485_so {

class StepperDriverRS485SOInterface
    : public stepper_driver::StepperDriverInterface {
 public:
  StepperDriverRS485SOInterface(rclcpp::Node *node);
  virtual ~StepperDriverRS485SOInterface() {}

  rclcpp::Parameter param_model;
  rclcpp::Parameter param_leaf_id;

 protected:
  void param_ppr_get_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprGet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprGet::Response> response)
      override;
  void param_ppr_set_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprSet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprSet::Response> response)
      override;

 private:
  modbus_rtu::ModbusRtuInterface prov_;
};

}  // namespace stepper_driver_rs485_so

#endif  // OPENVMP_SD_RS485_SO_INTERFACE_H
