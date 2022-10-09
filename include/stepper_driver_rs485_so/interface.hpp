/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SD_RS485_SO_INTERFACE_H
#define OPENVMP_SD_RS485_SO_INTERFACE_H

#include <memory>
#include <string>

#include "modbus/interface.hpp"
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
  std::shared_ptr<modbus::Interface> prov_;
};

}  // namespace stepper_driver_rs485_so

#endif  // OPENVMP_SD_RS485_SO_INTERFACE_H
