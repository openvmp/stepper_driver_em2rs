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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "stepper_driver/interface.hpp"
#include "stepper_driver_rs485_so/node.hpp"

namespace stepper_driver_rs485_so {

class StepperDriverRS485SOInterface final
    : public stepper_driver::StepperDriverInterface {
 public:
  StepperDriverRS485SOInterface(Node *node,
                                const std::string &interface_prefix);

 private:
  virtual void param_ppr_get_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprGet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprGet::Response> response)
      override;
  virtual void param_ppr_set_handler_(
      const std::shared_ptr<stepper_driver::srv::ParamPprSet::Request> request,
      std::shared_ptr<stepper_driver::srv::ParamPprSet::Response> response)
      override;
};

}  // namespace stepper_driver_rs485_so

#endif  // OPENVMP_SD_RS485_SO_INTERFACE_H
