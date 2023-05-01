/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SD_EM2RS_INTERFACE_H
#define OPENVMP_SD_EM2RS_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_modbus/interface.hpp"
#include "remote_stepper_driver/implementation.hpp"
#include "std_msgs/msg/int32.hpp"

namespace em2rs_driver {

class Interface : public remote_stepper_driver::Implementation {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  rclcpp::Parameter param_model;

 protected:
  virtual bool has_velocity() override { return true; }
  virtual void position_set_real_(double) override {}
  virtual void velocity_set_real_(double) override;

  virtual rclcpp::FutureReturnCode param_ppr_get_handler_(
      const std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Request>
          request,
      std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Response>
          response) override;
  virtual rclcpp::FutureReturnCode param_ppr_set_handler_(
      const std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Request>
          request,
      std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Response>
          response) override;

 private:
  int16_t velocity_last_ = 0x7FFF;
  std::shared_ptr<remote_modbus::Interface> prov_;
};

}  // namespace em2rs_driver

#endif  // OPENVMP_SD_EM2RS_INTERFACE_H
