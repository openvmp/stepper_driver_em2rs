/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SD_EM2RS_NODE_H
#define OPENVMP_SD_EM2RS_NODE_H

#include <memory>
#include <string>

#include "em2rs_driver/interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_serial/port.hpp"

namespace em2rs_driver {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<Interface> intf_;
};

}  // namespace em2rs_driver

#endif  // OPENVMP_SD_EM2RS_NODE_H
