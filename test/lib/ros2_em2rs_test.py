import sys

sys.path.append("../ros2_modbus_rtu/test/lib")

from modbus_rtu_test import ModbusRtuTesterNode

from time import sleep

import rclpy
from rclpy.node import Node as rclpyNode

from ros2_modbus.srv import HoldingRegisterRead, ConfiguredHoldingRegisterRead
from stepper_driver.srv import ParamPprGet, ParamPprSet
from ros2_serial.srv import InjectOutput
import std_msgs.msg


class EM2RSTesterNode(ModbusRtuTesterNode):
    test_context = {}

    def __init__(self, name="stepper_driver_em2rs_so_tester_node"):
        super().__init__(name)

    def configured_holding_register_read(self, id, name, timeout=10.0):
        client = self.create_client(
            ConfiguredHoldingRegisterRead,
            "/stepper/motor" + str(id) + "/modbus/get_" + name,
        )
        ready = client.wait_for_service(timeout_sec=timeout)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = ConfiguredHoldingRegisterRead.Request()

        future = client.call_async(request)
        return future

    def param_get_ppr(self, id, timeout=10.0):
        service_name = "/stepper/motor" + str(id) + "/param/ppr/get"
        client = self.create_client(
            ParamPprGet,
            service_name,
        )
        ready = client.wait_for_service(timeout_sec=timeout)
        if not ready:
            raise RuntimeError("Wait for service timed out: " + service_name)

        request = ParamPprGet.Request()

        future = client.call_async(request)
        return future
