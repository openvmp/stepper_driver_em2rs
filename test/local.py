import sys

sys.path.append("test/lib")

from stepper_driver_rs485_test import StepperDriverRS485TesterNode

from time import sleep
import rclpy

import pytest
import unittest
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    ExecuteProcess,
)
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable


TTY1 = "/tmp/ttyS21"
TTY2 = "/tmp/ttyS22"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    socat = ExecuteProcess(
        name="socat",
        cmd=[
            [
                FindExecutable(name="socat"),
                " -s",
                " PTY,rawer,link=",
                TTY1,
                " PTY,rawer,link=",
                TTY2,
            ]
        ],
        shell=True,
    )
    node1 = Node(
        name="stepper_driver",
        package="stepper_driver_rs485_so",
        executable="stepper_driver_rs485_so_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "stepper_driver_prefix": "/stepper_driver/motor1",
                "stepper_driver_model": "DM556RS",
                "modbus_is_remote": False,
                "modbus_prefix": "/modbus/bus1",
                "modbus_leaf_id": 1,
                "serial_is_remote": False,
                "serial_prefix": "/serial/com1",
                "serial_dev_name": TTY1,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )
    node2 = Node(
        name="serial_com2",
        package="serial",
        executable="serial_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_prefix": "/serial/com2",
                "serial_dev_name": TTY2,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                socat,
                RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=socat,
                        on_start=[
                            node1,
                            node2,
                        ],
                    )
                ),
                ReadyToTest(),
            ]
        ),
        {"socat": socat, "stepper_driver": node1, "serial_com2": node2},
    )


class TestLocal(unittest.TestCase):
    def test_configured_holding_register1(self, proc_output):
        rclpy.init()
        try:
            sleep(3)
            node = StepperDriverRS485TesterNode()
            node.subscribe(1)
            node.subscribe(2)
            node.subscribe(1, "output")
            node.subscribe(2, "output")
            node.subscribe_modbus_rtu(1)

            future = node.configured_holding_register_read(1, "ppr")
            sleep(2)
            node.inject(2, "output", bytes.fromhex("01 03 02 41 41 48 24"))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

            assert future.done(), "Client request timed out"

            response = future.result()
            print(response)
            assert response, "Could not inject!"
            sleep(3)

            assert response.exception_code == 0, "value mismatch"
            assert response.value == 16705, "value mismatch"
        finally:
            rclpy.shutdown()
            _ignore = 1

    def test_param_ppr_get(self, proc_output):
        rclpy.init()
        try:
            sleep(3)
            node = StepperDriverRS485TesterNode()
            node.subscribe(1)
            node.subscribe(2)
            node.subscribe(1, "output")
            node.subscribe(2, "output")
            node.subscribe_modbus_rtu(1)

            future = node.param_get_ppr(1)
            sleep(2)
            node.inject(2, "output", bytes.fromhex("01 03 02 41 41 48 24"))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

            assert future.done(), "Client request timed out"

            response = future.result()
            print(response)
            assert response, "Could not inject!"
            sleep(3)

            assert response.exception_code == 0, "value mismatch"
            assert response.ppr == 16705, "value mismatch"
        finally:
            rclpy.shutdown()
            _ignore = 1
