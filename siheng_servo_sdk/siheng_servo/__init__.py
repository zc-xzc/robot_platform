# -*- coding: utf-8 -*-
"""
siheng_servo
============
上海四横 D-AIS48025A 伺服驱动器 Python SDK
基于 Modbus RTU 协议，支持 RS485 通信控制。

快速开始:
    from siheng_servo import SihengServo

    with SihengServo(port="COM5", baudrate=115200) as servo:
        servo.enable()
        servo.run(speed=500)   # 正转 500 rpm
        import time; time.sleep(3)
        servo.stop()
"""

from .client import SihengServo
from .lift import LiftController
from .constants import (
    REGISTERS, REG_TYPES, DataType, ControlMode, SpeedSource,
    SERVO_STATUS, BAUDRATE_MAP, BAUDRATE_REVERSE, parse_param_address,
)
from .exceptions import (
    SihengError, ConnectionError, CommunicationError,
    ParameterError, ServoNotEnabledError, ServoFaultError,
)

__version__ = "1.0.0"
__author__ = "siheng_servo SDK"
__all__ = [
    "SihengServo",
    "LiftController",
    "REGISTERS", "REG_TYPES", "DataType", "ControlMode", "SpeedSource",
    "SERVO_STATUS", "BAUDRATE_MAP", "BAUDRATE_REVERSE", "parse_param_address",
    "SihengError", "ConnectionError", "CommunicationError",
    "ParameterError", "ServoNotEnabledError", "ServoFaultError",
    "launch_gui",
]


def launch_gui():
    """启动 GUI 控制界面（需要 PyQt5）。"""
    from .gui import launch
    launch()
