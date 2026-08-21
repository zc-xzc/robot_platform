# -*- coding: utf-8 -*-
"""
siheng_servo.exceptions
=======================
SDK 自定义异常类。
"""


class SihengError(Exception):
    """所有 siheng_servo 异常的基类。"""


class ConnectionError(SihengError):
    """串口连接失败。"""


class CommunicationError(SihengError):
    """Modbus 通信失败（超时、无响应、CRC 错误等）。"""


class ParameterError(SihengError):
    """参数错误（地址无效、数值超范围、数据类型错误等）。"""


class ServoNotEnabledError(SihengError):
    """伺服未使能时执行了需要使能的操作。"""


class ServoFaultError(SihengError):
    """伺服处于故障/急停状态。"""
