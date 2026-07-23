# -*- coding: utf-8 -*-
"""
siheng_servo.client
===================
四横 D-AIS48025A 伺服驱动器 Modbus RTU 通信客户端。

示例:
    from siheng_servo import SihengServo

    servo = SihengServo(port="COM5", baudrate=115200, slave_id=1)
    servo.connect()
    servo.enable()
    servo.run(speed=500)       # 正转 500 rpm
    servo.stop()
    servo.disable()
    servo.disconnect()
"""

import logging
from typing import Optional, Dict, Any

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.serial import ModbusSerialClient as ModbusSerialClient

from .constants import (
    REGISTERS, REG_TYPES, DataType, ControlMode, SpeedSource,
    SERVO_STATUS, BAUDRATE_MAP, BAUDRATE_REVERSE, parse_param_address,
)
from .exceptions import (
    SihengError, ConnectionError, CommunicationError,
    ParameterError, ServoNotEnabledError, ServoFaultError,
)

logger = logging.getLogger("siheng_servo")


class SihengServo:
    """
    四横 D-AIS48025A 伺服驱动器控制客户端。

    Args:
        port: 串口设备名 (如 "COM5" / "/dev/ttyUSB0")
        baudrate: 波特率 (默认 57600)
        slave_id: Modbus 从站地址 (默认 1)
        parity: 校验位 ("N"/"E"/"O")
        stopbits: 停止位 (1 或 2)
        bytesize: 数据位 (固定 8)
        timeout: 通信超时秒数 (默认 1.0)
        word_order: 32位数据高低位顺序 (0=低字在前, 1=高字在前, None=自动读取)
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 57600,
        slave_id: int = 1,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        timeout: float = 1.0,
        word_order: Optional[int] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.word_order = word_order if word_order is not None else 1  # 出厂默认高字在前
        self._client: Optional[ModbusSerialClient] = None
        self._connected = False
        self._servo_enabled = False

    # ===================== 连接管理 =====================

    def connect(self) -> bool:
        """连接驱动器。成功返回 True。"""
        try:
            self._client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=self.stopbits,
                bytesize=self.bytesize,
                timeout=self.timeout,
            )
            if not self._client.connect():
                raise ConnectionError(f"无法打开串口 {self.port}")
            self._connected = True
            # 自动读取高低位顺序设置
            if self.word_order is None:
                order = self._read_register_raw("H0C_26")
                if order is not None:
                    self.word_order = order
                    logger.info(f"自动检测高低位顺序 H0C_26 = {order}")
            logger.info(f"已连接 {self.port} @ {self.baudrate}, 从站 {self.slave_id}")
            return True
        except ConnectionError:
            raise
        except Exception as e:
            raise ConnectionError(f"连接失败: {e}") from e

    def disconnect(self):
        """断开连接。"""
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self._connected = False
        self._servo_enabled = False
        logger.info("已断开连接")

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_enabled(self) -> bool:
        return self._servo_enabled

    # ===================== 底层读写 =====================

    def _read_register_raw(self, param: str) -> Optional[int]:
        """读取单个 UInt16 寄存器原始值。"""
        if not self._connected:
            raise ConnectionError("未连接驱动器")
        addr = parse_param_address(param)
        try:
            rr = self._client.read_holding_registers(addr, count=1, slave=self.slave_id)
            if rr.isError():
                raise CommunicationError(f"读 {param} (0x{addr:04X}) 失败: {rr}")
            return rr.registers[0]
        except CommunicationError:
            raise
        except Exception as e:
            raise CommunicationError(f"读 {param} 异常: {e}") from e

    def read(self, param: str) -> int:
        """
        读取参数值（自动按数据类型处理 16/32 位和符号）。

        Args:
            param: 参数号，如 "H06_03" / "H0B_00"

        Returns:
            int: 参数值（已按类型转换）
        """
        if not self._connected:
            raise ConnectionError("未连接驱动器")
        addr = parse_param_address(param)
        dtype = REG_TYPES.get(param, DataType.UINT16)

        try:
            if dtype in (DataType.UINT32, DataType.INT32):
                rr = self._client.read_holding_registers(addr, count=2, slave=self.slave_id)
                if rr.isError():
                    raise CommunicationError(f"读 {param} 失败: {rr}")
                r0, r1 = rr.registers[0], rr.registers[1]
                if self.word_order == 1:  # 高字在前
                    val = (r0 << 16) | r1
                else:  # 低字在前
                    val = (r1 << 16) | r0
                if dtype == DataType.INT32 and val > 2147483647:
                    val -= 4294967296
                return val
            else:
                rr = self._client.read_holding_registers(addr, count=1, slave=self.slave_id)
                if rr.isError():
                    raise CommunicationError(f"读 {param} 失败: {rr}")
                val = rr.registers[0]
                if dtype == DataType.INT16 and val > 32767:
                    val -= 65536
                return val
        except CommunicationError:
            raise
        except Exception as e:
            raise CommunicationError(f"读 {param} 异常: {e}") from e

    def write(self, param: str, value: int):
        """
        写入参数值（自动按数据类型处理 16/32 位）。

        Args:
            param: 参数号，如 "H06_03"
            value: 要写入的值
        """
        if not self._connected:
            raise ConnectionError("未连接驱动器")
        addr = parse_param_address(param)
        dtype = REG_TYPES.get(param, DataType.UINT16)

        try:
            if dtype in (DataType.UINT32, DataType.INT32):
                if value < 0:
                    value = value & 0xFFFFFFFF
                hi = (value >> 16) & 0xFFFF
                lo = value & 0xFFFF
                if self.word_order == 1:  # 高字在前
                    self._client.write_register(addr, hi, slave=self.slave_id)
                    self._client.write_register(addr + 1, lo, slave=self.slave_id)
                else:
                    self._client.write_register(addr, lo, slave=self.slave_id)
                    self._client.write_register(addr + 1, hi, slave=self.slave_id)
            else:
                if value < 0:
                    value = value & 0xFFFF
                rr = self._client.write_register(addr, value, slave=self.slave_id)
                if rr.isError():
                    raise CommunicationError(f"写 {param} 失败: {rr}")
        except CommunicationError:
            raise
        except Exception as e:
            raise CommunicationError(f"写 {param} 异常: {e}") from e

    # ===================== 使能控制 =====================

    def enable(self):
        """伺服使能 (H32_01=1)。"""
        self.write("H32_01", 1)
        self._servo_enabled = True
        logger.info("伺服使能")

    def disable(self):
        """伺服断使能 (H32_01=0)。"""
        self.write("H32_01", 0)
        self._servo_enabled = False
        logger.info("伺服断使能")

    def fault_reset(self):
        """故障复位 (H0D_01=1)。"""
        self.write("H0D_01", 1)
        logger.info("故障复位")

    def emergency_stop(self):
        """紧急停机 (H0D_05=1)。"""
        self.write("H0D_05", 1)
        self._servo_enabled = False
        logger.warning("紧急停机")

    # ===================== 速度控制 =====================

    def set_speed(self, rpm: int):
        """
        设置目标转速 (H06_03)。
        正值正转，负值反转，0 停止。
        速度控制模式下需要先使能。

        Args:
            rpm: 转速 -3000~3000
        """
        if not self._servo_enabled:
            logger.warning("设置速度时伺服未使能，电机可能不会转动")
        if not -3000 <= rpm <= 3000:
            raise ParameterError(f"转速 {rpm} 超出范围 [-3000, 3000]")
        self.write("H06_03", rpm)

    def run(self, speed: int):
        """
        设置速度并运行（等价于 set_speed）。

        Args:
            speed: 转速 rpm，负值反转
        """
        self.set_speed(speed)
        logger.info(f"运行 speed={speed} rpm")

    def stop(self):
        """停止运行 (H06_03=0)。"""
        self.write("H06_03", 0)
        logger.info("停止")

    def jog(self, speed: int):
        """
        点动运行。

        Args:
            speed: 点动速度 rpm，正值正转，负值反转
        """
        self.write("H06_04", abs(speed))
        self.set_speed(speed)
        logger.info(f"点动 {speed} rpm")

    def jog_stop(self):
        """停止点动。"""
        self.set_speed(0)

    def set_acceleration(self, accel_ms: int, decel_ms: int = None):
        """
        设置加减速时间 (H06_05 / H06_06)。

        Args:
            accel_ms: 加速时间 ms
            decel_ms: 减速时间 ms (默认与加速相同)
        """
        if decel_ms is None:
            decel_ms = accel_ms
        self.write("H06_05", accel_ms)
        self.write("H06_06", decel_ms)
        logger.info(f"加减速时间: 加速={accel_ms}ms 减速={decel_ms}ms")

    # ===================== 模式配置 =====================

    def set_control_mode(self, mode: int):
        """
        设置控制模式 (H02_00)。需要断使能后生效。

        Args:
            mode: 0=位置, 1=速度, 2=转矩
        """
        if self._servo_enabled:
            raise ServoNotEnabledError("设置控制模式前需要先断使能")
        self.write("H02_00", mode)
        logger.info(f"控制模式 = {mode}")

    def set_speed_source(self, source: int):
        """
        设置速度指令来源 (H06_00)。

        Args:
            source: 0=内部, 1=模拟量, 2=通讯
        """
        self.write("H06_00", source)
        logger.info(f"速度源 = {source}")

    def configure_speed_mode(self):
        """
        一键配置为内部速度控制模式。
        需在断使能状态下调用。
        """
        self.set_control_mode(ControlMode.SPEED)
        self.set_speed_source(SpeedSource.INTERNAL)

    # ===================== 状态读取 =====================

    def get_status(self) -> Dict[str, Any]:
        """
        读取并返回完整状态字典。

        Returns:
            dict: 包含以下字段:
                - servo_status (int): 伺服状态码 0~6
                - servo_status_text (str): 状态描述
                - speed (int): 实际转速 rpm
                - position (int): 绝对位置
                - torque (int): 转矩指令 0.1%
                - load (int): 负载率 0.1%
                - current (int): 相电流 0.01A
                - voltage (float): 母线电压 V
                - temperature (int): 模块温度 ℃
                - fault (int): 故障码
        """
        status_code = self.read("H32_02")
        return {
            "servo_status": status_code,
            "servo_status_text": SERVO_STATUS.get(status_code, f"未知({status_code})"),
            "speed": self.read("H0B_00"),
            "position": self.read("H0B_07"),
            "torque": self.read("H0B_02"),
            "load": self.read("H0B_12"),
            "current": self.read("H0B_24"),
            "voltage": self.read("H0B_26") / 10.0,
            "temperature": self.read("H0B_27"),
            "fault": self.read("H0B_33"),
        }

    def get_speed(self) -> int:
        """读取实际转速 rpm。"""
        return self.read("H0B_00")

    def get_position(self) -> int:
        """读取绝对位置。"""
        return self.read("H0B_07")

    def get_servo_status(self) -> str:
        """读取伺服状态文本。"""
        code = self.read("H32_02")
        return SERVO_STATUS.get(code, f"未知({code})")

    def get_fault_code(self) -> int:
        """读取故障码 (0=无故障)。"""
        return self.read("H0B_33")

    def get_bus_voltage(self) -> float:
        """读取母线电压 V。"""
        return self.read("H0B_26") / 10.0

    def get_temperature(self) -> int:
        """读取模块温度 ℃。"""
        return self.read("H0B_27")

    # ===================== 通信参数 =====================

    def get_slave_id(self) -> int:
        """读取从站地址 (H0C_00)。"""
        return self.read("H0C_00")

    def get_baudrate(self) -> int:
        """读取实际波特率值 (H0C_02 解码)。"""
        code = self.read("H0C_02")
        return BAUDRATE_MAP.get(code, -1)

    def save_to_eeprom(self):
        """将参数写入 EEPROM 永久保存 (H0C_13=1)。"""
        self.write("H0C_13", 1)
        logger.info("参数已写入 EEPROM")

    # ===================== 上下文管理 =====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._servo_enabled:
            try:
                self.stop()
            except Exception:
                pass
        self.disconnect()
        return False

    def __repr__(self):
        return (f"SihengServo(port={self.port!r}, baudrate={self.baudrate}, "
                f"slave_id={self.slave_id}, connected={self._connected})")
