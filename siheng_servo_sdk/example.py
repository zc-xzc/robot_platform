# -*- coding: utf-8 -*-
"""
siheng_servo SDK 使用示例
=========================
运行前请先安装: pip install .
"""

import time
from siheng_servo import SihengServo, ControlMode, SpeedSource, SihengError


def example_basic_control():
    """示例 1: 基本速度控制（with 语法自动断开连接）"""
    print("=== 示例 1: 基本速度控制 ===")
    with SihengServo(port="COM5", baudrate=57600, slave_id=1) as servo:
        # 读取状态
        print(f"伺服状态: {servo.get_servo_status()}")
        print(f"母线电压: {servo.get_bus_voltage()} V")
        print(f"模块温度: {servo.get_temperature()} ℃")

        # 使能并运行
        servo.enable()
        print("伺服已使能")

        servo.run(speed=300)   # 正转 300 rpm
        print("正转 300 rpm, 运行 3 秒...")
        time.sleep(3)

        servo.run(speed=-300)  # 反转 300 rpm
        print("反转 300 rpm, 运行 3 秒...")
        time.sleep(3)

        servo.stop()
        print("停止")

        servo.disable()
        print("断使能")


def example_configure_speed_mode():
    """示例 2: 配置为内部速度控制模式（需断使能）"""
    print("\n=== 示例 2: 配置速度模式 ===")
    servo = SihengServo(port="COM5", baudrate=57600, slave_id=1)
    servo.connect()

    # 确保断使能
    servo.disable()
    # 一键配置: 速度模式 + 内部速度源
    servo.configure_speed_mode()
    print("已配置为内部速度控制模式")

    servo.disconnect()


def example_continuous_monitor():
    """示例 3: 持续监控状态"""
    print("\n=== 示例 3: 状态监控 ===")
    servo = SihengServo(port="COM5", baudrate=57600, slave_id=1)
    servo.connect()

    try:
        for i in range(10):
            status = servo.get_status()
            print(f"[{i}] {status['servo_status_text']} | "
                  f"转速: {status['speed']} rpm | "
                  f"电压: {status['voltage']:.1f}V | "
                  f"温度: {status['temperature']}℃ | "
                  f"故障: {status['fault']}")
            time.sleep(0.5)
    finally:
        servo.disconnect()


def example_param_read_write():
    """示例 4: 任意参数读写"""
    print("\n=== 示例 4: 参数读写 ===")
    servo = SihengServo(port="COM5", baudrate=57600, slave_id=1)
    servo.connect()

    # 读取加减速时间
    accel = servo.read("H06_05")
    decel = servo.read("H06_06")
    print(f"当前加速时间: {accel} ms, 减速时间: {decel} ms")

    # 修改加减速时间
    servo.set_acceleration(accel_ms=1000, decel_ms=1000)
    print("已设置加速/减速时间 = 1000 ms")

    servo.disconnect()


def example_scan_communication():
    """示例 5: 读取通信参数"""
    print("\n=== 示例 5: 通信参数 ===")
    servo = SihengServo(port="COM5", baudrate=57600, slave_id=1)
    servo.connect()

    print(f"从站地址: {servo.get_slave_id()}")
    print(f"实际波特率: {servo.get_baudrate()}")
    print(f"软件版本: {servo.read('H01_00')}")

    servo.disconnect()


def example_error_handling():
    """示例 6: 异常处理"""
    print("\n=== 示例 6: 异常处理 ===")
    from siheng_servo import (
        ConnectionError, CommunicationError, ParameterError
    )

    try:
        servo = SihengServo(port="COM99", baudrate=57600)
        servo.connect()
    except ConnectionError as e:
        print(f"连接错误: {e}")
    except CommunicationError as e:
        print(f"通信错误: {e}")
    except SihengError as e:
        print(f"其他错误: {e}")


if __name__ == "__main__":
    # 选择运行哪个示例
    examples = [
        example_basic_control,
        example_configure_speed_mode,
        example_continuous_monitor,
        example_param_read_write,
        example_scan_communication,
        example_error_handling,
    ]

    for ex in examples:
        try:
            ex()
        except SihengError as e:
            print(f"[{ex.__name__}] 失败: {e}")
        except KeyboardInterrupt:
            print("\n用户中断")
            break
