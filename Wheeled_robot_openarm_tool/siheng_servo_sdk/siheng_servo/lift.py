# -*- coding: utf-8 -*-
"""
siheng_servo.lift
=================
升降控制模块。
基于 SihengServo SDK，封装升降机构的上升/下降/停止/定位功能。

支持两种控制方式:
  1. 速度控制: up()/down()/stop() —— 持续运行直到停止
  2. 位置控制: move_to_height() —— 自动运行到指定高度

示例:
    from siheng_servo import SihengServo
    from siheng_servo.lift import LiftController

    servo = SihengServo(port="COM5", baudrate=57600)
    servo.connect()

    lift = LiftController(servo, up_direction=1, pulses_per_mm=1000)
    lift.enable()
    lift.up(speed=200)        # 上升 (速度 200 rpm)
    lift.stop()               # 停止
    lift.down(speed=200)      # 下降
    lift.stop()
    lift.move_to_height(50)   # 自动移动到 50mm 高度
"""

import time
import threading
import logging
from typing import Optional

from .client import SihengServo
from .exceptions import SihengError, ParameterError

logger = logging.getLogger("siheng_servo.lift")


class LiftController:
    """
    升降控制器。

    通过控制伺服电机的正反转实现升降机构的上升/下降。
    正转=上升 (up_direction=1)，反转=下降。

    Args:
        servo: 已连接的 SihengServo 实例
        up_direction: 上升对应的电机方向 (1=正转上升, -1=反转上升)
        default_speed: 默认运行速度 rpm
        pulses_per_mm: 每毫米对应的编码器脉冲数 (位置控制必需，用于高度换算)
        tolerance_mm: 位置控制到位容差 mm
        timeout_s: 位置控制超时秒数
    """

    def __init__(
        self,
        servo: SihengServo,
        up_direction: int = 1,
        default_speed: int = 200,
        pulses_per_mm: Optional[float] = None,
        tolerance_mm: float = 0.5,
        timeout_s: float = 30.0,
    ):
        self.servo = servo
        self.up_direction = 1 if up_direction >= 0 else -1
        self.default_speed = default_speed
        self.pulses_per_mm = pulses_per_mm
        self.tolerance_mm = tolerance_mm
        self.timeout_s = timeout_s
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._target_height: Optional[float] = None

    # ===================== 使能控制 =====================

    def enable(self):
        """伺服使能。升降操作前必须先调用。"""
        self.servo.enable()
        logger.info("升降伺服已使能")

    def disable(self):
        """伺服断使能。"""
        self._stop_monitor()
        self.servo.stop()
        self.servo.disable()
        logger.info("升降伺服已断使能")

    def fault_reset(self):
        """故障复位。"""
        self.servo.fault_reset()
        logger.info("故障已复位")

    def emergency_stop(self):
        """紧急停止。立即停止并取消所有位置控制任务。"""
        self._stop_monitor()
        self.servo.emergency_stop()
        logger.warning("升降急停")

    # ===================== 速度控制 =====================

    def up(self, speed: Optional[int] = None):
        """
        持续上升。调用后升降机构会持续上升，直到调用 stop() 或到达机械限位。

        Args:
            speed: 上升速度 rpm (None=使用默认速度)
        """
        self._stop_monitor()
        s = speed if speed is not None else self.default_speed
        if not 1 <= s <= 3000:
            raise ParameterError(f"速度 {s} 超出范围 [1, 3000]")
        self.servo.run(s * self.up_direction)
        logger.info(f"上升 speed={s} rpm")

    def down(self, speed: Optional[int] = None):
        """
        持续下降。调用后升降机构会持续下降，直到调用 stop() 或到达机械限位。

        Args:
            speed: 下降速度 rpm (None=使用默认速度)
        """
        self._stop_monitor()
        s = speed if speed is not None else self.default_speed
        if not 1 <= s <= 3000:
            raise ParameterError(f"速度 {s} 超出范围 [1, 3000]")
        self.servo.run(-s * self.up_direction)
        logger.info(f"下降 speed={s} rpm")

    def stop(self):
        """停止升降。"""
        self._stop_monitor()
        self.servo.stop()
        logger.info("升降停止")

    # ===================== 位置控制 =====================

    def get_position(self) -> int:
        """读取当前编码器位置（原始脉冲值）。"""
        return self.servo.get_position()

    def get_height(self) -> Optional[float]:
        """
        获取当前高度 mm。
        需要配置 pulses_per_mm 才能换算，否则返回 None。

        Returns:
            当前高度 mm，或 None（未配置 pulses_per_mm 时）
        """
        if self.pulses_per_mm is None:
            return None
        return self.servo.get_position() / self.pulses_per_mm

    def move_to_height(self, target_height_mm: float, speed: Optional[int] = None):
        """
        移动到指定高度（阻塞执行，自动停止）。
        基于速度模式 + 位置反馈实现，无需切换控制模式。

        需要先调用 enable() 使能伺服，且配置 pulses_per_mm。

        Args:
            target_height_mm: 目标高度 mm
            speed: 运行速度 rpm (None=使用默认速度)

        Raises:
            ParameterError: 未配置 pulses_per_mm 或参数非法
        """
        if self.pulses_per_mm is None:
            raise ParameterError("位置控制需要配置 pulses_per_mm (每毫米脉冲数)")
        if not 1 <= speed <= 3000 if speed else not 1 <= self.default_speed <= 3000:
            raise ParameterError("速度超出范围 [1, 3000]")

        s = speed if speed is not None else self.default_speed
        current = self.get_height()
        diff = target_height_mm - current
        tolerance = self.tolerance_mm

        if abs(diff) <= tolerance:
            logger.info(f"已在目标高度 {target_height_mm}mm (当前 {current:.2f}mm)")
            return

        # 启动方向
        if diff > 0:
            self.up(s)
        else:
            self.down(s)

        logger.info(f"移动到 {target_height_mm}mm (当前 {current:.2f}mm, 差 {diff:.2f}mm)")

        # 启动监控线程
        self._stop_event.clear()
        self._target_height = target_height_mm
        self._monitor_thread = threading.Thread(
            target=self._position_monitor,
            args=(target_height_mm, s, tolerance),
            daemon=True,
        )
        self._monitor_thread.start()

    def move_relative(self, delta_mm: float, speed: Optional[int] = None):
        """
        相对移动指定高度（阻塞执行，自动停止）。

        Args:
            delta_mm: 高度变化量 mm，正值上升，负值下降
            speed: 运行速度 rpm
        """
        if self.pulses_per_mm is None:
            raise ParameterError("位置控制需要配置 pulses_per_mm")
        current = self.get_height()
        self.move_to_height(current + delta_mm, speed)

    def _position_monitor(self, target_mm: float, speed: int, tolerance: float):
        """位置监控线程：到达目标后自动停止。"""
        start_time = time.time()
        direction = 1 if target_mm > self.get_height() else -1

        while not self._stop_event.is_set():
            # 超时检查
            if time.time() - start_time > self.timeout_s:
                logger.warning(f"位置控制超时 ({self.timeout_s}s)，停止")
                self.servo.stop()
                break

            try:
                current = self.get_height()
                if current is None:
                    break
                diff = target_mm - current

                # 到达目标
                if abs(diff) <= tolerance:
                    self.servo.stop()
                    logger.info(f"到达目标高度 {current:.2f}mm (目标 {target_mm}mm)")
                    break

                # 过冲检测（方向反转说明过冲了）
                if direction > 0 and diff < 0:
                    self.servo.stop()
                    logger.info(f"上升过冲，停止 (当前 {current:.2f}mm)")
                    break
                if direction < 0 and diff > 0:
                    self.servo.stop()
                    logger.info(f"下降过冲，停止 (当前 {current:.2f}mm)")
                    break

            except SihengError as e:
                logger.error(f"位置监控异常: {e}")
                self.servo.stop()
                break

            time.sleep(0.05)

    def _stop_monitor(self):
        """停止位置监控线程。"""
        self._stop_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None
        self._target_height = None

    def is_moving_to_target(self) -> bool:
        """是否正在执行位置控制任务。"""
        return (self._monitor_thread is not None
                and self._monitor_thread.is_alive())

    # ===================== 配置 =====================

    def set_speed(self, speed: int):
        """设置默认运行速度。"""
        if not 1 <= speed <= 3000:
            raise ParameterError(f"速度 {speed} 超出范围 [1, 3000]")
        self.default_speed = speed
        logger.info(f"默认速度 = {speed} rpm")

    def set_pulses_per_mm(self, pulses_per_mm: float):
        """
        设置每毫米脉冲数（用于位置控制的高度换算）。

        计算方法:
            pulses_per_mm = 电机每圈脉冲数 / 丝杠导程(mm)
            电机每圈脉冲数 = 编码器分辨率 × 减速比

        Args:
            pulses_per_mm: 每毫米对应的脉冲数
        """
        self.pulses_per_mm = pulses_per_mm
        logger.info(f"脉冲当量 = {pulses_per_mm} pulses/mm")

    def zero_position(self):
        """
        将当前位置清零（作为高度零点）。
        注意: 需要驱动器支持位置清零功能，部分型号需要硬件支持。
        """
        # 通过写 H0D_20 绝对编码器复位选择
        try:
            self.servo.write("H0D_20", 1)
            logger.info("当前位置已清零")
        except SihengError as e:
            logger.error(f"位置清零失败: {e}")
            raise

    def __repr__(self):
        return (f"LiftController(up_direction={self.up_direction}, "
                f"default_speed={self.default_speed}, "
                f"pulses_per_mm={self.pulses_per_mm})")
