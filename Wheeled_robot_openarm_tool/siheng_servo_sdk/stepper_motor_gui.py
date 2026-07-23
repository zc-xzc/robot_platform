# -*- coding: utf-8 -*-
"""
步进电机 RS485 控制器 GUI
======================================
通过 USB-RS485 适配器连接步进电机控制器，使用 Modbus RTU 协议进行通信。
支持：串口连接管理、速度调节、方向控制、点动运行、相对/绝对位置运动、急停。

⚠️  重要说明：
    不同厂家的控制器寄存器地址不同，请务必对照您控制器的使用手册，
    修改下方 REGISTERS 配置区中的寄存器地址（保持功能码类型一致即可）。
    若控制器不支持 Modbus RTU，请改为厂商私有协议实现 _send_command 方法。
"""

import sys
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QSpinBox, QDoubleSpinBox,
    QTextEdit, QLineEdit, QMessageBox, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor

# 使用 pymodbus 3.x 客户端 API
try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    # 兼容旧版本
    from pymodbus.client.serial import ModbusSerialClient as ModbusSerialClient


# ============================================================
#  寄存器配置区 —— 请根据控制器手册修改以下地址
#  功能码: 0=线圈(FC01/FC05), 1=离散输入(FC02), 3=保持寄存器(FC03/FC06/FC16), 4=输入寄存器(FC04)
# ============================================================
REGISTERS = {
    # 控制寄存器（写）
    "control_word":      {"addr": 0x0000, "fc": 3},  # 控制字：启动/停止/使能
    "speed":             {"addr": 0x0001, "fc": 3},  # 速度设定（单位由控制器定义，常见为 RPM 或 Hz）
    "target_position":   {"addr": 0x0002, "fc": 3},  # 目标位置（脉冲数或角度）
    "direction":         {"addr": 0x0003, "fc": 3},  # 方向：0=正转(CW), 1=反转(CCW)
    "acceleration":      {"addr": 0x0004, "fc": 3},  # 加速度
    "deceleration":      {"addr": 0x0005, "fc": 3},  # 减速度

    # 状态寄存器（读）
    "status_word":       {"addr": 0x0000, "fc": 3},  # 状态字
    "current_position":  {"addr": 0x0001, "fc": 3},  # 当前位置
    "current_speed":     {"addr": 0x0002, "fc": 3},  # 当前速度
}

# 控制字位定义（常见定义，请按手册调整）
CONTROL_BITS = {
    "enable":   0x0001,   # 使能
    "start":    0x0002,   # 启动
    "stop":     0x0004,   # 停止
    "emergency":0x0008,   # 急停
    "reset":    0x0010,   # 故障复位
    "jog":      0x0020,   # 点动模式
    "relative": 0x0040,   # 相对定位
    "absolute": 0x0080,   # 绝对定位
}


class ModbusWorker(QThread):
    """后台 Modbus 通信线程，负责实际读写操作，避免阻塞 UI。"""
    log_signal = pyqtSignal(str, str)        # (消息, 级别)
    status_signal = pyqtSignal(dict)         # 状态字典

    def __init__(self, parent=None):
        super().__init__(parent)
        self.client = None
        self.running = True
        self.poll_interval = 0.5              # 状态轮询间隔(秒)
        self.slave_id = 1

    def connect_port(self, port, baudrate, parity, stopbits, bytesize, slave_id):
        """连接串口"""
        try:
            self.client = ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                parity=parity,
                stopbits=stopbits,
                bytesize=bytesize,
                timeout=1,
            )
            if self.client.connect():
                self.slave_id = slave_id
                self.log_signal.emit(f"已连接 {port} (波特率 {baudrate}, 从站地址 {slave_id})", "success")
                return True
            else:
                self.log_signal.emit(f"连接失败：{port} 无法打开", "error")
                return False
        except Exception as e:
            self.log_signal.emit(f"连接异常：{e}", "error")
            return False

    def disconnect_port(self):
        """断开连接"""
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
        self.log_signal.emit("已断开连接", "info")

    def write_register(self, name, value):
        """写单个保持寄存器"""
        if not self.client:
            self.log_signal.emit("未连接设备", "error")
            return False
        reg = REGISTERS[name]
        try:
            # 将负数转为 16 位无符号
            if value < 0:
                value = value & 0xFFFF
            rr = self.client.write_register(reg["addr"], value, slave=self.slave_id)
            if rr.isError():
                self.log_signal.emit(f"写寄存器 {name}(0x{reg['addr']:04X}) 失败: {rr}", "error")
                return False
            return True
        except Exception as e:
            self.log_signal.emit(f"写寄存器 {name} 异常: {e}", "error")
            return False

    def write_registers(self, name, values):
        """写多个保持寄存器"""
        if not self.client:
            return False
        reg = REGISTERS[name]
        try:
            rr = self.client.write_registers(reg["addr"], values, slave=self.slave_id)
            return not rr.isError()
        except Exception as e:
            self.log_signal.emit(f"写多寄存器异常: {e}", "error")
            return False

    def read_register(self, name):
        """读单个保持寄存器，返回值或 None"""
        if not self.client:
            return None
        reg = REGISTERS[name]
        try:
            rr = self.client.read_holding_registers(reg["addr"], count=1, slave=self.slave_id)
            if rr.isError():
                return None
            return rr.registers[0]
        except Exception:
            return None

    def read_registers(self, name, count):
        """读多个保持寄存器"""
        if not self.client:
            return None
        reg = REGISTERS[name]
        try:
            rr = self.client.read_holding_registers(reg["addr"], count=count, slave=self.slave_id)
            if rr.isError():
                return None
            return rr.registers
        except Exception:
            return None

    def run(self):
        """线程主循环：轮询读取状态"""
        while self.running:
            if self.client:
                self._poll_status()
            time.sleep(self.poll_interval)

    def _poll_status(self):
        """轮询读取当前状态"""
        try:
            data = {}
            sw = self.read_register("status_word")
            data["status_word"] = sw
            pos = self.read_register("current_position")
            data["current_position"] = pos
            spd = self.read_register("current_speed")
            data["current_speed"] = spd
            self.status_signal.emit(data)
        except Exception:
            pass

    def stop(self):
        self.running = False
        self.wait(2000)


class StepperMotorGUI(QMainWindow):
    """步进电机控制主界面"""

    def __init__(self):
        super().__init__()
        self.worker = ModbusWorker()
        self.worker.log_signal.connect(self.append_log)
        self.worker.status_signal.connect(self.update_status)
        self.worker.start()

        # 状态保持
        self.connected = False
        self.current_direction = 0       # 0=CW, 1=CCW
        self.jog_active = False

        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        self.setWindowTitle("步进电机 RS485 控制器")
        self.setMinimumSize(820, 640)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(10)

        main_layout.addWidget(self._create_connection_group())
        main_layout.addWidget(self._create_motion_group())
        main_layout.addWidget(self._create_status_group())
        main_layout.addWidget(self._create_log_group(), 1)

        self._set_controls_enabled(False)

    # ---------- 连接区 ----------
    def _create_connection_group(self):
        group = QGroupBox("① 串口连接")
        layout = QGridLayout(group)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(140)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self.refresh_ports)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["4800", "9600", "19200", "38400", "57600", "115200", "256000"])
        self.baud_combo.setCurrentText("9600")

        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["N (无)", "E (偶)", "O (奇)"])

        self.stopbits_combo = QComboBox()
        self.stopbits_combo.addItems(["1", "1.5", "2"])
        self.stopbits_combo.setCurrentText("1")

        self.databits_combo = QComboBox()
        self.databits_combo.addItems(["7", "8"])
        self.databits_combo.setCurrentText("8")

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(1)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setMinimumHeight(32)
        self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;}"
                                       "QPushButton:hover{background:#45a049;}"
                                       "QPushButton:disabled{background:#aaaaaa;}")
        self.btn_connect.clicked.connect(self.toggle_connection)

        layout.addWidget(QLabel("串口:"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(btn_refresh, 0, 2)
        layout.addWidget(QLabel("波特率:"), 0, 3)
        layout.addWidget(self.baud_combo, 0, 4)
        layout.addWidget(QLabel("校验:"), 0, 5)
        layout.addWidget(self.parity_combo, 0, 6)

        layout.addWidget(QLabel("数据位:"), 1, 0)
        layout.addWidget(self.databits_combo, 1, 1)
        layout.addWidget(QLabel("停止位:"), 1, 3)
        layout.addWidget(self.stopbits_combo, 1, 4)
        layout.addWidget(QLabel("从站地址:"), 1, 5)
        layout.addWidget(self.slave_spin, 1, 6)

        layout.addWidget(self.btn_connect, 0, 7, 2, 1)
        return group

    # ---------- 运动控制区 ----------
    def _create_motion_group(self):
        group = QGroupBox("② 运动控制")
        layout = QVBoxLayout(group)

        # 速度 & 方向
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("速度 (RPM):"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0, 100000)
        self.speed_spin.setValue(60)
        self.speed_spin.setDecimals(1)
        self.speed_spin.setSingleStep(10)
        self.speed_spin.setSuffix(" RPM")
        self.speed_spin.valueChanged.connect(self.on_speed_changed)
        row1.addWidget(self.speed_spin)

        row1.addSpacing(20)
        row1.addWidget(QLabel("方向:"))
        self.dir_combo = QComboBox()
        self.dir_combo.addItems(["正转 CW (↻)", "反转 CCW (↺)"])
        self.dir_combo.currentIndexChanged.connect(self.on_direction_changed)
        row1.addWidget(self.dir_combo)

        row1.addSpacing(20)
        row1.addWidget(QLabel("加速度:"))
        self.accel_spin = QSpinBox()
        self.accel_spin.setRange(0, 65535)
        self.accel_spin.setValue(100)
        row1.addWidget(self.accel_spin)

        row1.addStretch()
        layout.addLayout(row1)

        # 启动/停止/急停
        row2 = QHBoxLayout()
        self.btn_start = QPushButton("▶ 启动连续运行")
        self.btn_start.setMinimumHeight(40)
        self.btn_start.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;font-size:14px;border-radius:4px;}"
                                     "QPushButton:hover{background:#1976D2;}"
                                     "QPushButton:disabled{background:#aaaaaa;}")
        self.btn_start.clicked.connect(self.start_continuous)

        self.btn_stop = QPushButton("■ 停止")
        self.btn_stop.setMinimumHeight(40)
        self.btn_stop.setStyleSheet("QPushButton{background:#FF9800;color:white;font-weight:bold;font-size:14px;border-radius:4px;}"
                                    "QPushButton:hover{background:#F57C00;}"
                                    "QPushButton:disabled{background:#aaaaaa;}")
        self.btn_stop.clicked.connect(self.stop_motion)

        self.btn_emergency = QPushButton("⬛ 急停")
        self.btn_emergency.setMinimumHeight(40)
        self.btn_emergency.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;font-size:14px;border-radius:4px;}"
                                         "QPushButton:hover{background:#d32f2f;}"
                                         "QPushButton:disabled{background:#aaaaaa;}")
        self.btn_emergency.clicked.connect(self.emergency_stop)

        row2.addWidget(self.btn_start)
        row2.addWidget(self.btn_stop)
        row2.addWidget(self.btn_emergency)
        layout.addLayout(row2)

        # 点动控制
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("点动步数:"))
        self.jog_steps_spin = QSpinBox()
        self.jog_steps_spin.setRange(1, 1000000)
        self.jog_steps_spin.setValue(1000)
        self.jog_steps_spin.setSuffix(" 脉冲")
        row3.addWidget(self.jog_steps_spin)

        self.btn_jog_cw = QPushButton("◀ 点动正转")
        self.btn_jog_cw.setMinimumHeight(36)
        self.btn_jog_cw.pressed.connect(lambda: self.jog_motion(0))
        self.btn_jog_cw.released.connect(self.stop_motion)

        self.btn_jog_ccw = QPushButton("点动反转 ▶")
        self.btn_jog_ccw.setMinimumHeight(36)
        self.btn_jog_ccw.pressed.connect(lambda: self.jog_motion(1))
        self.btn_jog_ccw.released.connect(self.stop_motion)

        row3.addWidget(self.btn_jog_cw)
        row3.addWidget(self.btn_jog_ccw)
        layout.addLayout(row3)

        # 定位控制
        row4 = QHBoxLayout()
        row4.addWidget(QLabel("目标位置:"))
        self.position_spin = QSpinBox()
        self.position_spin.setRange(-2147483647, 2147483647)
        self.position_spin.setValue(0)
        self.position_spin.setSuffix(" 脉冲")
        row4.addWidget(self.position_spin)

        self.btn_move_relative = QPushButton("相对移动")
        self.btn_move_relative.setMinimumHeight(36)
        self.btn_move_relative.clicked.connect(self.move_relative)

        self.btn_move_absolute = QPushButton("绝对定位")
        self.btn_move_absolute.setMinimumHeight(36)
        self.btn_move_absolute.clicked.connect(self.move_absolute)

        self.btn_home = QPushButton("⌂ 回零")
        self.btn_home.setMinimumHeight(36)
        self.btn_home.clicked.connect(self.go_home)

        row4.addWidget(self.btn_move_relative)
        row4.addWidget(self.btn_move_absolute)
        row4.addWidget(self.btn_home)
        layout.addLayout(row4)

        return group

    # ---------- 状态显示区 ----------
    def _create_status_group(self):
        group = QGroupBox("③ 实时状态")
        layout = QGridLayout(group)

        self.lbl_status = self._make_status_label("未运行")
        self.lbl_position = self._make_status_label("--")
        self.lbl_speed = self._make_status_label("--")
        self.lbl_connection = self._make_status_label("未连接", color="#f44336")

        layout.addWidget(QLabel("连接状态:"), 0, 0)
        layout.addWidget(self.lbl_connection, 0, 1)
        layout.addWidget(QLabel("运行状态:"), 0, 2)
        layout.addWidget(self.lbl_status, 0, 3)
        layout.addWidget(QLabel("当前位置:"), 1, 0)
        layout.addWidget(self.lbl_position, 1, 1)
        layout.addWidget(QLabel("当前速度:"), 1, 2)
        layout.addWidget(self.lbl_speed, 1, 3)
        return group

    def _make_status_label(self, text, color="#333333"):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{color};font-weight:bold;font-size:13px;")
        lbl.setMinimumWidth(120)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return lbl

    # ---------- 日志区 ----------
    def _create_log_group(self):
        group = QGroupBox("④ 操作日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        layout.addWidget(self.log_text)

        btn_clear = QPushButton("清空日志")
        btn_clear.clicked.connect(self.log_text.clear)
        layout.addWidget(btn_clear, alignment=Qt.AlignRight)
        return group

    # ===================== 功能实现 =====================

    def refresh_ports(self):
        """刷新可用串口列表"""
        import serial.tools.list_ports
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.append_log("未检测到可用串口", "warning")

    def toggle_connection(self):
        if not self.connected:
            self.do_connect()
        else:
            self.worker.disconnect_port()
            self.connected = False
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;}"
                                           "QPushButton:hover{background:#45a049;}")
            self.lbl_connection.setText("未连接")
            self.lbl_connection.setStyleSheet("color:#f44336;font-weight:bold;font-size:13px;")
            self._set_controls_enabled(False)

    def do_connect(self):
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "提示", "请先选择串口")
            return
        parity_map = {"N (无)": "N", "E (偶)": "E", "O (奇)": "O"}
        ok = self.worker.connect_port(
            port=port,
            baudrate=int(self.baud_combo.currentText()),
            parity=parity_map[self.parity_combo.currentText()],
            stopbits=float(self.stopbits_combo.currentText()),
            bytesize=int(self.databits_combo.currentText()),
            slave_id=self.slave_spin.value(),
        )
        if ok:
            self.connected = True
            self.btn_connect.setText("断开")
            self.btn_connect.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;border-radius:4px;}"
                                           "QPushButton:hover{background:#d32f2f;}")
            self.lbl_connection.setText("已连接")
            self.lbl_connection.setStyleSheet("color:#4CAF50;font-weight:bold;font-size:13px;")
            self._set_controls_enabled(True)

    def _set_controls_enabled(self, enabled):
        for w in [self.btn_start, self.btn_stop, self.btn_emergency,
                  self.btn_jog_cw, self.btn_jog_ccw, self.btn_move_relative,
                  self.btn_move_absolute, self.btn_home, self.speed_spin,
                  self.dir_combo, self.accel_spin, self.jog_steps_spin,
                  self.position_spin]:
            w.setEnabled(enabled)

    # ---------- 速度/方向变更 ----------
    def on_speed_changed(self, val):
        if self.connected:
            self.worker.write_register("speed", int(val))
            self.append_log(f"速度设置为 {val} RPM", "info")

    def on_direction_changed(self, idx):
        self.current_direction = idx
        if self.connected:
            self.worker.write_register("direction", idx)
            self.append_log(f"方向: {'正转 CW' if idx == 0 else '反转 CCW'}", "info")

    # ---------- 运动控制 ----------
    def start_continuous(self):
        """启动连续运行"""
        self.worker.write_register("speed", int(self.speed_spin.value()))
        self.worker.write_register("direction", self.current_direction)
        self.worker.write_register("control_word", CONTROL_BITS["enable"] | CONTROL_BITS["start"])
        self.append_log("▶ 启动连续运行", "success")
        self.lbl_status.setText("运行中")
        self.lbl_status.setStyleSheet("color:#4CAF50;font-weight:bold;font-size:13px;")

    def stop_motion(self):
        """停止运动"""
        self.worker.write_register("control_word", CONTROL_BITS["stop"])
        self.append_log("■ 停止", "warning")
        self.lbl_status.setText("已停止")
        self.lbl_status.setStyleSheet("color:#FF9800;font-weight:bold;font-size:13px;")

    def emergency_stop(self):
        """急停"""
        self.worker.write_register("control_word", CONTROL_BITS["emergency"])
        self.append_log("⬛ 急停!", "error")
        self.lbl_status.setText("急停")
        self.lbl_status.setStyleSheet("color:#f44336;font-weight:bold;font-size:13px;")

    def jog_motion(self, direction):
        """点动运动"""
        steps = self.jog_steps_spin.value()
        self.worker.write_register("speed", int(self.speed_spin.value()))
        self.worker.write_register("direction", direction)
        self.worker.write_register("target_position", steps)
        self.worker.write_register("control_word",
                                   CONTROL_BITS["enable"] | CONTROL_BITS["jog"] | CONTROL_BITS["relative"])
        self.append_log(f"点动 {'正转' if direction == 0 else '反转'} {steps} 脉冲", "info")

    def move_relative(self):
        """相对移动"""
        steps = self.position_spin.value()
        self.worker.write_register("speed", int(self.speed_spin.value()))
        self.worker.write_register("target_position", abs(steps))
        self.worker.write_register("direction", 0 if steps >= 0 else 1)
        self.worker.write_register("control_word",
                                   CONTROL_BITS["enable"] | CONTROL_BITS["relative"] | CONTROL_BITS["start"])
        self.append_log(f"相对移动 {steps} 脉冲", "info")

    def move_absolute(self):
        """绝对定位"""
        pos = self.position_spin.value()
        self.worker.write_register("target_position", pos)
        self.worker.write_register("control_word",
                                   CONTROL_BITS["enable"] | CONTROL_BITS["absolute"] | CONTROL_BITS["start"])
        self.append_log(f"绝对定位到 {pos}", "info")

    def go_home(self):
        """回零"""
        self.worker.write_register("control_word", CONTROL_BITS["reset"])
        self.append_log("⌂ 回零指令已发送", "info")

    # ---------- 状态更新 ----------
    def update_status(self, data):
        if "current_position" in data and data["current_position"] is not None:
            # 处理负数（16位补码）
            pos = data["current_position"]
            if pos > 32767:
                pos -= 65536
            self.lbl_position.setText(f"{pos}")
        if "current_speed" in data and data["current_speed"] is not None:
            self.lbl_speed.setText(f"{data['current_speed']}")
        if "status_word" in data and data["status_word"] is not None:
            sw = data["status_word"]
            if sw & CONTROL_BITS["start"]:
                self.lbl_status.setText("运行中")
                self.lbl_status.setStyleSheet("color:#4CAF50;font-weight:bold;font-size:13px;")
            elif sw & CONTROL_BITS["emergency"]:
                self.lbl_status.setText("急停状态")
                self.lbl_status.setStyleSheet("color:#f44336;font-weight:bold;font-size:13px;")
            else:
                self.lbl_status.setText("停止")
                self.lbl_status.setStyleSheet("color:#FF9800;font-weight:bold;font-size:13px;")

    # ---------- 日志 ----------
    def append_log(self, msg, level="info"):
        color_map = {
            "info":    "#333333",
            "success": "#4CAF50",
            "warning": "#FF9800",
            "error":   "#f44336",
        }
        color = color_map.get(level, "#333333")
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#888;">[{timestamp}]</span> '
                             f'<span style="color:{color};">{msg}</span>')

    def closeEvent(self, event):
        self.worker.stop()
        if self.connected:
            self.worker.disconnect_port()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = StepperMotorGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
