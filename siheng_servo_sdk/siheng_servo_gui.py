# -*- coding: utf-8 -*-
"""
上海四横 D-AIS48025A 伺服驱动器专用控制 GUI
================================================
基于 D-AIS48025A 参数表，Modbus 地址映射规则: HXX_YY → 0xXXYY

控制流程:
  1. 断使能 → 设 H02_00=1(速度模式)、H06_00=0(内部速度源)
  2. 使能伺服 → H32_01=1
  3. 运行 → 写 H06_03=目标转速(rpm, 负值反转)
  4. 停止 → 写 H06_03=0
  5. 急停 → 写 H0D_05=1

参数说明:
  - H0C_02: 波特率 (0~6), 出厂=5, 常见: 0=4800,1=9600,2=19200,3=38400,4=57600,5=115200
  - H0C_03: 数据格式 (0~3), 出厂=3
  - H0C_26: 高低位顺序 (0=低字在前, 1=高字在前), 出厂=1
  - H02_00: 控制模式 (0=位置,1=速度,2=转矩,...)
  - H06_03: 速度设定值 (rpm, -18000~18000), 立即生效
"""

import sys
import time
import struct
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QSpinBox, QDoubleSpinBox,
    QTextEdit, QLineEdit, QMessageBox, QTabWidget, QTableWidget,
    QTableWidgetItem, QHeaderView, QCheckBox, QSlider, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.serial import ModbusSerialClient as ModbusSerialClient


# ============================================================
#  四横 D-AIS48025A 寄存器地址表 (HXX_YY → 0xXXYY)
# ============================================================
REG = {
    # 电机参数
    "H00_11": 0x000B,  # 电机额定电流 (10mA)
    "H00_14": 0x000E,  # 电机额定转速 (rpm)
    "H00_15": 0x000F,  # 电机最大转速 (rpm)

    # 控制模式
    "H02_00": 0x0200,  # 控制模式选择 (0=位置,1=速度,2=转矩)

    # 速度控制参数
    "H06_00": 0x0600,  # 主速度指令A来源 (0=内部/键盘)
    "H06_01": 0x0601,  # 辅助速度指令B来源
    "H06_02": 0x0602,  # 速度指令选择
    "H06_03": 0x0603,  # 速度指令键盘设定值 (rpm, 负=反转) ★核心
    "H06_04": 0x0604,  # 点动速度设定值 (rpm)
    "H06_05": 0x0605,  # 加速斜坡时间 (ms)
    "H06_06": 0x0606,  # 减速斜坡时间 (ms)
    "H06_18": 0x0612,  # 速度到达信号阀值

    # 监控参数 (只读)
    "H0B_00": 0x0B00,  # 实际电机转速 (rpm, Int16)
    "H0B_02": 0x0B02,  # 内部转矩指令 (0.1%, Int16)
    "H0B_03": 0x0B03,  # DI信号监视 (UInt32, 2寄存器)
    "H0B_05": 0x0B05,  # DO信号监视
    "H0B_07": 0x0B07,  # 绝对位置计数器 (Int32, 2寄存器)
    "H0B_12": 0x0B0C,  # 平均负载率 (0.1%, Int16)
    "H0B_24": 0x0B18,  # 相电流有效值 (0.01A, Int32, 2寄存器)
    "H0B_26": 0x0B1A,  # 母线电压值 (0.1V, UInt16)
    "H0B_27": 0x0B1B,  # 模块温度值 (℃, Int16)
    "H0B_33": 0x0B21,  # 故障记录

    # 通信参数
    "H0C_00": 0x0C00,  # 伺服轴地址 (从站ID, 1~247)
    "H0C_02": 0x0C02,  # 串口波特率 (0~6)
    "H0C_03": 0x0C03,  # MODBUS数据格式 (0~3)
    "H0C_26": 0x0C1A,  # 高低位顺序 (0=低前, 1=高前)

    # 控制命令
    "H0D_00": 0x0D00,  # 软件复位
    "H0D_01": 0x0D01,  # 故障复位
    "H0D_05": 0x0D05,  # 紧急停机

    # 使能与状态
    "H32_01": 0x3201,  # 内部伺服使能 (0=断, 1=通) ★核心
    "H32_02": 0x3202,  # 伺服状态 (0~6, 只读)
}

# 伺服状态字含义
SERVO_STATUS_MAP = {
    0: "未初始化",
    1: "准备就绪",
    2: "使能中",
    3: "运行中",
    4: "故障",
    5: "急停",
    6: "报警",
}


class ModbusWorker(QThread):
    """后台 Modbus 通信线程"""
    log_signal = pyqtSignal(str, str)
    status_signal = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.client = None
        self.slave_id = 1
        self.running = True
        self.poll_interval = 0.3
        self.word_order = 1  # 0=低字在前, 1=高字在前(出厂默认)
        self.poll_enabled = False

    def connect_port(self, port, baudrate, parity, stopbits, bytesize, slave_id):
        try:
            self.client = ModbusSerialClient(
                port=port, baudrate=baudrate, parity=parity,
                stopbits=stopbits, bytesize=bytesize, timeout=1
            )
            if self.client.connect():
                self.slave_id = slave_id
                self.log_signal.emit(f"已连接 {port} @ {baudrate}, 从站 {slave_id}", "success")
                return True
            else:
                self.log_signal.emit(f"连接失败：无法打开 {port}", "error")
                return False
        except Exception as e:
            self.log_signal.emit(f"连接异常：{e}", "error")
            return False

    def disconnect_port(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
        self.log_signal.emit("已断开连接", "info")

    def read_u16(self, addr):
        if not self.client:
            return None, "未连接"
        try:
            rr = self.client.read_holding_registers(addr, count=1, slave=self.slave_id)
            if rr.isError():
                return None, f"错误: {rr}"
            return rr.registers[0], None
        except Exception as e:
            return None, f"异常: {e}"

    def read_s16(self, addr):
        v, err = self.read_u16(addr)
        if err:
            return None, err
        if v > 32767:
            v -= 65536
        return v, None

    def read_u32(self, addr):
        if not self.client:
            return None, "未连接"
        try:
            rr = self.client.read_holding_registers(addr, count=2, slave=self.slave_id)
            if rr.isError():
                return None, f"错误: {rr}"
            r0, r1 = rr.registers[0], rr.registers[1]
            if self.word_order == 1:  # 高字在前
                val = (r0 << 16) | r1
            else:  # 低字在前
                val = (r1 << 16) | r0
            return val, None
        except Exception as e:
            return None, f"异常: {e}"

    def read_s32(self, addr):
        v, err = self.read_u32(addr)
        if err:
            return None, err
        if v > 2147483647:
            v -= 4294967296
        return v, None

    def write_u16(self, addr, value):
        if not self.client:
            return False, "未连接"
        try:
            if value < 0:
                value = value & 0xFFFF
            rr = self.client.write_register(addr, value, slave=self.slave_id)
            if rr.isError():
                return False, f"错误: {rr}"
            return True, None
        except Exception as e:
            return False, f"异常: {e}"

    def write_s16(self, addr, value):
        return self.write_u16(addr, value & 0xFFFF)

    def run(self):
        while self.running:
            if self.client and self.poll_enabled:
                self._poll_status()
            time.sleep(self.poll_interval)

    def _poll_status(self):
        try:
            data = {}
            # 实际转速
            v, _ = self.read_s16(REG["H0B_00"])
            data["speed"] = v
            # 伺服状态
            v, _ = self.read_u16(REG["H32_02"])
            data["servo_status"] = v
            # 绝对位置
            v, _ = self.read_s32(REG["H0B_07"])
            data["position"] = v
            # 转矩指令
            v, _ = self.read_s16(REG["H0B_02"])
            data["torque"] = v
            # 负载率
            v, _ = self.read_s16(REG["H0B_12"])
            data["load"] = v
            # 相电流
            v, _ = self.read_s32(REG["H0B_24"])
            data["current"] = v
            # 母线电压
            v, _ = self.read_u16(REG["H0B_26"])
            data["voltage"] = v
            # 模块温度
            v, _ = self.read_s16(REG["H0B_27"])
            data["temperature"] = v
            # 故障记录
            v, _ = self.read_u16(REG["H0B_33"])
            data["fault"] = v
            self.status_signal.emit(data)
        except Exception:
            pass

    def stop(self):
        self.running = False
        self.wait(2000)


class SihengServoGUI(QMainWindow):
    """四横 D-AIS48025A 伺服驱动器专用控制界面"""

    def __init__(self):
        super().__init__()
        self.worker = ModbusWorker()
        self.worker.log_signal.connect(self.append_log)
        self.worker.status_signal.connect(self.update_status)
        self.worker.start()
        self.connected = False
        self.servo_enabled = False
        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        self.setWindowTitle("四横 D-AIS48025A 伺服驱动器控制")
        self.setMinimumSize(900, 760)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)

        main_layout.addWidget(self._create_connection_group())

        tabs = QTabWidget()
        tabs.addTab(self._create_control_tab(), "速度控制")
        tabs.addTab(self._create_status_tab(), "实时状态")
        tabs.addTab(self._create_param_tab(), "参数读写")
        main_layout.addWidget(tabs, 1)

        main_layout.addWidget(self._create_log_group(), 1)

    # ---------- 连接区 ----------
    def _create_connection_group(self):
        group = QGroupBox("串口连接")
        layout = QGridLayout(group)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self.refresh_ports)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "9600", "4800", "19200", "38400"])
        self.baud_combo.setCurrentText("57600")  # 四横驱动器实测波特率 57600

        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["N", "E", "O"])

        self.stopbits_combo = QComboBox()
        self.stopbits_combo.addItems(["1", "2"])

        self.databits_combo = QComboBox()
        self.databits_combo.addItems(["8"])
        self.databits_combo.setCurrentText("8")

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(1)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
                                       "QPushButton:hover{background:#45a049;}")
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
        layout.addWidget(QLabel("从站:"), 1, 5)
        layout.addWidget(self.slave_spin, 1, 6)
        layout.addWidget(self.btn_connect, 0, 7, 2, 1)
        return group

    # ---------- 速度控制页 ----------
    def _create_control_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 使能控制
        group_en = QGroupBox("伺服使能")
        ge = QHBoxLayout(group_en)
        self.btn_enable = QPushButton("伺服使能 (H32_01=1)")
        self.btn_enable.setMinimumHeight(40)
        self.btn_enable.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;font-size:13px;border-radius:4px;}"
                                      "QPushButton:hover{background:#1976D2;}"
                                      "QPushButton:disabled{background:#aaa;}")
        self.btn_enable.clicked.connect(self.enable_servo)

        self.btn_disable = QPushButton("伺服断使能 (H32_01=0)")
        self.btn_disable.setMinimumHeight(40)
        self.btn_disable.setStyleSheet("QPushButton{background:#9E9E9E;color:white;font-weight:bold;font-size:13px;border-radius:4px;}"
                                       "QPushButton:hover{background:#757575;}"
                                       "QPushButton:disabled{background:#aaa;}")
        self.btn_disable.clicked.connect(self.disable_servo)

        self.btn_fault_reset = QPushButton("故障复位 (H0D_01)")
        self.btn_fault_reset.setMinimumHeight(40)
        self.btn_fault_reset.setStyleSheet("QPushButton{background:#FF9800;color:white;font-weight:bold;font-size:13px;border-radius:4px;}"
                                           "QPushButton:hover{background:#F57C00;}")
        self.btn_fault_reset.clicked.connect(self.fault_reset)

        ge.addWidget(self.btn_enable)
        ge.addWidget(self.btn_disable)
        ge.addWidget(self.btn_fault_reset)
        layout.addWidget(group_en)

        # 速度设定
        group_spd = QGroupBox("速度控制 (H06_03)")
        gs = QGridLayout(group_spd)

        gs.addWidget(QLabel("目标转速:"), 0, 0)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(-3000, 3000)
        self.speed_spin.setValue(100)
        self.speed_spin.setSuffix(" rpm")
        self.speed_spin.setSingleStep(10)
        gs.addWidget(self.speed_spin, 0, 1)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(-3000, 3000)
        self.speed_slider.setValue(100)
        self.speed_slider.valueChanged.connect(self.speed_spin.setValue)
        self.speed_spin.valueChanged.connect(self.speed_slider.setValue)
        gs.addWidget(self.speed_slider, 0, 2, 1, 3)

        gs.addWidget(QLabel("加速时间:"), 1, 0)
        self.accel_spin = QSpinBox()
        self.accel_spin.setRange(0, 65535)
        self.accel_spin.setValue(500)
        self.accel_spin.setSuffix(" ms")
        gs.addWidget(self.accel_spin, 1, 1)
        gs.addWidget(QLabel("减速时间:"), 1, 2)
        self.decel_spin = QSpinBox()
        self.decel_spin.setRange(0, 65535)
        self.decel_spin.setValue(500)
        self.decel_spin.setSuffix(" ms")
        gs.addWidget(self.decel_spin, 1, 3)

        self.btn_apply_acc = QPushButton("应用加减速")
        self.btn_apply_acc.clicked.connect(self.apply_acc_dec)
        gs.addWidget(self.btn_apply_acc, 1, 4)

        # 运行控制按钮
        self.btn_run = QPushButton("▶ 运行")
        self.btn_run.setMinimumHeight(48)
        self.btn_run.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;font-size:15px;border-radius:4px;}"
                                   "QPushButton:hover{background:#43A047;}"
                                   "QPushButton:disabled{background:#aaa;}")
        self.btn_run.clicked.connect(self.run_motor)

        self.btn_stop = QPushButton("■ 停止 (速度=0)")
        self.btn_stop.setMinimumHeight(48)
        self.btn_stop.setStyleSheet("QPushButton{background:#FF9800;color:white;font-weight:bold;font-size:15px;border-radius:4px;}"
                                    "QPushButton:hover{background:#F57C00;}"
                                    "QPushButton:disabled{background:#aaa;}")
        self.btn_stop.clicked.connect(self.stop_motor)

        self.btn_emergency = QPushButton("⬛ 急停 (H0D_05=1)")
        self.btn_emergency.setMinimumHeight(48)
        self.btn_emergency.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;font-size:15px;border-radius:4px;}"
                                         "QPushButton:hover{background:#D32F2F;}"
                                         "QPushButton:disabled{background:#aaa;}")
        self.btn_emergency.clicked.connect(self.emergency_stop)

        gs.addWidget(self.btn_run, 2, 0, 1, 2)
        gs.addWidget(self.btn_stop, 2, 2, 1, 2)
        gs.addWidget(self.btn_emergency, 2, 4)
        layout.addWidget(group_spd)

        # 点动控制
        group_jog = QGroupBox("点动控制 (JOG)")
        gj = QHBoxLayout(group_jog)
        gj.addWidget(QLabel("点动速度:"))
        self.jog_speed_spin = QSpinBox()
        self.jog_speed_spin.setRange(0, 18000)
        self.jog_speed_spin.setValue(100)
        self.jog_speed_spin.setSuffix(" rpm")
        gj.addWidget(self.jog_speed_spin)

        self.btn_jog_cw = QPushButton("◀ 正转点动")
        self.btn_jog_cw.setMinimumHeight(40)
        self.btn_jog_cw.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;border-radius:4px;}")
        self.btn_jog_cw.pressed.connect(lambda: self.jog_start(1))
        self.btn_jog_cw.released.connect(self.jog_stop)

        self.btn_jog_ccw = QPushButton("反转点动 ▶")
        self.btn_jog_ccw.setMinimumHeight(40)
        self.btn_jog_ccw.setStyleSheet("QPushButton{background:#9C27B0;color:white;font-weight:bold;border-radius:4px;}")
        self.btn_jog_ccw.pressed.connect(lambda: self.jog_start(-1))
        self.btn_jog_ccw.released.connect(self.jog_stop)

        gj.addWidget(self.btn_jog_cw)
        gj.addWidget(self.btn_jog_ccw)
        layout.addWidget(group_jog)

        # 模式配置
        group_mode = QGroupBox("控制模式配置 (修改需断使能)")
        gm = QGridLayout(group_mode)
        gm.addWidget(QLabel("控制模式 H02_00:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["0 - 位置控制", "1 - 速度控制", "2 - 转矩控制"])
        self.mode_combo.setCurrentIndex(1)
        gm.addWidget(self.mode_combo, 0, 1)
        self.btn_set_mode = QPushButton("设置模式")
        self.btn_set_mode.clicked.connect(self.set_control_mode)
        gm.addWidget(self.btn_set_mode, 0, 2)

        gm.addWidget(QLabel("速度源 H06_00:"), 1, 0)
        self.source_combo = QComboBox()
        self.source_combo.addItems(["0 - 内部/键盘", "1 - 模拟量", "2 - 通讯"])
        self.source_combo.setCurrentIndex(0)
        gm.addWidget(self.source_combo, 1, 1)
        self.btn_set_source = QPushButton("设置速度源")
        self.btn_set_source.clicked.connect(self.set_speed_source)
        gm.addWidget(self.btn_set_source, 1, 2)
        layout.addWidget(group_mode)

        layout.addStretch()
        return tab

    # ---------- 实时状态页 ----------
    def _create_status_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("实时监控 (每 300ms 刷新)")
        g = QGridLayout(group)

        self.lbl_servo_status = self._make_label("未连接", "#f44336")
        self.lbl_speed = self._make_label("-- rpm")
        self.lbl_position = self._make_label("--")
        self.lbl_torque = self._make_label("-- %")
        self.lbl_load = self._make_label("-- %")
        self.lbl_current = self._make_label("-- A")
        self.lbl_voltage = self._make_label("-- V")
        self.lbl_temp = self._make_label("-- ℃")
        self.lbl_fault = self._make_label("--")

        g.addWidget(QLabel("伺服状态:"), 0, 0); g.addWidget(self.lbl_servo_status, 0, 1)
        g.addWidget(QLabel("故障代码:"), 0, 2); g.addWidget(self.lbl_fault, 0, 3)
        g.addWidget(QLabel("实际转速:"), 1, 0); g.addWidget(self.lbl_speed, 1, 1)
        g.addWidget(QLabel("绝对位置:"), 1, 2); g.addWidget(self.lbl_position, 1, 3)
        g.addWidget(QLabel("转矩指令:"), 2, 0); g.addWidget(self.lbl_torque, 2, 1)
        g.addWidget(QLabel("负载率:"), 2, 2); g.addWidget(self.lbl_load, 2, 3)
        g.addWidget(QLabel("相电流:"), 3, 0); g.addWidget(self.lbl_current, 3, 1)
        g.addWidget(QLabel("母线电压:"), 3, 2); g.addWidget(self.lbl_voltage, 3, 3)
        g.addWidget(QLabel("模块温度:"), 4, 0); g.addWidget(self.lbl_temp, 4, 1)

        self.btn_poll = QPushButton("开始监控")
        self.btn_poll.setCheckable(True)
        self.btn_poll.setMinimumHeight(36)
        self.btn_poll.clicked.connect(self.toggle_poll)
        g.addWidget(self.btn_poll, 4, 2, 1, 2)

        layout.addWidget(group)
        layout.addStretch()

        # 单次读取按钮
        group2 = QGroupBox("单次读取")
        g2 = QHBoxLayout(group2)
        btn_read_once = QPushButton("读取一次状态")
        btn_read_once.clicked.connect(lambda: self.worker.status_signal.emit({}) or self.read_status_once())
        g2.addWidget(btn_read_once)
        layout.addWidget(group2)

        return tab

    def _make_label(self, text, color="#333"):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{color};font-weight:bold;font-size:14px;")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumWidth(140)
        return lbl

    # ---------- 参数读写页 ----------
    def _create_param_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("参数读写 (HXX_YY 格式)")
        g = QGridLayout(group)

        g.addWidget(QLabel("参数号:"), 0, 0)
        self.param_name = QLineEdit()
        self.param_name.setPlaceholderText("例如: H06_03 或 H0B_00")
        g.addWidget(self.param_name, 0, 1)

        g.addWidget(QLabel("Modbus地址:"), 0, 2)
        self.param_addr = QLineEdit()
        self.param_addr.setReadOnly(True)
        g.addWidget(self.param_addr, 0, 3)

        g.addWidget(QLabel("数值:"), 1, 0)
        self.param_value = QLineEdit()
        self.param_value.setPlaceholderText("支持十进制或0x开头十六进制")
        g.addWidget(self.param_value, 1, 1)

        self.param_type = QComboBox()
        self.param_type.addItems(["UInt16", "Int16", "UInt32(2寄存器)", "Int32(2寄存器)"])
        g.addWidget(self.param_type, 1, 2)

        btn_calc = QPushButton("解析地址")
        btn_calc.clicked.connect(self.parse_param_addr)
        g.addWidget(btn_calc, 1, 3)

        btn_read = QPushButton("读取")
        btn_read.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;}")
        btn_read.clicked.connect(self.read_param)
        g.addWidget(btn_read, 2, 0, 1, 2)

        btn_write = QPushButton("写入")
        btn_write.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;}")
        btn_write.clicked.connect(self.write_param)
        g.addWidget(btn_write, 2, 2, 1, 2)

        layout.addWidget(group)

        # 常用参数快捷按钮
        group2 = QGroupBox("常用参数快捷读取")
        g2 = QGridLayout(group2)
        quick_params = [
            ("H0C_00 从站地址", "H0C_00"), ("H0C_02 波特率", "H0C_02"),
            ("H0C_03 数据格式", "H0C_03"), ("H02_00 控制模式", "H02_00"),
            ("H06_00 速度源", "H06_00"), ("H06_03 当前速度设定", "H06_03"),
            ("H32_01 使能状态", "H32_01"), ("H32_02 伺服状态", "H32_02"),
            ("H0B_00 实际转速", "H0B_00"), ("H0B_26 母线电压", "H0B_26"),
        ]
        for i, (label, param) in enumerate(quick_params):
            btn = QPushButton(label)
            btn.clicked.connect(lambda checked, p=param: self.quick_read(p))
            g2.addWidget(btn, i // 3, i % 3)
        layout.addWidget(group2)
        layout.addStretch()
        return tab

    # ---------- 日志区 ----------
    def _create_log_group(self):
        group = QGroupBox("操作日志")
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
        import serial.tools.list_ports
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.append_log("未检测到串口", "warning")

    def toggle_connection(self):
        if not self.connected:
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "提示", "请选择串口")
                return
            ok = self.worker.connect_port(
                port=port,
                baudrate=int(self.baud_combo.currentText()),
                parity=self.parity_combo.currentText(),
                stopbits=int(self.stopbits_combo.currentText()),
                bytesize=int(self.databits_combo.currentText()),
                slave_id=self.slave_spin.value(),
            )
            if ok:
                self.connected = True
                self.btn_connect.setText("断开")
                self.btn_connect.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
                                               "QPushButton:hover{background:#D32F2F;}")
                self._set_controls_enabled(True)
                # 读取高低位顺序设置
                order, err = self.worker.read_u16(REG["H0C_26"])
                if not err and order is not None:
                    self.worker.word_order = order
                    self.append_log(f"高低位顺序 H0C_26 = {order} ({'高字在前' if order else '低字在前'})", "info")
        else:
            self.worker.disconnect_port()
            self.connected = False
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
                                           "QPushButton:hover{background:#45a049;}")
            self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled):
        for w in [self.btn_enable, self.btn_disable, self.btn_fault_reset,
                  self.btn_run, self.btn_stop, self.btn_emergency,
                  self.btn_jog_cw, self.btn_jog_ccw, self.btn_set_mode,
                  self.btn_set_source, self.btn_apply_acc, self.btn_poll]:
            w.setEnabled(enabled)

    # ---------- 使能控制 ----------
    def enable_servo(self):
        ok, err = self.worker.write_u16(REG["H32_01"], 1)
        if ok:
            self.servo_enabled = True
            self.append_log("伺服使能 H32_01=1", "success")
        else:
            self.append_log(f"使能失败: {err}", "error")

    def disable_servo(self):
        ok, err = self.worker.write_u16(REG["H32_01"], 0)
        if ok:
            self.servo_enabled = False
            self.append_log("伺服断使能 H32_01=0", "warning")
        else:
            self.append_log(f"断使能失败: {err}", "error")

    def fault_reset(self):
        ok, err = self.worker.write_u16(REG["H0D_01"], 1)
        if ok:
            self.append_log("故障复位 H0D_01=1", "success")
        else:
            self.append_log(f"故障复位失败: {err}", "error")

    # ---------- 速度控制 ----------
    def run_motor(self):
        speed = self.speed_spin.value()
        ok, err = self.worker.write_s16(REG["H06_03"], speed)
        if ok:
            self.append_log(f"▶ 运行 H06_03={speed} rpm", "success")
        else:
            self.append_log(f"运行失败: {err}", "error")

    def stop_motor(self):
        ok, err = self.worker.write_s16(REG["H06_03"], 0)
        if ok:
            self.append_log("■ 停止 H06_03=0", "warning")
        else:
            self.append_log(f"停止失败: {err}", "error")

    def emergency_stop(self):
        ok, err = self.worker.write_u16(REG["H0D_05"], 1)
        if ok:
            self.append_log("⬛ 急停 H0D_05=1", "error")
        else:
            self.append_log(f"急停失败: {err}", "error")

    def apply_acc_dec(self):
        accel = self.accel_spin.value()
        decel = self.decel_spin.value()
        ok1, _ = self.worker.write_u16(REG["H06_05"], accel)
        ok2, _ = self.worker.write_u16(REG["H06_06"], decel)
        if ok1 and ok2:
            self.append_log(f"加减速时间: 加速={accel}ms 减速={decel}ms", "success")

    # ---------- 点动 ----------
    def jog_start(self, direction):
        speed = self.jog_speed_spin.value() * direction
        self.worker.write_u16(REG["H06_04"], abs(speed))
        self.worker.write_s16(REG["H06_03"], speed)
        self.append_log(f"点动 {'正转' if direction > 0 else '反转'} {abs(speed)} rpm", "info")

    def jog_stop(self):
        self.worker.write_s16(REG["H06_03"], 0)
        self.append_log("点动停止", "info")

    # ---------- 模式配置 ----------
    def set_control_mode(self):
        mode = self.mode_combo.currentIndex()
        ok, err = self.worker.write_u16(REG["H02_00"], mode)
        if ok:
            self.append_log(f"控制模式 H02_00={mode} ({['位置','速度','转矩'][mode]})", "success")
        else:
            self.append_log(f"设置模式失败: {err}", "error")

    def set_speed_source(self):
        src = self.source_combo.currentIndex()
        ok, err = self.worker.write_u16(REG["H06_00"], src)
        if ok:
            self.append_log(f"速度源 H06_00={src}", "success")
        else:
            self.append_log(f"设置速度源失败: {err}", "error")

    # ---------- 状态监控 ----------
    def toggle_poll(self):
        self.worker.poll_enabled = self.btn_poll.isChecked()
        self.btn_poll.setText("停止监控" if self.btn_poll.isChecked() else "开始监控")

    def read_status_once(self):
        """单次读取状态"""
        if not self.connected:
            return
        data = {}
        v, _ = self.worker.read_s16(REG["H0B_00"]); data["speed"] = v
        v, _ = self.worker.read_u16(REG["H32_02"]); data["servo_status"] = v
        v, _ = self.worker.read_s32(REG["H0B_07"]); data["position"] = v
        v, _ = self.worker.read_s16(REG["H0B_02"]); data["torque"] = v
        v, _ = self.worker.read_s16(REG["H0B_12"]); data["load"] = v
        v, _ = self.worker.read_s32(REG["H0B_24"]); data["current"] = v
        v, _ = self.worker.read_u16(REG["H0B_26"]); data["voltage"] = v
        v, _ = self.worker.read_s16(REG["H0B_27"]); data["temperature"] = v
        v, _ = self.worker.read_u16(REG["H0B_33"]); data["fault"] = v
        self.update_status(data)

    def update_status(self, data):
        if not data:
            return
        if "servo_status" in data and data["servo_status"] is not None:
            status = data["servo_status"]
            text = SERVO_STATUS_MAP.get(status, f"未知({status})")
            color = "#4CAF50" if status == 3 else ("#f44336" if status in [4, 5] else "#FF9800")
            self.lbl_servo_status.setText(text)
            self.lbl_servo_status.setStyleSheet(f"color:{color};font-weight:bold;font-size:14px;")
        if "speed" in data and data["speed"] is not None:
            self.lbl_speed.setText(f"{data['speed']} rpm")
        if "position" in data and data["position"] is not None:
            self.lbl_position.setText(f"{data['position']}")
        if "torque" in data and data["torque"] is not None:
            self.lbl_torque.setText(f"{data['torque'] / 10:.1f} %")
        if "load" in data and data["load"] is not None:
            self.lbl_load.setText(f"{data['load'] / 10:.1f} %")
        if "current" in data and data["current"] is not None:
            self.lbl_current.setText(f"{data['current'] / 100:.2f} A")
        if "voltage" in data and data["voltage"] is not None:
            self.lbl_voltage.setText(f"{data['voltage'] / 10:.1f} V")
        if "temperature" in data and data["temperature"] is not None:
            self.lbl_temp.setText(f"{data['temperature']} ℃")
        if "fault" in data and data["fault"] is not None:
            f = data["fault"]
            self.lbl_fault.setText("无故障" if f == 0 else f"故障码: {f}")

    # ---------- 参数读写 ----------
    def parse_param_addr(self):
        name = self.param_name.text().strip().upper()
        if not name:
            return
        if name in REG:
            addr = REG[name]
            self.param_addr.setText(f"0x{addr:04X} ({addr})")
        else:
            # 尝试解析 HXX_YY 格式
            if name.startswith("H") and "_" in name:
                parts = name[1:].split("_")
                if len(parts) == 2:
                    try:
                        hi = int(parts[0], 16)
                        lo = int(parts[1], 16)
                        addr = (hi << 8) | lo
                        self.param_addr.setText(f"0x{addr:04X} ({addr})")
                    except ValueError:
                        QMessageBox.warning(self, "错误", f"无法解析参数号: {name}")

    def read_param(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接")
            return
        name = self.param_name.text().strip().upper()
        if not name:
            return
        self.parse_param_addr()
        if name in REG:
            addr = REG[name]
        else:
            addr_text = self.param_addr.text()
            if "0x" in addr_text:
                addr = int(addr_text.split("0x")[1].split(" ")[0], 16)
            else:
                QMessageBox.warning(self, "错误", "无法解析地址")
                return
        typ = self.param_type.currentText()
        if "32" in typ:
            if "Int" in typ:
                val, err = self.worker.read_s32(addr)
            else:
                val, err = self.worker.read_u32(addr)
        else:
            if "Int" in typ:
                val, err = self.worker.read_s16(addr)
            else:
                val, err = self.worker.read_u16(addr)
        if err:
            self.param_value.setText(f"错误: {err}")
            self.append_log(f"读 {name} 失败: {err}", "error")
        else:
            self.param_value.setText(str(val))
            self.append_log(f"读 {name} (0x{addr:04X}) = {val}", "success")

    def write_param(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接")
            return
        name = self.param_name.text().strip().upper()
        text = self.param_value.text().strip()
        if not name or not text:
            return
        self.parse_param_addr()
        if name in REG:
            addr = REG[name]
        else:
            addr_text = self.param_addr.text()
            if "0x" in addr_text:
                addr = int(addr_text.split("0x")[1].split(" ")[0], 16)
            else:
                return
        try:
            val = int(text, 0)
        except ValueError:
            QMessageBox.warning(self, "错误", "数值格式错误")
            return
        typ = self.param_type.currentText()
        if "32" in typ:
            ok, err = self.worker.write_u16(addr, (val >> 16) & 0xFFFF)
            self.worker.write_u16(addr + 1, val & 0xFFFF)
        else:
            ok, err = self.worker.write_u16(addr, val & 0xFFFF)
        if ok:
            self.append_log(f"写 {name} (0x{addr:04X}) = {val} 成功", "success")
        else:
            self.append_log(f"写 {name} 失败: {err}", "error")

    def quick_read(self, param):
        self.param_name.setText(param)
        self.parse_param_addr()
        self.read_param()

    # ---------- 日志 ----------
    def append_log(self, msg, level="info"):
        colors = {"info": "#333", "success": "#4CAF50", "warning": "#FF9800", "error": "#f44336"}
        t = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#888;">[{t}]</span> '
                             f'<span style="color:{colors.get(level, "#333")};">{msg}</span>')

    def closeEvent(self, event):
        self.worker.stop()
        if self.connected:
            self.worker.disconnect_port()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = SihengServoGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
