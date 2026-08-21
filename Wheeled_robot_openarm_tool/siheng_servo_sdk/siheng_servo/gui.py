# -*- coding: utf-8 -*-
"""
siheng_servo.gui
================
四横 D-AIS48025A 伺服驱动器 GUI 控制界面。
基于 SihengServo SDK 客户端构建。

启动:
    python -m siheng_servo.gui
    或
    from siheng_servo.gui import launch
    launch()
"""

import sys
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QSpinBox, QTextEdit, QLineEdit,
    QMessageBox, QTabWidget, QSlider
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont

import serial.tools.list_ports

from .client import SihengServo
from .constants import REGISTERS, SERVO_STATUS, ControlMode, SpeedSource, parse_param_address
from .exceptions import SihengError


class _StatusPoller(QThread):
    """后台状态轮询线程"""
    status_signal = pyqtSignal(dict)
    error_signal = pyqtSignal(str)

    def __init__(self, servo: SihengServo, interval_ms: int = 300, parent=None):
        super().__init__(parent)
        self.servo = servo
        self.interval = interval_ms / 1000.0
        self.running = True
        self.enabled = False

    def run(self):
        while self.running:
            if self.enabled and self.servo and self.servo.is_connected:
                try:
                    data = self.servo.get_status()
                    self.status_signal.emit(data)
                except SihengError as e:
                    self.error_signal.emit(str(e))
            time.sleep(self.interval)

    def stop(self):
        self.running = False
        self.wait(2000)


class ServoControlGUI(QMainWindow):
    """四横伺服驱动器 GUI 控制主窗口"""

    def __init__(self):
        super().__init__()
        self.servo: SihengServo = None
        self.poller = _StatusPoller(None)
        self.poller.status_signal.connect(self._update_status)
        self.poller.error_signal.connect(lambda e: self._log(e, "error"))
        self.poller.start()
        self.init_ui()
        self._refresh_ports()

    def init_ui(self):
        self.setWindowTitle("四横 D-AIS48025A 伺服驱动器控制")
        self.setMinimumSize(900, 760)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(8)

        layout.addWidget(self._build_connection_group())

        tabs = QTabWidget()
        tabs.addTab(self._build_control_tab(), "速度控制")
        tabs.addTab(self._build_status_tab(), "实时状态")
        tabs.addTab(self._build_param_tab(), "参数读写")
        layout.addWidget(tabs, 1)
        layout.addWidget(self._build_log_group(), 1)

    # ---------- 连接区 ----------
    def _build_connection_group(self):
        group = QGroupBox("串口连接")
        g = QGridLayout(group)

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "9600", "4800", "19200", "38400", "256000"])
        self.baud_combo.setCurrentText("57600")

        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["N", "E", "O"])

        self.stopbits_combo = QComboBox()
        self.stopbits_combo.addItems(["1", "2"])

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(1)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet(
            "QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
            "QPushButton:hover{background:#45a049;}"
        )
        self.btn_connect.clicked.connect(self._toggle_connection)

        g.addWidget(QLabel("串口:"), 0, 0); g.addWidget(self.port_combo, 0, 1); g.addWidget(btn_refresh, 0, 2)
        g.addWidget(QLabel("波特率:"), 0, 3); g.addWidget(self.baud_combo, 0, 4)
        g.addWidget(QLabel("校验:"), 0, 5); g.addWidget(self.parity_combo, 0, 6)
        g.addWidget(QLabel("停止位:"), 1, 0); g.addWidget(self.stopbits_combo, 1, 1)
        g.addWidget(QLabel("从站:"), 1, 3); g.addWidget(self.slave_spin, 1, 4)
        g.addWidget(self.btn_connect, 0, 7, 2, 1)
        return group

    # ---------- 速度控制页 ----------
    def _build_control_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # 使能
        group_en = QGroupBox("伺服使能")
        ge = QHBoxLayout(group_en)
        self.btn_enable = QPushButton("伺服使能")
        self.btn_enable.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;font-size:13px;border-radius:4px;padding:8px;}")
        self.btn_enable.clicked.connect(self._enable_servo)

        self.btn_disable = QPushButton("断使能")
        self.btn_disable.setStyleSheet("QPushButton{background:#9E9E9E;color:white;font-weight:bold;font-size:13px;border-radius:4px;padding:8px;}")
        self.btn_disable.clicked.connect(self._disable_servo)

        self.btn_fault_reset = QPushButton("故障复位")
        self.btn_fault_reset.setStyleSheet("QPushButton{background:#FF9800;color:white;font-weight:bold;font-size:13px;border-radius:4px;padding:8px;}")
        self.btn_fault_reset.clicked.connect(self._fault_reset)
        ge.addWidget(self.btn_enable); ge.addWidget(self.btn_disable); ge.addWidget(self.btn_fault_reset)
        layout.addWidget(group_en)

        # 速度
        group_spd = QGroupBox("速度控制 (H06_03)")
        gs = QGridLayout(group_spd)
        gs.addWidget(QLabel("目标转速:"), 0, 0)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(-3000, 3000)
        self.speed_spin.setValue(100)
        self.speed_spin.setSuffix(" rpm")
        gs.addWidget(self.speed_spin, 0, 1)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(-3000, 3000)
        self.speed_slider.setValue(100)
        self.speed_slider.valueChanged.connect(self.speed_spin.setValue)
        self.speed_spin.valueChanged.connect(self.speed_slider.setValue)
        gs.addWidget(self.speed_slider, 0, 2, 1, 3)

        self.btn_run = QPushButton("▶ 运行")
        self.btn_run.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;font-size:15px;border-radius:4px;padding:10px;}")
        self.btn_run.clicked.connect(self._run_motor)

        self.btn_stop = QPushButton("■ 停止")
        self.btn_stop.setStyleSheet("QPushButton{background:#FF9800;color:white;font-weight:bold;font-size:15px;border-radius:4px;padding:10px;}")
        self.btn_stop.clicked.connect(self._stop_motor)

        self.btn_emergency = QPushButton("⬛ 急停")
        self.btn_emergency.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;font-size:15px;border-radius:4px;padding:10px;}")
        self.btn_emergency.clicked.connect(self._emergency_stop)
        gs.addWidget(self.btn_run, 1, 0, 1, 2); gs.addWidget(self.btn_stop, 1, 2, 1, 2); gs.addWidget(self.btn_emergency, 1, 4)
        layout.addWidget(group_spd)

        # 点动
        group_jog = QGroupBox("点动控制")
        gj = QHBoxLayout(group_jog)
        gj.addWidget(QLabel("速度:"))
        self.jog_spin = QSpinBox()
        self.jog_spin.setRange(0, 18000)
        self.jog_spin.setValue(100)
        self.jog_spin.setSuffix(" rpm")
        gj.addWidget(self.jog_spin)

        self.btn_jog_cw = QPushButton("◀ 正转点动")
        self.btn_jog_cw.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;padding:8px;}")
        self.btn_jog_cw.pressed.connect(lambda: self._jog_start(1))
        self.btn_jog_cw.released.connect(self._jog_stop)

        self.btn_jog_ccw = QPushButton("反转点动 ▶")
        self.btn_jog_ccw.setStyleSheet("QPushButton{background:#9C27B0;color:white;font-weight:bold;padding:8px;}")
        self.btn_jog_ccw.pressed.connect(lambda: self._jog_start(-1))
        self.btn_jog_ccw.released.connect(self._jog_stop)
        gj.addWidget(self.btn_jog_cw); gj.addWidget(self.btn_jog_ccw)
        layout.addWidget(group_jog)

        # 模式
        group_mode = QGroupBox("控制模式 (需断使能)")
        gm = QGridLayout(group_mode)
        gm.addWidget(QLabel("控制模式:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["0-位置", "1-速度", "2-转矩"])
        self.mode_combo.setCurrentIndex(1)
        gm.addWidget(self.mode_combo, 0, 1)
        btn_mode = QPushButton("设置")
        btn_mode.clicked.connect(self._set_mode)
        gm.addWidget(btn_mode, 0, 2)

        gm.addWidget(QLabel("速度源:"), 1, 0)
        self.source_combo = QComboBox()
        self.source_combo.addItems(["0-内部", "1-模拟量", "2-通讯"])
        gm.addWidget(self.source_combo, 1, 1)
        btn_src = QPushButton("设置")
        btn_src.clicked.connect(self._set_source)
        gm.addWidget(btn_src, 1, 2)
        layout.addWidget(group_mode)

        layout.addStretch()
        return tab

    # ---------- 状态页 ----------
    def _build_status_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("实时监控")
        g = QGridLayout(group)

        self.lbl_status = self._mk_label("未连接", "#f44336")
        self.lbl_speed = self._mk_label("-- rpm")
        self.lbl_position = self._mk_label("--")
        self.lbl_torque = self._mk_label("-- %")
        self.lbl_load = self._mk_label("-- %")
        self.lbl_current = self._mk_label("-- A")
        self.lbl_voltage = self._mk_label("-- V")
        self.lbl_temp = self._mk_label("-- ℃")
        self.lbl_fault = self._mk_label("--")

        g.addWidget(QLabel("伺服状态:"), 0, 0); g.addWidget(self.lbl_status, 0, 1)
        g.addWidget(QLabel("故障码:"), 0, 2); g.addWidget(self.lbl_fault, 0, 3)
        g.addWidget(QLabel("实际转速:"), 1, 0); g.addWidget(self.lbl_speed, 1, 1)
        g.addWidget(QLabel("位置:"), 1, 2); g.addWidget(self.lbl_position, 1, 3)
        g.addWidget(QLabel("转矩:"), 2, 0); g.addWidget(self.lbl_torque, 2, 1)
        g.addWidget(QLabel("负载率:"), 2, 2); g.addWidget(self.lbl_load, 2, 3)
        g.addWidget(QLabel("相电流:"), 3, 0); g.addWidget(self.lbl_current, 3, 1)
        g.addWidget(QLabel("母线电压:"), 3, 2); g.addWidget(self.lbl_voltage, 3, 3)
        g.addWidget(QLabel("温度:"), 4, 0); g.addWidget(self.lbl_temp, 4, 1)

        self.btn_poll = QPushButton("开始监控")
        self.btn_poll.setCheckable(True)
        self.btn_poll.clicked.connect(self._toggle_poll)
        g.addWidget(self.btn_poll, 4, 2, 1, 2)

        btn_once = QPushButton("读取一次")
        btn_once.clicked.connect(self._read_once)
        g.addWidget(btn_once, 4, 1)
        layout.addWidget(group)
        layout.addStretch()
        return tab

    def _mk_label(self, text, color="#333"):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{color};font-weight:bold;font-size:14px;")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumWidth(140)
        return lbl

    # ---------- 参数读写页 ----------
    def _build_param_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("参数读写")
        g = QGridLayout(group)

        g.addWidget(QLabel("参数号:"), 0, 0)
        self.param_name = QLineEdit()
        self.param_name.setPlaceholderText("如 H06_03")
        g.addWidget(self.param_name, 0, 1)
        g.addWidget(QLabel("地址:"), 0, 2)
        self.param_addr = QLineEdit()
        self.param_addr.setReadOnly(True)
        g.addWidget(self.param_addr, 0, 3)

        g.addWidget(QLabel("数值:"), 1, 0)
        self.param_value = QLineEdit()
        g.addWidget(self.param_value, 1, 1)
        btn_calc = QPushButton("解析")
        btn_calc.clicked.connect(self._parse_addr)
        g.addWidget(btn_calc, 1, 2)

        btn_read = QPushButton("读取")
        btn_read.setStyleSheet("QPushButton{background:#2196F3;color:white;font-weight:bold;}")
        btn_read.clicked.connect(self._read_param)
        g.addWidget(btn_read, 2, 0, 1, 2)
        btn_write = QPushButton("写入")
        btn_write.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;}")
        btn_write.clicked.connect(self._write_param)
        g.addWidget(btn_write, 2, 2, 1, 2)
        layout.addWidget(group)

        group2 = QGroupBox("常用参数快捷读取")
        g2 = QGridLayout(group2)
        quick = [
            ("H0C_00 从站", "H0C_00"), ("H0C_02 波特率", "H0C_02"),
            ("H02_00 模式", "H02_00"), ("H06_03 速度设定", "H06_03"),
            ("H32_01 使能", "H32_01"), ("H32_02 伺服状态", "H32_02"),
            ("H0B_00 转速", "H0B_00"), ("H0B_26 电压", "H0B_26"),
        ]
        for i, (label, p) in enumerate(quick):
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, x=p: self._quick_read(x))
            g2.addWidget(btn, i // 3, i % 3)
        layout.addWidget(group2)
        layout.addStretch()
        return tab

    # ---------- 日志 ----------
    def _build_log_group(self):
        group = QGroupBox("操作日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        layout.addWidget(self.log_text)
        btn_clear = QPushButton("清空")
        btn_clear.clicked.connect(self.log_text.clear)
        layout.addWidget(btn_clear, alignment=Qt.AlignRight)
        return group

    # ===================== 事件处理 =====================

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

    def _toggle_connection(self):
        if self.servo and self.servo.is_connected:
            try:
                self.servo.disconnect()
            except SihengError:
                pass
            self.servo = None
            self.poller.servo = None
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet(
                "QPushButton{background:#4CAF50;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
                "QPushButton:hover{background:#45a049;}"
            )
            self._set_controls(False)
            self._log("已断开", "warning")
        else:
            port = self.port_combo.currentData()
            if not port:
                QMessageBox.warning(self, "提示", "请选择串口")
                return
            try:
                self.servo = SihengServo(
                    port=port,
                    baudrate=int(self.baud_combo.currentText()),
                    slave_id=self.slave_spin.value(),
                    parity=self.parity_combo.currentText(),
                    stopbits=int(self.stopbits_combo.currentText()),
                )
                self.servo.connect()
                self.poller.servo = self.servo
                self.btn_connect.setText("断开")
                self.btn_connect.setStyleSheet(
                    "QPushButton{background:#f44336;color:white;font-weight:bold;border-radius:4px;padding:8px;}"
                    "QPushButton:hover{background:#D32F2F;}"
                )
                self._set_controls(True)
                self._log(f"已连接 {port} @ {self.baud_combo.currentText()}", "success")
            except SihengError as e:
                QMessageBox.critical(self, "连接失败", str(e))
                self._log(f"连接失败: {e}", "error")

    def _set_controls(self, enabled):
        for w in [self.btn_enable, self.btn_disable, self.btn_fault_reset,
                  self.btn_run, self.btn_stop, self.btn_emergency,
                  self.btn_jog_cw, self.btn_jog_ccw, self.btn_poll]:
            w.setEnabled(enabled)

    def _enable_servo(self):
        try:
            self.servo.enable()
            self._log("伺服使能", "success")
        except SihengError as e:
            self._log(f"使能失败: {e}", "error")

    def _disable_servo(self):
        try:
            self.servo.disable()
            self._log("断使能", "warning")
        except SihengError as e:
            self._log(f"断使能失败: {e}", "error")

    def _fault_reset(self):
        try:
            self.servo.fault_reset()
            self._log("故障复位", "success")
        except SihengError as e:
            self._log(f"复位失败: {e}", "error")

    def _run_motor(self):
        try:
            self.servo.run(self.speed_spin.value())
            self._log(f"运行 {self.speed_spin.value()} rpm", "success")
        except SihengError as e:
            self._log(f"运行失败: {e}", "error")

    def _stop_motor(self):
        try:
            self.servo.stop()
            self._log("停止", "warning")
        except SihengError as e:
            self._log(f"停止失败: {e}", "error")

    def _emergency_stop(self):
        try:
            self.servo.emergency_stop()
            self._log("急停", "error")
        except SihengError as e:
            self._log(f"急停失败: {e}", "error")

    def _jog_start(self, direction):
        try:
            self.servo.jog(self.jog_spin.value() * direction)
        except SihengError as e:
            self._log(f"点动失败: {e}", "error")

    def _jog_stop(self):
        try:
            self.servo.jog_stop()
        except SihengError:
            pass

    def _set_mode(self):
        try:
            self.servo.set_control_mode(self.mode_combo.currentIndex())
            self._log(f"模式 = {self.mode_combo.currentIndex()}", "success")
        except SihengError as e:
            self._log(f"设置模式失败: {e}", "error")

    def _set_source(self):
        try:
            self.servo.set_speed_source(self.source_combo.currentIndex())
            self._log(f"速度源 = {self.source_combo.currentIndex()}", "success")
        except SihengError as e:
            self._log(f"设置速度源失败: {e}", "error")

    def _toggle_poll(self):
        self.poller.enabled = self.btn_poll.isChecked()
        self.btn_poll.setText("停止监控" if self.btn_poll.isChecked() else "开始监控")

    def _read_once(self):
        if not self.servo or not self.servo.is_connected:
            return
        try:
            self._update_status(self.servo.get_status())
        except SihengError as e:
            self._log(f"读取失败: {e}", "error")

    def _update_status(self, data):
        if not data:
            return
        if "servo_status" in data:
            s = data["servo_status"]
            text = SERVO_STATUS.get(s, f"未知({s})")
            color = "#4CAF50" if s == 3 else ("#f44336" if s in [4, 5] else "#FF9800")
            self.lbl_status.setText(text)
            self.lbl_status.setStyleSheet(f"color:{color};font-weight:bold;font-size:14px;")
        if "speed" in data:
            self.lbl_speed.setText(f"{data['speed']} rpm")
        if "position" in data:
            self.lbl_position.setText(f"{data['position']}")
        if "torque" in data:
            self.lbl_torque.setText(f"{data['torque']/10:.1f} %")
        if "load" in data:
            self.lbl_load.setText(f"{data['load']/10:.1f} %")
        if "current" in data:
            self.lbl_current.setText(f"{data['current']/100:.2f} A")
        if "voltage" in data:
            self.lbl_voltage.setText(f"{data['voltage']:.1f} V")
        if "temperature" in data:
            self.lbl_temp.setText(f"{data['temperature']} ℃")
        if "fault" in data:
            f = data["fault"]
            self.lbl_fault.setText("无故障" if f == 0 else f"故障:{f}")

    def _parse_addr(self):
        name = self.param_name.text().strip().upper()
        if not name:
            return
        try:
            addr = parse_param_address(name)
            self.param_addr.setText(f"0x{addr:04X} ({addr})")
        except ValueError:
            QMessageBox.warning(self, "错误", f"无法解析: {name}")

    def _read_param(self):
        if not self.servo or not self.servo.is_connected:
            QMessageBox.warning(self, "提示", "请先连接")
            return
        name = self.param_name.text().strip().upper()
        if not name:
            return
        self._parse_addr()
        try:
            val = self.servo.read(name)
            self.param_value.setText(str(val))
            self._log(f"读 {name} = {val}", "success")
        except SihengError as e:
            self.param_value.setText(f"错误: {e}")
            self._log(f"读 {name} 失败: {e}", "error")

    def _write_param(self):
        if not self.servo or not self.servo.is_connected:
            QMessageBox.warning(self, "提示", "请先连接")
            return
        name = self.param_name.text().strip().upper()
        text = self.param_value.text().strip()
        if not name or not text:
            return
        try:
            val = int(text, 0)
            self.servo.write(name, val)
            self._log(f"写 {name} = {val} 成功", "success")
        except ValueError:
            QMessageBox.warning(self, "错误", "数值格式错误")
        except SihengError as e:
            self._log(f"写 {name} 失败: {e}", "error")

    def _quick_read(self, param):
        self.param_name.setText(param)
        self._parse_addr()
        self._read_param()

    def _log(self, msg, level="info"):
        colors = {"info": "#333", "success": "#4CAF50", "warning": "#FF9800", "error": "#f44336"}
        t = time.strftime("%H:%M:%S")
        self.log_text.append(
            f'<span style="color:#888;">[{t}]</span> '
            f'<span style="color:{colors.get(level, "#333")};">{msg}</span>'
        )

    def closeEvent(self, event):
        self.poller.stop()
        if self.servo and self.servo.is_connected:
            try:
                self.servo.disconnect()
            except SihengError:
                pass
        event.accept()


def launch():
    """启动 GUI 界面。"""
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ServoControlGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    launch()
