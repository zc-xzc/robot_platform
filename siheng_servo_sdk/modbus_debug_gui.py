# -*- coding: utf-8 -*-
"""
Modbus RTU 调试探测版 GUI
==================================
用于通过 USB-RS485 连接上海四横 D-AIS48025A-C 等伺服驱动器，
手动探测 Modbus 寄存器地址、测试读写、扫描从站、控制速度。

功能：
- 串口连接管理
- 单个/多个保持寄存器读写 (FC03/FC06/FC16)
- 线圈读写 (FC01/FC05)
- 寄存器地址范围扫描
- 从站地址扫描
- 可配置地址的速度测试控制
- 常用命令模板
"""

import sys
import time
import struct
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QSpinBox, QLineEdit,
    QTextEdit, QMessageBox, QTabWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QSplitter, QCheckBox, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.serial import ModbusSerialClient as ModbusSerialClient


class ModbusWorker(QThread):
    """后台 Modbus 通信线程"""
    log_signal = pyqtSignal(str, str)
    result_signal = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.client = None
        self.slave_id = 1
        self.running = True
        self.poll_interval = 0.3
        self.poll_registers = []  # [(name, addr, count), ...]

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

    def read_holding(self, addr, count, slave=None):
        s = slave if slave is not None else self.slave_id
        if not self.client:
            return None, "未连接"
        try:
            rr = self.client.read_holding_registers(addr, count=count, slave=s)
            if rr.isError():
                return None, f"错误: {rr}"
            return rr.registers, None
        except Exception as e:
            return None, f"异常: {e}"

    def write_holding(self, addr, value_or_values, slave=None):
        s = slave if slave is not None else self.slave_id
        if not self.client:
            return False, "未连接"
        try:
            if isinstance(value_or_values, list):
                rr = self.client.write_registers(addr, value_or_values, slave=s)
            else:
                rr = self.client.write_register(addr, value_or_values, slave=s)
            if rr.isError():
                return False, f"错误: {rr}"
            return True, None
        except Exception as e:
            return False, f"异常: {e}"

    def read_coil(self, addr, count, slave=None):
        s = slave if slave is not None else self.slave_id
        if not self.client:
            return None, "未连接"
        try:
            rr = self.client.read_coils(addr, count=count, slave=s)
            if rr.isError():
                return None, f"错误: {rr}"
            return rr.bits[:count], None
        except Exception as e:
            return None, f"异常: {e}"

    def write_coil(self, addr, value, slave=None):
        s = slave if slave is not None else self.slave_id
        if not self.client:
            return False, "未连接"
        try:
            rr = self.client.write_coil(addr, value, slave=s)
            if rr.isError():
                return False, f"错误: {rr}"
            return True, None
        except Exception as e:
            return False, f"异常: {e}"

    def run(self):
        while self.running:
            if self.client and self.poll_registers:
                results = {}
                for name, addr, count in self.poll_registers:
                    vals, err = self.read_holding(addr, count)
                    results[name] = {"addr": addr, "values": vals, "error": err}
                if results:
                    self.result_signal.emit({"type": "poll", "data": results})
            time.sleep(self.poll_interval)

    def stop(self):
        self.running = False
        self.wait(2000)


class ModbusDebugGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = ModbusWorker()
        self.worker.log_signal.connect(self.append_log)
        self.worker.result_signal.connect(self.on_worker_result)
        self.worker.start()
        self.connected = False
        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        self.setWindowTitle("Modbus RTU 调试探测工具")
        self.setMinimumSize(1000, 720)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)

        main_layout.addWidget(self._create_connection_group())

        tabs = QTabWidget()
        tabs.addTab(self._create_readwrite_tab(), "手动读写")
        tabs.addTab(self._create_scan_tab(), "扫描探测")
        tabs.addTab(self._create_speed_tab(), "速度控制")
        tabs.addTab(self._create_templates_tab(), "命令模板")
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
        self.baud_combo.addItems(["4800", "9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("9600")

        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["N", "E", "O"])

        self.stopbits_combo = QComboBox()
        self.stopbits_combo.addItems(["1", "1.5", "2"])

        self.databits_combo = QComboBox()
        self.databits_combo.addItems(["7", "8"])
        self.databits_combo.setCurrentText("8")

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(1)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;}")
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

    # ---------- 手动读写页 ----------
    def _create_readwrite_tab(self):
        tab = QWidget()
        layout = QGridLayout(tab)

        # 读寄存器
        group_r = QGroupBox("读寄存器")
        gr = QGridLayout(group_r)
        self.rw_type_r = QComboBox()
        self.rw_type_r.addItems(["保持寄存器 (FC03)", "线圈 (FC01)"])
        gr.addWidget(QLabel("类型:"), 0, 0)
        gr.addWidget(self.rw_type_r, 0, 1)
        gr.addWidget(QLabel("地址:"), 1, 0)
        self.r_addr = QSpinBox()
        self.r_addr.setRange(0, 65535)
        self.r_addr.setDisplayIntegerBase(16)
        self.r_addr.setPrefix("0x")
        gr.addWidget(self.r_addr, 1, 1)
        gr.addWidget(QLabel("数量:"), 1, 2)
        self.r_count = QSpinBox()
        self.r_count.setRange(1, 125)
        self.r_count.setValue(1)
        gr.addWidget(self.r_count, 1, 3)
        btn_read = QPushButton("读取")
        btn_read.clicked.connect(self.do_read)
        gr.addWidget(btn_read, 2, 0, 1, 4)
        self.r_result = QLineEdit()
        self.r_result.setReadOnly(True)
        gr.addWidget(self.r_result, 3, 0, 1, 4)
        layout.addWidget(group_r, 0, 0)

        # 写寄存器
        group_w = QGroupBox("写寄存器")
        gw = QGridLayout(group_w)
        self.rw_type_w = QComboBox()
        self.rw_type_w.addItems(["保持寄存器 (FC06)", "线圈 (FC05)", "多寄存器 (FC16)"])
        gw.addWidget(QLabel("类型:"), 0, 0)
        gw.addWidget(self.rw_type_w, 0, 1)
        gw.addWidget(QLabel("地址:"), 1, 0)
        self.w_addr = QSpinBox()
        self.w_addr.setRange(0, 65535)
        self.w_addr.setDisplayIntegerBase(16)
        self.w_addr.setPrefix("0x")
        gw.addWidget(self.w_addr, 1, 1)
        gw.addWidget(QLabel("数值:"), 2, 0)
        self.w_value = QLineEdit()
        self.w_value.setPlaceholderText("单个值: 100 | 多寄存器: 100,200,300")
        gw.addWidget(self.w_value, 2, 1, 1, 3)
        btn_write = QPushButton("写入")
        btn_write.clicked.connect(self.do_write)
        gw.addWidget(btn_write, 3, 0, 1, 4)
        layout.addWidget(group_w, 0, 1)

        # 轮询监控
        group_p = QGroupBox("轮询监控 (每 300ms)")
        gp = QVBoxLayout(group_p)
        gp.addWidget(QLabel("格式: 名称=地址:数量, 多个用分号分隔"))
        self.poll_input = QLineEdit()
        self.poll_input.setPlaceholderText("例如: 状态=0x0000:1, 速度=0x0001:1")
        gp.addWidget(self.poll_input)
        btn_poll = QPushButton("开始轮询")
        btn_poll.setCheckable(True)
        btn_poll.toggled.connect(self.toggle_poll)
        gp.addWidget(btn_poll)
        self.poll_table = QTableWidget(0, 3)
        self.poll_table.setHorizontalHeaderLabels(["名称", "地址", "值"])
        self.poll_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        gp.addWidget(self.poll_table)
        layout.addWidget(group_p, 1, 0, 1, 2)

        return tab

    # ---------- 扫描页 ----------
    def _create_scan_tab(self):
        tab = QWidget()
        layout = QGridLayout(tab)

        group_slave = QGroupBox("扫描从站地址")
        gs = QGridLayout(group_slave)
        gs.addWidget(QLabel("范围:"), 0, 0)
        self.slave_start = QSpinBox()
        self.slave_start.setRange(1, 247)
        self.slave_start.setValue(1)
        gs.addWidget(self.slave_start, 0, 1)
        gs.addWidget(QLabel("-"), 0, 2)
        self.slave_end = QSpinBox()
        self.slave_end.setRange(1, 247)
        self.slave_end.setValue(10)
        gs.addWidget(self.slave_end, 0, 3)
        btn_scan_slave = QPushButton("扫描从站")
        btn_scan_slave.clicked.connect(self.scan_slave)
        gs.addWidget(btn_scan_slave, 1, 0, 1, 4)
        self.slave_result = QTextEdit()
        self.slave_result.setReadOnly(True)
        self.slave_result.setMaximumHeight(100)
        gs.addWidget(self.slave_result, 2, 0, 1, 4)
        layout.addWidget(group_slave, 0, 0)

        group_reg = QGroupBox("扫描寄存器地址")
        gr = QGridLayout(group_reg)
        gr.addWidget(QLabel("起始:"), 0, 0)
        self.reg_start = QSpinBox()
        self.reg_start.setRange(0, 65535)
        self.reg_start.setDisplayIntegerBase(16)
        self.reg_start.setPrefix("0x")
        gr.addWidget(self.reg_start, 0, 1)
        gr.addWidget(QLabel("结束:"), 0, 2)
        self.reg_end = QSpinBox()
        self.reg_end.setRange(0, 65535)
        self.reg_end.setValue(100)
        self.reg_end.setDisplayIntegerBase(16)
        self.reg_end.setPrefix("0x")
        gr.addWidget(self.reg_end, 0, 3)
        self.scan_coil_rb = QRadioButton("线圈")
        self.scan_holding_rb = QRadioButton("保持寄存器")
        self.scan_holding_rb.setChecked(True)
        gr.addWidget(self.scan_holding_rb, 1, 0)
        gr.addWidget(self.scan_coil_rb, 1, 1)
        btn_scan_reg = QPushButton("扫描寄存器")
        btn_scan_reg.clicked.connect(self.scan_registers)
        gr.addWidget(btn_scan_reg, 2, 0, 1, 4)
        self.reg_table = QTableWidget(0, 3)
        self.reg_table.setHorizontalHeaderLabels(["地址", "十进制", "十六进制"])
        self.reg_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        gr.addWidget(self.reg_table, 3, 0, 1, 4)
        layout.addWidget(group_reg, 0, 1)

        return tab

    # ---------- 速度控制页 ----------
    def _create_speed_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("可配置地址速度控制")
        g = QGridLayout(group)

        g.addWidget(QLabel("控制字地址:"), 0, 0)
        self.sc_ctrl_addr = QSpinBox()
        self.sc_ctrl_addr.setRange(0, 65535)
        self.sc_ctrl_addr.setValue(0x0000)
        self.sc_ctrl_addr.setDisplayIntegerBase(16)
        self.sc_ctrl_addr.setPrefix("0x")
        g.addWidget(self.sc_ctrl_addr, 0, 1)

        g.addWidget(QLabel("速度地址:"), 0, 2)
        self.sc_speed_addr = QSpinBox()
        self.sc_speed_addr.setRange(0, 65535)
        self.sc_speed_addr.setValue(0x0001)
        self.sc_speed_addr.setDisplayIntegerBase(16)
        self.sc_speed_addr.setPrefix("0x")
        g.addWidget(self.sc_speed_addr, 0, 3)

        g.addWidget(QLabel("方向地址:"), 1, 0)
        self.sc_dir_addr = QSpinBox()
        self.sc_dir_addr.setRange(0, 65535)
        self.sc_dir_addr.setValue(0x0002)
        self.sc_dir_addr.setDisplayIntegerBase(16)
        self.sc_dir_addr.setPrefix("0x")
        g.addWidget(self.sc_dir_addr, 1, 1)

        g.addWidget(QLabel("使能值:"), 1, 2)
        self.sc_enable_val = QSpinBox()
        self.sc_enable_val.setRange(0, 65535)
        self.sc_enable_val.setValue(0x0001)
        self.sc_enable_val.setDisplayIntegerBase(16)
        self.sc_enable_val.setPrefix("0x")
        g.addWidget(self.sc_enable_val, 1, 3)

        g.addWidget(QLabel("速度值:"), 2, 0)
        self.sc_speed_val = QSpinBox()
        self.sc_speed_val.setRange(-32768, 32767)
        self.sc_speed_val.setValue(100)
        g.addWidget(self.sc_speed_val, 2, 1)

        g.addWidget(QLabel("方向:"), 2, 2)
        self.sc_dir_val = QSpinBox()
        self.sc_dir_val.setRange(0, 1)
        self.sc_dir_val.setValue(0)
        g.addWidget(self.sc_dir_val, 2, 3)

        btn_enable = QPushButton("使能")
        btn_enable.clicked.connect(lambda: self.sc_write_ctrl(self.sc_enable_val.value()))
        btn_run = QPushButton("启动运行")
        btn_run.clicked.connect(self.sc_run)
        btn_stop = QPushButton("停止")
        btn_stop.clicked.connect(self.sc_stop)

        g.addWidget(btn_enable, 3, 0)
        g.addWidget(btn_run, 3, 1)
        g.addWidget(btn_stop, 3, 2)

        layout.addWidget(group)
        layout.addStretch()
        return tab

    # ---------- 命令模板页 ----------
    def _create_templates_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        group = QGroupBox("常用测试模板")
        g = QVBoxLayout(group)

        templates = [
            ("读取设备地址 (0x0000)", lambda: self.apply_template(0x0000, 1, "read_holding")),
            ("读取当前速度 (假设 0x0001)", lambda: self.apply_template(0x0001, 1, "read_holding")),
            ("读取当前位置 (假设 0x0002)", lambda: self.apply_template(0x0002, 1, "read_holding")),
            ("读取状态字 (假设 0x0000)", lambda: self.apply_template(0x0000, 1, "read_holding")),
            ("写速度 100 (假设 0x0001)", lambda: self.apply_template_write(0x0001, 100)),
            ("写控制字 0x0001 使能", lambda: self.apply_template_write(0x0000, 0x0001)),
            ("写控制字 0x0002 启动", lambda: self.apply_template_write(0x0000, 0x0002)),
            ("写控制字 0x0004 停止", lambda: self.apply_template_write(0x0000, 0x0004)),
        ]

        for text, cb in templates:
            btn = QPushButton(text)
            btn.clicked.connect(cb)
            g.addWidget(btn)

        layout.addWidget(group)
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
                stopbits=float(self.stopbits_combo.currentText()),
                bytesize=int(self.databits_combo.currentText()),
                slave_id=self.slave_spin.value(),
            )
            if ok:
                self.connected = True
                self.btn_connect.setText("断开")
                self.btn_connect.setStyleSheet("QPushButton{background:#f44336;color:white;font-weight:bold;}")
        else:
            self.worker.disconnect_port()
            self.connected = False
            self.btn_connect.setText("连接")
            self.btn_connect.setStyleSheet("QPushButton{background:#4CAF50;color:white;font-weight:bold;}")

    def do_read(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        addr = self.r_addr.value()
        count = self.r_count.value()
        typ = self.rw_type_r.currentText()
        if "保持" in typ:
            vals, err = self.worker.read_holding(addr, count)
        else:
            vals, err = self.worker.read_coil(addr, count)
        if err:
            self.r_result.setText(f"失败: {err}")
            self.append_log(f"读 0x{addr:04X} 失败: {err}", "error")
        else:
            hex_vals = [f"0x{v:04X}" for v in vals]
            self.r_result.setText(f"十进制: {vals} | 十六进制: {hex_vals}")
            self.append_log(f"读 0x{addr:04X} x{count}: {vals} ({hex_vals})", "success")

    def do_write(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        addr = self.w_addr.value()
        text = self.w_value.text().strip()
        typ = self.rw_type_w.currentText()
        try:
            if "线圈" in typ:
                val = 1 if text in ["1", "true", "True", "ON", "on"] else 0
                ok, err = self.worker.write_coil(addr, val)
            elif "多寄存器" in typ:
                vals = [int(x.strip()) for x in text.split(",")]
                ok, err = self.worker.write_holding(addr, vals)
            else:
                val = int(text, 0)
                ok, err = self.worker.write_holding(addr, val)
            if ok:
                self.append_log(f"写 0x{addr:04X} = {text} 成功", "success")
            else:
                self.append_log(f"写 0x{addr:04X} 失败: {err}", "error")
        except Exception as e:
            self.append_log(f"写入数值解析错误: {e}", "error")

    def toggle_poll(self, checked):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            self.sender().setChecked(False)
            return
        if checked:
            text = self.poll_input.text().strip()
            if not text:
                self.sender().setChecked(False)
                return
            regs = []
            rows = []
            for item in text.split(";"):
                item = item.strip()
                if "=" in item:
                    name, rest = item.split("=", 1)
                else:
                    name = item
                    rest = item
                name = name.strip()
                if ":" in rest:
                    addr_str, count_str = rest.split(":", 1)
                else:
                    addr_str = rest
                    count_str = "1"
                addr = int(addr_str.strip(), 0)
                count = int(count_str.strip())
                regs.append((name, addr, count))
                rows.append([name, f"0x{addr:04X}", ""])
            self.worker.poll_registers = regs
            self.poll_table.setRowCount(len(rows))
            for i, row in enumerate(rows):
                for j, v in enumerate(row):
                    self.poll_table.setItem(i, j, QTableWidgetItem(v))
            self.append_log(f"开始轮询: {regs}", "info")
            self.sender().setText("停止轮询")
        else:
            self.worker.poll_registers = []
            self.append_log("停止轮询", "info")
            self.sender().setText("开始轮询")

    def on_worker_result(self, payload):
        if payload.get("type") == "poll":
            for name, info in payload["data"].items():
                for row in range(self.poll_table.rowCount()):
                    if self.poll_table.item(row, 0).text() == name:
                        if info["error"]:
                            self.poll_table.item(row, 2).setText(f"错误: {info['error']}")
                        else:
                            vals = info["values"]
                            hexs = [f"0x{v:04X}" for v in vals]
                            self.poll_table.item(row, 2).setText(f"{vals} / {hexs}")
                        break

    # ---------- 扫描 ----------
    def scan_slave(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        self.slave_result.clear()
        self.append_log(f"开始扫描从站 {self.slave_start.value()}~{self.slave_end.value()}", "info")
        found = []
        for sid in range(self.slave_start.value(), self.slave_end.value() + 1):
            vals, err = self.worker.read_holding(0, 1, slave=sid)
            if not err and vals is not None:
                found.append(sid)
                self.append_log(f"发现从站: {sid}", "success")
        self.slave_result.setText(f"发现从站: {found}" if found else "未发现任何从站")

    def scan_registers(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        start = self.reg_start.value()
        end = self.reg_end.value()
        if end < start:
            end, start = start, end
        self.reg_table.setRowCount(0)
        self.append_log(f"开始扫描寄存器 0x{start:04X}~0x{end:04X}", "info")
        use_coil = self.scan_coil_rb.isChecked()
        for addr in range(start, end + 1):
            if use_coil:
                vals, err = self.worker.read_coil(addr, 1)
            else:
                vals, err = self.worker.read_holding(addr, 1)
            if not err and vals is not None:
                row = self.reg_table.rowCount()
                self.reg_table.insertRow(row)
                self.reg_table.setItem(row, 0, QTableWidgetItem(f"0x{addr:04X}"))
                self.reg_table.setItem(row, 1, QTableWidgetItem(str(vals[0])))
                self.reg_table.setItem(row, 2, QTableWidgetItem(f"0x{vals[0]:04X}"))
                self.append_log(f"  0x{addr:04X} = {vals[0]} (0x{vals[0]:04X})", "success")
        self.append_log("扫描完成", "info")

    # ---------- 速度控制 ----------
    def sc_write_ctrl(self, value):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        addr = self.sc_ctrl_addr.value()
        ok, err = self.worker.write_holding(addr, value)
        if ok:
            self.append_log(f"写控制字 0x{addr:04X} = 0x{value:04X} 成功", "success")
        else:
            self.append_log(f"写控制字失败: {err}", "error")

    def sc_run(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        # 先写方向，再写速度，最后写启动（地址可配置）
        self.worker.write_holding(self.sc_dir_addr.value(), self.sc_dir_val.value())
        self.worker.write_holding(self.sc_speed_addr.value(), self.sc_speed_val.value())
        self.sc_write_ctrl(self.sc_enable_val.value() | 0x0002)
        self.append_log("启动运行指令已发送", "success")

    def sc_stop(self):
        if not self.connected:
            QMessageBox.warning(self, "提示", "请先连接串口")
            return
        self.sc_write_ctrl(0x0004)

    # ---------- 模板 ----------
    def apply_template(self, addr, count, typ):
        self.r_addr.setValue(addr)
        self.r_count.setValue(count)
        if typ == "read_holding":
            self.rw_type_r.setCurrentIndex(0)
        elif typ == "read_coil":
            self.rw_type_r.setCurrentIndex(1)
        self.do_read()

    def apply_template_write(self, addr, value):
        self.w_addr.setValue(addr)
        self.w_value.setText(str(value))
        self.rw_type_w.setCurrentIndex(0)
        self.do_write()

    # ---------- 日志 ----------
    def append_log(self, msg, level="info"):
        colors = {"info": "#333", "success": "#4CAF50", "warning": "#FF9800", "error": "#f44336"}
        t = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#888;">[{t}]</span> '
                             f'<span style="color:{colors.get(level,"#333")};">{msg}</span>')

    def closeEvent(self, event):
        self.worker.stop()
        if self.connected:
            self.worker.disconnect_port()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ModbusDebugGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
