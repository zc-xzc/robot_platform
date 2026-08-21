# -*- coding: utf-8 -*-
"""
四横伺服驱动器 Modbus 自动诊断扫描脚本
==========================================
遍历所有常见波特率、校验位、停止位、从站地址组合，
尝试读取多个可能地址，找出驱动器的正确通信参数。

用法: python siheng_auto_scan.py
"""

import sys
import time
import itertools
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QTextEdit, QProgressBar, QCheckBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.serial import ModbusSerialClient as ModbusSerialClient

import serial.tools.list_ports


class ScanWorker(QThread):
    log_signal = pyqtSignal(str, str)
    progress_signal = pyqtSignal(int, int)
    found_signal = pyqtSignal(dict)
    finished_signal = pyqtSignal()

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port = port
        self.running = True
        # 待测试的通信参数组合
        self.baudrates = [57600, 115200, 9600, 19200, 38400, 4800, 256000]
        self.parities = ["N", "E", "O"]
        self.stopbits_list = [1, 2]
        self.slave_ids = [1, 2, 3, 0]  # 0 是广播，最后试
        # 待测试的寄存器地址（从站地址参数，一定有值）
        # 尝试多种可能的地址映射规则
        self.test_addresses = [
            0x0C00,  # H0C_00 → 0x0C00 (假设的映射)
            0x0000,  # 可能从 0 开始
            0x1000,  # 可能 H0C → 0x1000
            0x0C,    # 可能直接用十进制
            0x2000,  # 其他常见起始
        ]

    def try_read(self, client, slave_id, addr):
        """尝试读取单个寄存器"""
        try:
            rr = client.read_holding_registers(addr, count=1, slave=slave_id, timeout=0.3)
            if not rr.isError():
                return rr.registers[0]
        except Exception:
            pass
        return None

    def run(self):
        total = len(self.baudrates) * len(self.parities) * len(self.stopbits_list)
        done = 0

        for baudrate in self.baudrates:
            if not self.running:
                break
            for parity in self.parities:
                if not self.running:
                    break
                for stopbits in self.stopbits_list:
                    if not self.running:
                        break
                    done += 1
                    self.progress_signal.emit(done, total)
                    self.log_signal.emit(f"尝试 {baudrate}-{parity}-{stopbits}...", "info")

                    try:
                        client = ModbusSerialClient(
                            port=self.port,
                            baudrate=baudrate,
                            parity=parity,
                            stopbits=stopbits,
                            bytesize=8,
                            timeout=0.3,
                        )
                        if not client.connect():
                            continue

                        # 尝试所有从站地址和测试地址
                        for slave_id in self.slave_ids:
                            if not self.running:
                                break
                            for addr in self.test_addresses:
                                val = self.try_read(client, slave_id, addr)
                                if val is not None:
                                    # 找到了！
                                    result = {
                                        "baudrate": baudrate,
                                        "parity": parity,
                                        "stopbits": stopbits,
                                        "slave_id": slave_id,
                                        "addr": addr,
                                        "value": val,
                                    }
                                    self.log_signal.emit(
                                        f"✓ 成功! {baudrate}-{parity}-{stopbits} "
                                        f"从站{slave_id} 地址0x{addr:04X} = {val}",
                                        "success"
                                    )
                                    self.found_signal.emit(result)
                                    client.close()
                                    # 继续扫描看是否还有其他组合
                                    time.sleep(0.1)
                                    break
                            if val is not None:
                                break

                        client.close()
                    except Exception as e:
                        self.log_signal.emit(f"  {baudrate}-{parity}-{stopbits} 异常: {e}", "error")

        self.log_signal.emit("扫描完成", "info")
        self.finished_signal.emit()

    def stop(self):
        self.running = False
        self.wait(3000)


class AutoScanGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.worker = None
        self.found_results = []
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("四横伺服 Modbus 自动诊断扫描")
        self.setMinimumSize(700, 600)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 说明
        info = QLabel(
            "此工具遍历所有波特率/校验位/停止位组合，自动找出驱动器的正确通信参数。\n"
            "扫描完成后，把成功的结果填入主程序即可正常通信。"
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding:10px;background:#FFF9C4;border-radius:4px;")
        layout.addWidget(info)

        # 端口选择
        group_port = QGroupBox("选择串口")
        gp = QHBoxLayout(group_port)
        gp.addWidget(QLabel("串口:"))
        self.port_combo = []
        from PyQt5.QtWidgets import QComboBox
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(300)
        gp.addWidget(self.port_combo)
        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self.refresh_ports)
        gp.addWidget(btn_refresh)
        layout.addWidget(group_port)

        # 扫描按钮
        group_btn = QGroupBox("扫描控制")
        gb = QHBoxLayout(group_btn)
        self.btn_scan = QPushButton("▶ 开始扫描")
        self.btn_scan.setMinimumHeight(40)
        self.btn_scan.setStyleSheet(
            "QPushButton{background:#4CAF50;color:white;font-weight:bold;font-size:14px;border-radius:4px;}"
            "QPushButton:hover{background:#43A047;}"
            "QPushButton:disabled{background:#aaa;}"
        )
        self.btn_scan.clicked.connect(self.toggle_scan)
        gb.addWidget(self.btn_scan)

        self.btn_stop = QPushButton("■ 停止")
        self.btn_stop.setMinimumHeight(40)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            "QPushButton{background:#f44336;color:white;font-weight:bold;font-size:14px;border-radius:4px;}"
            "QPushButton:hover{background:#D32F2F;}"
            "QPushButton:disabled{background:#aaa;}"
        )
        self.btn_stop.clicked.connect(self.stop_scan)
        gb.addWidget(self.btn_stop)
        layout.addWidget(group_btn)

        # 进度条
        self.progress = QProgressBar()
        layout.addWidget(self.progress)

        # 结果显示
        group_result = QGroupBox("扫描结果")
        gr = QVBoxLayout(group_result)
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(100)
        self.result_text.setStyleSheet("font-size:14px;font-weight:bold;")
        gr.addWidget(self.result_text)
        layout.addWidget(group_result)

        # 日志
        group_log = QGroupBox("详细日志")
        gl = QVBoxLayout(group_log)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(__import__("PyQt5.QtGui", fromlist=["QFont"]).QFont("Consolas", 9))
        gl.addWidget(self.log_text)
        layout.addWidget(group_log)

        self.refresh_ports()

    def refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)

    def toggle_scan(self):
        port = self.port_combo.currentData()
        if not port:
            return
        self.found_results = []
        self.result_text.clear()
        self.log_text.clear()
        self.progress.setValue(0)
        self.btn_scan.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.worker = ScanWorker(port)
        self.worker.log_signal.connect(self.append_log)
        self.worker.progress_signal.connect(self.update_progress)
        self.worker.found_signal.connect(self.on_found)
        self.worker.finished_signal.connect(self.on_finished)
        self.worker.start()

    def stop_scan(self):
        if self.worker:
            self.worker.stop()
        self.on_finished()

    def on_found(self, result):
        self.found_results.append(result)
        text = "\n".join([
            f"波特率: {r['baudrate']}, 校验: {r['parity']}, 停止位: {r['stopbits']}, "
            f"从站: {r['slave_id']}, 地址0x{r['addr']:04X} = {r['value']}"
            for r in self.found_results
        ])
        self.result_text.setText(text)

    def update_progress(self, done, total):
        self.progress.setValue(int(done * 100 / total))

    def on_finished(self):
        self.btn_scan.setEnabled(True)
        self.btn_stop.setEnabled(False)
        if not self.found_results:
            self.result_text.setText("未找到任何可通信的组合。\n请检查:\n1. USB-RS485 是否正确连接\n2. 485 A/B 线是否接反\n3. 驱动器是否上电\n4. 驱动器是否已配置 Modbus 模式")

    def append_log(self, msg, level="info"):
        colors = {"info": "#333", "success": "#4CAF50", "warning": "#FF9800", "error": "#f44336"}
        t = time.strftime("%H:%M:%S")
        self.log_text.append(
            f'<span style="color:#888;">[{t}]</span> '
            f'<span style="color:{colors.get(level, "#333")};">{msg}</span>'
        )

    def closeEvent(self, event):
        if self.worker:
            self.worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = AutoScanGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
