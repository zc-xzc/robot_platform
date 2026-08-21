# siheng_servo SDK

上海四横 **D-AIS48025A** 低压伺服驱动器 Python SDK，基于 Modbus RTU 协议，通过 USB-RS485 控制伺服电机。

## 📦 功能特性

- ✅ **Modbus RTU 通信**：标准 RS485 协议，支持 16/32 位数据自动处理
- ✅ **速度控制**：设置目标转速、点动、加减速时间
- ✅ **位置控制**：基于编码器反馈的闭环定位
- ✅ **状态监控**：转速、位置、转矩、负载率、电流、电压、温度、故障码
- ✅ **升降控制**：专用的 `LiftController` 模块，支持上升/下降/定位
- ✅ **GUI 界面**：PyQt5 可视化控制，速度滑块、点动、急停
- ✅ **自动扫描**：自动探测驱动器通信参数
- ✅ **完整异常处理**：6 种自定义异常类型
- ✅ **上下文管理**：支持 `with` 语法自动断开

## 🚀 快速开始

### 安装

```bash
cd siheng_servo_sdk
pip install .
```

安装后可直接命令行启动 GUI：
```bash
siheng-gui
```

### 最简示例

```python
from siheng_servo import SihengServo

# 连接驱动器
servo = SihengServo(port="COM5", baudrate=57600, slave_id=1)
servo.connect()

# 使能并运行
servo.enable()
servo.run(speed=500)   # 正转 500 rpm

import time
time.sleep(3)

# 停止并断开
servo.stop()
servo.disable()
servo.disconnect()
```

### 升降控制示例

```python
from siheng_servo import SihengServo, LiftController

servo = SihengServo(port="COM5", baudrate=57600)
servo.connect()

# 创建升降控制器 (正转=上升, 脉冲当量 1000/mm)
lift = LiftController(
    servo,
    up_direction=1,
    default_speed=200,
    pulses_per_mm=1000
)
lift.enable()

# 速度控制
lift.up()                    # 上升 (默认速度)
lift.stop()
lift.down(speed=300)         # 快速下降
lift.stop()

# 位置控制
lift.move_to_height(50)      # 移动到 50mm 高度
lift.move_relative(10)       # 相对上升 10mm
print(f"当前高度: {lift.get_height()} mm")

# 急停
lift.emergency_stop()
```

### GUI 界面

```bash
# 方式1: 命令行 (安装后可用)
siheng-gui

# 方式2: Python 调用
python -m siheng_servo.gui

# 方式3: 代码启动
from siheng_servo import launch_gui
launch_gui()
```

GUI 提供三个标签页：
- **速度控制**：使能、速度滑块、运行/停止/急停、点动、模式配置
- **实时状态**：转速、位置、转矩、负载率、电流、电压、温度、故障码
- **参数读写**：任意 HXX_YY 参数号的读写

## 📖 API 参考

### SihengServo 类

核心通信客户端。

```python
servo = SihengServo(
    port="COM5",           # 串口
    baudrate=57600,        # 波特率
    slave_id=1,            # 从站地址
    parity="N",            # 校验位 N/E/O
    stopbits=1,            # 停止位 1/2
    bytesize=8,            # 数据位
    timeout=1.0,           # 通信超时(秒)
    word_order=1,          # 32位数据字序 (0=低前, 1=高前)
)
```

#### 连接管理

| 方法 | 说明 |
|------|------|
| `connect()` | 连接驱动器 |
| `disconnect()` | 断开连接 |
| `is_connected` | 属性，是否已连接 |
| `is_enabled` | 属性，是否已使能 |

#### 使能控制

| 方法 | 说明 |
|------|------|
| `enable()` | 伺服使能 (H32_01=1) |
| `disable()` | 伺服断使能 (H32_01=0) |
| `fault_reset()` | 故障复位 (H0D_01=1) |
| `emergency_stop()` | 紧急停止 (H0D_05=1) |

#### 速度控制

| 方法 | 说明 |
|------|------|
| `run(speed)` | 设置转速并运行，负值反转 |
| `stop()` | 停止 (速度=0) |
| `jog(speed)` | 点动，正值正转，负值反转 |
| `jog_stop()` | 停止点动 |
| `set_acceleration(accel_ms, decel_ms)` | 设置加减速时间 |

#### 模式配置

| 方法 | 说明 |
|------|------|
| `set_control_mode(mode)` | 设置控制模式 (0=位置/1=速度/2=转矩) |
| `set_speed_source(source)` | 设置速度源 (0=内部/1=模拟量/2=通讯) |
| `configure_speed_mode()` | 一键配置为内部速度模式 |

#### 状态读取

| 方法 | 返回值 | 单位 |
|------|--------|------|
| `get_status()` | 完整状态字典 | - |
| `get_speed()` | 实际转速 | rpm |
| `get_position()` | 绝对位置 | 脉冲 |
| `get_servo_status()` | 状态文本 | - |
| `get_fault_code()` | 故障码 | 0=无故障 |
| `get_bus_voltage()` | 母线电压 | V |
| `get_temperature()` | 模块温度 | ℃ |

#### 参数读写

```python
# 读任意参数
value = servo.read("H06_03")    # 当前速度设定

# 写任意参数
servo.write("H06_05", 1000)     # 加速时间=1000ms
```

#### 通信参数

| 方法 | 说明 |
|------|------|
| `get_slave_id()` | 读取从站地址 |
| `get_baudrate()` | 读取实际波特率 |
| `save_to_eeprom()` | 保存参数到 EEPROM |

### LiftController 类

升降控制专用模块。

```python
lift = LiftController(
    servo,                  # SihengServo 实例
    up_direction=1,         # 上升方向 (1=正转上升, -1=反转上升)
    default_speed=200,      # 默认速度 rpm
    pulses_per_mm=1000,     # 每毫米脉冲数 (位置控制必需)
    tolerance_mm=0.5,       # 到位容差 mm
    timeout_s=30.0,         # 位置控制超时
)
```

| 方法 | 说明 |
|------|------|
| `enable()` / `disable()` | 使能控制 |
| `up(speed)` | 持续上升 |
| `down(speed)` | 持续下降 |
| `stop()` | 停止 |
| `emergency_stop()` | 急停 |
| `move_to_height(mm, speed)` | 移动到指定高度 |
| `move_relative(mm, speed)` | 相对移动 |
| `get_height()` | 读取当前高度 mm |
| `get_position()` | 读取编码器位置 |
| `set_speed(rpm)` | 设置默认速度 |
| `set_pulses_per_mm(n)` | 配置脉冲当量 |
| `zero_position()` | 位置清零 |
| `is_moving_to_target()` | 是否正在定位 |

### 异常类型

| 异常 | 说明 |
|------|------|
| `SihengError` | 基类 |
| `ConnectionError` | 串口连接失败 |
| `CommunicationError` | Modbus 通信失败 |
| `ParameterError` | 参数错误 |
| `ServoNotEnabledError` | 伺服未使能 |
| `ServoFaultError` | 伺服故障 |

## 📁 项目结构

```
siheng_servo_sdk/
├── siheng_servo/              # SDK 包
│   ├── __init__.py            # 包入口，导出公共 API
│   ├── constants.py           # 寄存器地址表 + 数据类型
│   ├── exceptions.py          # 自定义异常
│   ├── client.py              # SihengServo 通信客户端
│   ├── lift.py                # LiftController 升降控制
│   └── gui.py                 # PyQt5 GUI 界面
├── setup.py                   # 打包配置
├── example.py                 # 使用示例
├── siheng_servo_gui.py        # 独立 GUI 程序
├── siheng_auto_scan.py        # 通信参数自动扫描工具
├── modbus_debug_gui.py        # Modbus 调试探测工具
├── stepper_motor_gui.py       # 通用步进电机 GUI
└── README.md                  # 本文档
```

## 🔧 硬件信息

### 驱动器

| 项目 | 值 |
|------|---|
| 品牌 | 上海四横电机 (Shanghai Siheng Motor) |
| 型号 | D-AIS48025A |
| 类型 | 低压智能伺服驱动器 |
| 电压 | DC 24-60V |
| 额定电流 | 25A |
| 通信接口 | RS485 (Modbus RTU) / CANopen |
| 默认波特率 | 57600 |

### 电机

| 项目 | 值 |
|------|---|
| 型号 | M60AIS117-24-C01330-B2-5-Z |
| 类型 | AC 伺服电机 |
| 额定电压 | 24V |
| 额定电流 | 23A |
| 额定功率 | 0.4kW |
| 额定转矩 | 1.27Nm |
| 额定转速 | 3000RPM |

### 接线

| 驱动器端子 | 连接到 | 说明 |
|-----------|--------|------|
| CN7/CN8 pin5 (485A) | USB-RS485 A | RS485 正端 |
| CN7/CN8 pin4 (485B) | USB-RS485 B | RS485 负端 |
| CN7/CN8 pin6 (GND) | USB-RS485 GND | 信号地 |
| CN1 (DC+/DC-) | 24V 电源 | 驱动器供电 |
| CN1 (U/V/W) | 电机 | 电机动力线 |
| CN2 | 电机编码器 | 编码器反馈 |

## 📋 Modbus 寄存器地址表

地址映射规则：**HXX_YY → 0xXXYY**

| 参数号 | 地址 | 说明 | 数据类型 |
|--------|------|------|---------|
| H02_00 | 0x0200 | 控制模式 (0=位置/1=速度/2=转矩) | UInt16 |
| H06_00 | 0x0600 | 速度指令来源 (0=内部) | UInt16 |
| H06_03 | 0x0603 | **速度设定值** (rpm, 负=反转) | Int16 |
| H06_04 | 0x0604 | 点动速度 | UInt16 |
| H06_05 | 0x0605 | 加速时间 (ms) | UInt16 |
| H06_06 | 0x0606 | 减速时间 (ms) | UInt16 |
| H0B_00 | 0x0B00 | 实际转速 (只读) | Int16 |
| H0B_07 | 0x0B07 | 绝对位置 (只读) | Int32 |
| H0B_24 | 0x0B18 | 相电流 (0.01A) | Int32 |
| H0B_26 | 0x0B1A | 母线电压 (0.1V) | UInt16 |
| H0B_27 | 0x0B1B | 模块温度 (℃) | Int16 |
| H0B_33 | 0x0B21 | 故障码 | UInt16 |
| H0C_00 | 0x0C00 | 从站地址 | UInt16 |
| H0C_02 | 0x0C02 | 波特率编码 | UInt16 |
| H0D_01 | 0x0D01 | 故障复位 | UInt16 |
| H0D_05 | 0x0D05 | 紧急停机 | UInt16 |
| H32_01 | 0x3201 | **伺服使能** (0=断/1=通) | UInt16 |
| H32_02 | 0x3202 | 伺服状态 (只读) | UInt16 |

完整地址表见 [siheng_servo/constants.py](siheng_servo/constants.py)。

## 🛠️ 使用流程

### 首次使用

1. **接线**：按接线表连接 USB-RS485、电源、电机
2. **上电**：驱动器供电 24V
3. **扫描** (可选)：如果不确定波特率，运行 `siheng_auto_scan.py`
4. **连接**：启动 GUI 或运行示例代码
5. **配置模式**：断使能 → 设置 H02_00=1(速度模式) → 设置 H06_00=0(内部速度源)
6. **使能运行**：使能 → 设置速度 → 运行

### 日常使用

```python
servo = SihengServo(port="COM5")   # 默认 57600
servo.connect()
servo.enable()
servo.run(500)                     # 500 rpm
# ... 使用 ...
servo.stop()
servo.disable()
servo.disconnect()
```

## ⚠️ 注意事项

1. **修改模式前必须断使能**：H02_00 的生效方式是"使能断开"
2. **首次测试速度从小值开始**：建议 50-100 rpm，确认方向正确后再加大
3. **急停准备**：测试时确保急停按钮/断电开关在手边
4. **485 接线**：A/B 不能接反，否则无法通信
5. **终端电阻**：长距离通信建议在总线末端加 120Ω 终端电阻
6. **脉冲当量**：位置控制需要实测 `pulses_per_mm`，计算方法：编码器分辨率×减速比/丝杠导程

## 🔍 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 连接失败 | 串口被占用 | 关闭其他占用 COM 口的程序 |
| 无响应 | A/B 接反 | 对调 RS485 A/B 线 |
| 无响应 | 波特率不对 | 用 `siheng_auto_scan.py` 扫描 |
| 无响应 | 从站地址不对 | 尝试 1-5 的从站地址 |
| 电机不转 | 未使能 | 先调用 `enable()` |
| 电机不转 | 速度模式未配置 | 调用 `configure_speed_mode()` |
| 方向反了 | up_direction 配置 | 修改为 -1 |
| 通信超时 | 线缆太长/干扰 | 加终端电阻，使用屏蔽线 |

## 📝 依赖

- Python >= 3.7
- pymodbus >= 3.0
- pyserial >= 3.4
- PyQt5 >= 5.15 (GUI 功能)

## 📄 许可证

MIT License
