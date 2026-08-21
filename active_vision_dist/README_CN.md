# 主动视觉系统 — Active Vision System

PICO 4 头部追踪 → 实时双轴 STS3032 舵机云台 → D415 摄像头跟随 → 画面回传 PICO 头显。

**"头动视角动"——沉浸式远程视觉。**

---

## 硬件配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 头显 | PICO 4 | WiFi 连接 PC |
| 舵机 (×2) | STS3032 12V (ST-3032-C062) | ID1=水平旋转, ID2=垂直俯仰 |
| 串口适配器 | URT-1 | USB 转 TTL |
| 摄像头 | Intel RealSense D415 | USB 3.0，装在云台上 |
| PC | Windows/Linux | WiFi + USB 接口 |

PICO 4 运行 [PicoBridge](https://github.com/OpenRobotTech/PicoBridge) APK，通过 WiFi 将头部追踪数据（四元数 + 身体关节）实时传输到 PC。

---

## 安装步骤

### 1. Python 环境

```bash
conda create -n active_vision python=3.10 -y
conda activate active_vision
```

### 2. 依赖包

```bash
pip install -r requirements.txt
```

### 3. PicoBridge

```bash
pip install pico_bridge-0.2.1-py3-none-any.whl
```

### 4. PICO 头显

在 PICO 4 上安装 `PicoBridge_v0.2.1_20260522_release.apk`。确保 PICO 和 PC 在同一 WiFi 网络下。在头显上打开 PicoBridge。

---

## 快速开始

### Windows

```bash
cd windows
python run.py
```

### Linux

```bash
cd linux
python run.py
```

---

## 首次配置

### 第1步：校准舵机极限

```bash
python run.py --calibrate
```

舵机会自动扫描物理极限范围，然后锁定在 2048（中位）。**此时手动旋转云台支架，使摄像头对准正前方，拧紧螺丝。** 按 Enter 完成。

会自动生成 `gimbal_config.json`。每次重新拆装硬件后需重做一次。

### 第2步：调优舵机内部参数（消除抖动）

```bash
python run.py --tune-servo
```

将优化后的 PID 参数（D=10, 死区=8）写入舵机 EEPROM。一次性操作，断电不丢失。

### 第3步：验证方向

```bash
python run.py
```

戴上头显，正视前方，等待出现 `[CALIBRATED]`。转头测试：

| 头部动作 | 摄像头应 |
|---------|---------|
| 抬头 | 向上 |
| 低头 | 向下 |
| 头右转 | 向右 |

如果某方向反了，加对应参数：

```bash
python run.py --no-inv-yaw            # 水平反了
python run.py --no-inv-pitch          # 垂直反了
python run.py --no-inv-yaw --no-inv-pitch  # 都反了
```

---

## 端口配置

| 系统 | 默认端口 | 覆盖命令 |
|------|---------|---------|
| Windows | COM6 | `--port COM5` |
| Linux | /dev/ttyUSB0 | `--port /dev/ttyUSB1` |

---

## 参数调优

默认参数（通过自动阶跃响应测试优化，适配 STS3032 12V）：

| 参数 | 默认值 | 范围 | 作用 |
|------|--------|------|------|
| `--acc` | 80 | 0-254 | 加速度（越高响应越快，254 会限速） |
| `--kp` | 2.0 | 0.1-10 | 追踪力度（越高跟得越紧） |
| `--kd` | 0.5 | 0-2 | 阻尼（越高刹车越稳） |

### 运行时调参

在项目根目录**另开一个终端**：

```bash
echo acc 60 > tune_cmd.txt
echo kp 2.5 > tune_cmd.txt
echo kd 0.7 > tune_cmd.txt
echo c    > tune_cmd.txt    # 重新归中
echo q    > tune_cmd.txt    # 退出
```

### 预设方案

```bash
# 激进（最快）
python run.py --acc 0 --kp 2.5 --kd 0.7

# 平衡（推荐）
python run.py --acc 80 --kp 2.0 --kd 0.5

# 保守（最稳）
python run.py --acc 100 --kp 1.5 --kd 0.3
```

---

## 运行模式

| 命令 | 说明 |
|------|------|
| `python run.py` | 完整追踪（默认） |
| `python run.py --calibrate` | 舵机极限校准 |
| `python run.py --tune-servo` | 舵机内部 PID 调优 |
| `python run.py --test-head` | 仅测试 PICO 连接 |
| `python run.py --test-camera` | 仅测试 D415 推流 |
| `python run.py --log data.csv` | 追踪 + 保存 CSV 数据 |
| `python run.py --no-camera` | 不启用摄像头 |
| `python run.py --no-body` | 不启用身体相对模式 |

---

## 自动寻优（可选）

```bash
python autotune.py
```

自动测试 100+ 种参数组合的舵机阶跃响应，输出最优命令行。约需 5 分钟。

---

## 数据采集

```bash
python run.py --log experiment_001.csv
```

14 列 CSV：时间戳、帧号、原始 yaw/pitch/roll、目标位置、PD 平滑位置、舵机步数、速度、KP、KD。

---

## 算法链路

```
PICO 四元数 (50Hz)
  → TWIST2 身体相对: q_rel = conjugate(spine) × head
  → YXZ 欧拉角分解: yaw, pitch, roll
  → 归一化: yaw/90, pitch/60 → [-1, +1]
  → 跳变 (|误差| > 7.2°): 瞬时到位，0延迟
  → PD (|误差| ≤ 7.2°): 2.0×误差 + 0.5×d(误差) + 前馈
  → 舵机映射: 位置 = 2048 + 归一化值 × 系数
    (1024步 = 90° 水平, 682步 = 60° 垂直)
  → WritePosEx(ID, 位置, 速度, 加速度)
  → STS3032 转动 → D415 跟随 → 视频回传 PICO
```

---

## 文件结构

```
active-vision-dist/
├── README.md                  ← 英文说明
├── README_CN.md               ← 中文说明（本文件）
├── TECHNICAL_REFERENCE.md     ← 英文技术详解
├── TECHNICAL_REFERENCE_CN.md  ← 中文技术详解
├── requirements.txt           ← Python 依赖
├── autotune.py                ← 自动寻优工具
├── scservo_sdk/               ← 飞特舵机 SDK（勿修改）
├── windows/
│   └── run.py                 ← Windows 版 (默认 COM6)
└── linux/
    └── run.py                 ← Linux 版 (默认 /dev/ttyUSB0)
```

运行时自动生成的文件：
- `gimbal_config.json` — `--calibrate` 创建
- `tune_cmd.txt` — 实时调参命令

---

## 常见问题

| 现象 | 解决 |
|------|------|
| 舵机无响应 | 检查 12V 电源（需 3A+） |
| PICO 连不上 | 同 WiFi，PicoBridge APK 打开 |
| 抖动/振荡 | `python run.py --acc 100 --kd 0.8` |
| 速度太慢 | `python run.py --acc 50 --kp 2.5` |
| 方向反了 | 加 `--no-inv-yaw` 或 `--no-inv-pitch` |
| 中心偏了 | 正视前方，`echo c > tune_cmd.txt` |

更多技术细节见 `TECHNICAL_REFERENCE_CN.md`。
