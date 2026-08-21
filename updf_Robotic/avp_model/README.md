# AVP URDF + MuJoCo 共用网格版

本目录同时包含 URDF 与 MuJoCo 模型，两者共用根目录下唯一一套 `meshes/` 网格文件。

主要入口：

- `urdf/avp_model.SLDASM.urdf`：ROS / URDF 模型
- `avp_model.xml`：MuJoCo 模型
- `meshes/`：两种模型共用的 STL 网格
- `view_model.py`：MuJoCo 交互查看程序

本版将第二关节的旋转方向修正为 STL 圆柱端面测得的真实中心线：

```text
-0.0360199767720671  0.998802471712146  0.033108666162308
```

该修正确保舵机输出端、舵盘、中央支架和右侧轴在旋转过程中保持同轴，模型中不包含红色调试线。

运行 MuJoCo 静态查看：

```bash
cd /Users/m/Desktop/IAAA/avp_model_unified
mjpython view_model.py
```

运行第二关节往复旋转：

```bash
cd /Users/m/Desktop/IAAA/avp_model_unified
mjpython view_model.py --sweep
```

macOS 上 MuJoCo 的窗口查看器需要 `mjpython`，不能直接用普通 `python3` 启动。
