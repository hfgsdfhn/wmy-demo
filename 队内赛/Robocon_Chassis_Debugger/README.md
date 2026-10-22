# Robocon Chassis Debug Assistant

Robocon 全向底盘路径调试与实时监控上位机。工程按 UI、算法、数据、机器人模型和 UART 通信分层，STM32 负责轨迹跟踪、底盘逆解与电机控制，上位机只负责轨迹编辑/发送、监控和调试。

## 启动

```powershell
cd Robocon_Chassis_Debugger
python -m pip install -r requirements.txt
python main.py
```

也可以从父目录以模块方式启动：

```powershell
python -m Robocon_Chassis_Debugger.main
```

启动后默认使用模拟状态流，可以先熟悉工作台和键盘手动控制。连接真实 UART 时，在右侧选择端口和波特率，点击“连接 UART”。

## 轨迹兼容

“加载轨迹”支持：

- B-spline 编辑器导出的 `path.h` / `.c`，字段 `{x, y, theta, curvature, s, velocity}`；
- `TrajectoryPoint` JSON（`x, y, yaw, velocity`），支持 `trajectory` 或 `points` 数组；
- CSV（列名 `x,y,yaw,velocity`，也接受旧字段名 `theta`）。

旧编辑器的毫米坐标会自动转换为米，内部统一使用世界坐标米、弧度和 m/s。

## UART 协议

帧格式为 `0xAA 0x55 | ID:uint8 | LEN:uint16 | DATA | CRC16:uint16`，CRC 使用 CRC16-CCITT。协议模块提供手动速度、轨迹列表和 START/STOP/RESET/ESTOP 命令编码，也能解析 STM32 的 `app_real_world_t` 状态帧。

## 安全门禁

轨迹启动前自动执行地图边界和机器人膨胀模型检测。机器人尺寸及安全余量在 `config/robot_config.json` 配置，障碍物采用矩形并按机器人外接圆半径膨胀；失败时会指出路径点序号和距离，START 与轨迹发送都会被拦截。

## 模块

`ui/` 只负责界面和信号；`communication/` 负责 UART 与帧协议；`trajectory/` 负责加载、B-spline 采样和安全检测；`data/` 保存状态与 CSV 记录；`robot/` 负责坐标和底盘几何模型。
                    