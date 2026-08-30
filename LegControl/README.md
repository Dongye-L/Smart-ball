# LegControl — 双传感器（大腿+小腿）→ 虚拟串口 → Blender 腿部实时驱动

## 文件说明

| 文件 | 作用 | 运行位置 |
|---|---|---|
| `ble_to_blender.py` | 读取 BLE 大腿(UpperLeg)+小腿(LowerLeg)传感器，转成身体坐标系四元数，写入虚拟串口（格式与 Arduino 固件一致） | Windows 命令行 |
| `legcontrol_dual.py` | 从虚拟串口读 `/UpperLeg` `/LowerLeg`，按 `armcontrol.py` 的父子分解驱动右腿骨骼 | Blender 脚本编辑器 |
| `uartcomm.py` | 串口 JSON 行解析（Blender 端依赖） | Blender 侧 `scripts/` 目录 |

## 链路

```
WT901 BLE 大腿/小腿  ──bleak──>  ble_to_blender.py  ──虚拟串口 COM1──>  [VSPD 配对]
                                                                              │
env_leg.blend  ──legcontrol_dual.py──>  UARTTable 读 COM2  ──>  驱动右腿骨骼
```

## 运行步骤

1. **虚拟串口**：用 "Configure Virtual Serial Port Driver"（VSPD）创建一对端口，例如 `COM1 <-> COM2`。
2. **Blender**：打开 `env_blender_leg/env_leg.blend` → Scripting 工作区 → 打开并运行 `legcontrol_dual.py`（先确认里面 `MCU_COM_PORT` 是 VSPD 配对的一侧，如 `COM2`）。
3. **Python**：打开命令行运行：
   ```
   python ble_to_blender.py --port COM1
   ```
   （`--port` 填 VSPD 配对的另一侧端口，与 Blender 端的相反；默认值已是 `COM1`，可省略不传）
4. **校准**：启动后自动等待 3 秒，用第一个有效姿态作为基准。之后抬腿/屈膝，观察 Blender 里右腿跟随。

## 关键点

- **父子分解**：大腿(UpperLeg)骨骼设绝对旋转 `q0`；小腿(LowerLeg)骨骼设相对大腿的旋转 `q1_rel = q0_inv * q1`（与 `armcontrol.py` 完全一致）。这样大腿转动不会"带着"小腿产生假弯曲。
- **坐标系**：`MOUNT` 矩阵把 WT901 轴转到身体坐标系（+X 右、+Y 前、+Z 上），与 `env_leg.blend` 的模型坐标系一致。
- **校准**：若静止时 Blender 腿就是歪的，重新站直后重启脚本（或改代码在特定按键时重新校准）。

## 常见问题

- **Blender 端报 `uartcomm` 找不到**：把 `uartcomm.py` 复制到 `.blend` 文件旁边的 `scripts/` 目录（即 `env_blender_leg/scripts/`）。
- **串口打不开**：确认 VSPD 配对成功、两边端口号与脚本一致、其他程序没有占用该端口。
- **腿部方向反了/轴不对**：调整 `ble_to_blender.py` 中 `MOUNT` 矩阵的正负号（每个传感器安装方向不同，可能需要微调）。
