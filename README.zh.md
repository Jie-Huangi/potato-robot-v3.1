# Potato Robot — 动态切块 v2

中文版 | [English Version](README.md)

本仓库交付 Delta 马铃薯切块单元的 **距离触发 + 编码器锁存 + 动态跟随/切块** 相关资源：Python 核心算法、STM32 烧录文件、以及 YOLO OBB 权重。

## 1. 概述

用**相机触发时刻的硬件编码器锁存**把视觉与皮带运动对齐，再把目标推算进 Delta 工作区；切块模式下运行短跟随状态机。

坐标链：`pixel → plane → encoder delta → robot`。

```text
皮带编码器 → M850 触发 → 相机 + M851 锁存 → YOLO OBB → 目标队列 → 动态位姿 → M701 跟随/切块
```

## 2. 功能

- **距离触发取帧** — 固件 `M850` 按皮带里程触发相机；可用 `M852` 手动触发。
- **硬件锁存对齐** — `M851` 返回触发时刻编码器快照 `E0`。
- **YOLO OBB 检测** — 权重位于 `code/yolo_weights/best.pt`。
- **目标队列与跨帧关联** — 同帧去重、匹配、锚点修正、里程清理（`code/core_dynamic_cut/`）。
- **编码器卡尔曼滤波** — 控制速度由位置 E 估计（固件 V 仅日志）。
- **运行模式（主机侧设计）** — `monitor` / `follow` / `cut`。
- **切块状态机** — `IDLE → SYNC → CUT_DOWN → HOLD → RECOVER`。
- **输送带控制** — `M815` / `M802`。

## 3. 目录结构

| 路径 | 说明 |
|------|------|
| [`code/core_dynamic_cut/`](code/core_dynamic_cut/) | 核心算法模块（中文注释） |
| [`code/firmware/`](code/firmware/) | STM32F407 烧录文件 + 固件说明 |
| [`code/yolo_weights/`](code/yolo_weights/) | YOLO OBB 权重（`best.pt`） |

## 4. 使用方法

### 4.1 烧录固件

见 [`code/firmware/README.md`](code/firmware/README.md)。在 `code/firmware/` 下：

```bash
openocd -f stlink.cfg -c "program core_stm32f407.elf verify reset exit"
```

烧录后主机口为 USB CDC（例如 `/dev/ttyACM0`）。

### 4.2 加载 YOLO 权重

```python
from pathlib import Path
from ultralytics import YOLO

model = YOLO(str(Path("code/yolo_weights/best.pt")))
```

### 4.3 使用核心算法

[`code/core_dynamic_cut/`](code/core_dynamic_cut/) 提供编码器速度滤波、动态位姿/工作窗口、目标关联与切块状态机。在主机应用中从该目录导入即可。说明见 [`code/core_dynamic_cut/README.md`](code/core_dynamic_cut/README.md)。

## 5. 运行模式（主机侧设计）

| 模式 | 行为 |
|------|------|
| `monitor` | 只做视觉/锁存/坐标链；不下发 M701 |
| `follow` | 在工作窗口内发送 M701 动态跟随 |
| `cut` | 短跟随切块状态机 |

## 6. 关键参数

常见主机/算法参数（见 `code/core_dynamic_cut/` 中的 `CutConfig` / `QueueConfig`，以及固件 `M850` / `M700`）：

| 参数 | 含义 |
|------|------|
| `trigger_interval_mm` | 相机触发的皮带间距 |
| `workspace_limit_x/y_mm` | 安全跟随窗口半幅 |
| `cut_station_x_mm` | 机器人坐标系切割站位 X |
| `cut_sync_start_distance_mm` / lead | 同步锁定与提前下刀 |
| `follow_feed_mm_s` / `follow_feed_z_mm_s` | XY / Z 跟随进给（固件 `M700`） |
| `conveyor_speed_mm_s` | 皮带速度（`M815`） |
| 关联/记忆门限 | 跨帧匹配与目标生命周期 |

## 7. 相关文档

- 核心算法：[`code/core_dynamic_cut/README.md`](code/core_dynamic_cut/README.md)
- 固件：[`code/firmware/README.md`](code/firmware/README.md)
- 权重：[`code/yolo_weights/README.md`](code/yolo_weights/README.md)
- 英文版：[`README.md`](README.md)
