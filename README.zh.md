# Potato Robot — 动态切块 v2

中文版 | [English Version](README.md)

本仓库说明与配套资源对应脚本：

```text
/home/jiehuang/potato-robot-v3/code/perception-planning-action/stm32_potato_dynamic_cut_v2.py
```

实现 **距离触发相机 + 编码器锁存 + 动态跟随/切块** 流水线。

## 1. 概述

本程序用**相机触发时刻的硬件编码器锁存**把视觉与皮带运动对齐，再把马铃薯目标推算进 Delta 工作区；在 cut 模式下运行短跟随切块状态机。

坐标链：`pixel → plane → encoder delta → robot`。

```text
皮带编码器 → M850 触发 → 海康相机 + M851 锁存 → YOLO OBB → 目标队列 → 动态位姿 → M701 跟随/切块
```

## 2. 实现功能

- **距离触发取帧** — 固件 `M850` 按皮带里程触发相机；可用 `M852` 手动补触发。
- **硬件锁存对齐** — 每帧通过 `M851` 取触发时刻编码器 `E0`。
- **YOLO OBB 检测** — 后台推理，主循环保持编码器 / M701 节奏。
- **目标队列与跨帧关联** — 同帧去重、匹配、锚点修正、里程清理。
- **编码器卡尔曼滤波** — 控制速度由位置 E 估计（固件 V 仅日志）。
- **三种运行模式** — `monitor` / `follow` / `cut`。
- **切块状态机** — `IDLE → SYNC → CUT_DOWN → HOLD → RECOVER`。
- **输送带控制** — `M815` / `M802`，启动确认 + 键盘切换。
- **调试复盘** — CSV 与标注图像输出。

## 3. 本仓库目录

| 路径 | 说明 |
|------|------|
| [`code/core_dynamic_cut/`](code/core_dynamic_cut/) | 核心算法摘录（中文注释） |
| [`code/firmware/`](code/firmware/) | STM32F407 烧录文件 + 英文固件说明 |
| [`code/yolo_weights/`](code/yolo_weights/) | YOLO OBB 权重（`best.pt`） |

## 4. 环境与依赖

- Python 3、OpenCV、NumPy、Ultralytics YOLO
- 海康 SDK（`MvCamera.py`）
- STM32 USB 串口（默认 `/dev/ttyACM0`，需读写权限）
- 标定文件 `calibration_current.json`（单应、平面→机器人、编码器/触发方向）
- YOLO OBB 权重（本仓库：[`code/yolo_weights/best.pt`](code/yolo_weights/best.pt)）

## 5. 使用方法

### 5.1 运行完整脚本

```bash
cd /home/jiehuang/potato-robot-v3/code/perception-planning-action
python3 stm32_potato_dynamic_cut_v2.py
```

**没有命令行参数**。启动前在 `main()` 里改：`run_mode`、`dry_run_motion`、`model_path`、`conveyor_speed_mm_s`、`work_z_mm` / `cut_z_mm` 等。

若使用本仓库权重，可将 `model_path` 设为：

```python
model_path=Path("/home/jiehuang/potato-robot-v3.1/code/yolo_weights/best.pt")
```

启动时会加载标定（若存在）、覆盖现场方向符号，并打印 preflight 摘要。

### 5.2 交互安全确认

| 输入 | 作用 |
|------|------|
| `HOME` | 确认后发送 `G28 S1` 回零 |
| `WORKZ` | IK 预览通过后移到工作平面 |
| `TOOLZERO` / `SKIPTOOL` | 末端执行器归零或跳过 |
| `BELT` | 启动输送带（运动模式且开启皮带控制时） |

### 5.3 运行中键盘

| 按键 | 作用 |
|------|------|
| `q` / Esc | 退出 |
| `s` | 保存最近一帧图像 |
| `c` | 启停输送带 |
| `t` | 手动触发相机（`M852`） |

## 6. 运行模式

| 模式 | 行为 |
|------|------|
| `monitor` | 只做视觉/锁存/坐标链；不下发 M701；不自动启皮带 |
| `follow` | 在工作窗口内发送 M701 动态跟随 |
| `cut` | 完整短跟随切块状态机（`main()` 默认） |

`dry_run_motion=True` 时不连接机械臂，只验证相机与坐标链。

## 7. 关键参数（在 `main()` 中）

| 参数 | 含义 |
|------|------|
| `trigger_interval_mm` | 相机触发的皮带间距 |
| `workspace_limit_x/y_mm` | 安全跟随窗口半幅 |
| `cut_station_x_mm` | 机器人坐标系切割站位 X |
| `cut_sync_start_distance_mm` / lead | 同步锁定与提前下刀 |
| `follow_feed_mm_s` / `follow_feed_z_mm_s` | XY / Z 跟随进给 |
| `conveyor_speed_mm_s` | 主机控制时的皮带速度 |
| `debug_log_enabled` 等 | CSV / 调试图输出开关 |

## 8. 输出与排查

调试会话目录：

```text
output/stm32_potato_dynamic_latch_v2/debug_<timestamp>/
```

常见 CSV：`events`、`frames`、`detections`、`follow`、`cut`、`timing`。

常见问题：

- 缺少标定 → 无有效 homography / plane→robot
- `/dev/ttyACM0` 串口权限不足
- 编码器未就绪（ID6 stale）
- latch 过期（`max_latch_age_ms`）
- YOLO 权重路径不存在

## 9. 相关文档

- 算法摘录：[`code/core_dynamic_cut/README.md`](code/core_dynamic_cut/README.md)
- 固件烧录说明（英文）：[`code/firmware/README.md`](code/firmware/README.md)
- 英文版：[`README.md`](README.md)
