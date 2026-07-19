# 动态切块核心算法摘录

本目录从完整脚本 `stm32_potato_dynamic_cut_v2.py` 中抽出**关键算法函数**，并加上中文注释，便于阅读与对照。

> 这些文件**不能**替代完整运行程序（缺少相机、串口、YOLO、标定加载与主循环）。

| 文件 | 内容 |
|------|------|
| `encoder_kalman.py` | 编码器位置 → 带速的两状态卡尔曼滤波 |
| `dynamic_pose.py` | 平面↔机器人变换、动态位姿推算、工作窗口判断 |
| `target_queue_core.py` | 同帧去重、跨帧关联、锚点修正、里程清理 |
| `cut_state_machine.py` | 切块状态机：`IDLE→SYNC→CUT_DOWN→HOLD→RECOVER` |

完整可运行脚本路径（原工程）：

`/home/jiehuang/potato-robot-v3/code/perception-planning-action/stm32_potato_dynamic_cut_v2.py`
