# 动态切块核心算法

本目录提供动态切块流水线中的核心算法模块（含中文注释），可直接阅读或集成到主机程序。

| 文件 | 内容 |
|------|------|
| `encoder_kalman.py` | 编码器位置 → 带速的两状态卡尔曼滤波 |
| `dynamic_pose.py` | 平面↔机器人变换、动态位姿推算、工作窗口判断 |
| `target_queue_core.py` | 同帧去重、跨帧关联、锚点修正、里程清理 |
| `cut_state_machine.py` | 切块状态机：`IDLE→SYNC→CUT_DOWN→HOLD→RECOVER` |

依赖：Python 3；`dynamic_pose.py` 需要 NumPy。

示例：

```python
from encoder_kalman import EncoderKalmanFilter
from cut_state_machine import CutStateMachine, CutConfig

kf = EncoderKalmanFilter(accel_noise=50.0, meas_noise_mm=0.5, innovation_gate_mm=30.0)
sm = CutStateMachine(CutConfig())
```
