#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编码器卡尔曼滤波核心（摘录自 stm32_potato_dynamic_cut_v2.EncoderKalmanFilter）。

作用：用可靠的编码器位置 E 估计带速，供动态跟随与切块控制使用。
背景：固件 M811 的速度字段 V 存在离散倍率错误，控制侧不直接使用 V。
本文件为算法摘录，不能单独替代完整运行脚本。
"""

from __future__ import annotations

from typing import Optional


class EncoderKalmanFilter:
    """两状态（位置/速度）卡尔曼滤波器：由位置序列导出可信带速。"""

    # 速度限幅，防止偶发野值把跟随指令冲飞
    VELOCITY_LIMIT_MM_S = 300.0

    def __init__(self, accel_noise: float, meas_noise_mm: float, innovation_gate_mm: float) -> None:
        """保存噪声参数，并把滤波器置为未初始化状态。"""
        self.accel_noise = max(1.0, accel_noise)
        self.meas_var = max(0.01, meas_noise_mm) ** 2
        self.innovation_gate_mm = max(1.0, innovation_gate_mm)
        self.t_s: Optional[float] = None
        self.d_mm = 0.0
        self.v_mm_s = 0.0
        self.p = [[1e6, 0.0], [0.0, 1e6]]
        self.outlier_streak = 0
        self.last_was_outlier = False

    def reset(self, d_mm: float, t_s: float) -> None:
        """用当前测量重置滤波器；速度从零重新收敛（约 2–3 个采样）。"""
        self.t_s = t_s
        self.d_mm = d_mm
        self.v_mm_s = 0.0
        self.p = [[self.meas_var, 0.0], [0.0, 1e4]]
        self.outlier_streak = 0

    def velocity(self) -> float:
        """返回限幅后的速度估计 (mm/s)。"""
        return max(-self.VELOCITY_LIMIT_MM_S, min(self.VELOCITY_LIMIT_MM_S, self.v_mm_s))

    def update(self, d_mm: float, t_s: float) -> float:
        """喂入一次位置测量，执行预测+更新，返回限幅后的速度估计 (mm/s)。"""
        self.last_was_outlier = False
        if self.t_s is None:
            self.reset(d_mm, t_s)
            return self.velocity()

        dt = t_s - self.t_s
        if dt <= 0.0:
            return self.velocity()
        # 采样间隔过大则重新初始化，避免错误外推
        if dt > 1.0:
            self.reset(d_mm, t_s)
            return self.velocity()

        # 匀速模型预测
        d_pred = self.d_mm + self.v_mm_s * dt
        v_pred = self.v_mm_s
        q = self.accel_noise ** 2
        p00 = self.p[0][0] + dt * (self.p[0][1] + self.p[1][0]) + dt * dt * self.p[1][1] + q * dt ** 4 / 4.0
        p01 = self.p[0][1] + dt * self.p[1][1] + q * dt ** 3 / 2.0
        p10 = self.p[1][0] + dt * self.p[1][1] + q * dt ** 3 / 2.0
        p11 = self.p[1][1] + q * dt * dt

        innovation = d_mm - d_pred
        # 创新门限：超限视为位置野值，连续多次才硬重置
        if abs(innovation) > self.innovation_gate_mm:
            self.last_was_outlier = True
            self.outlier_streak += 1
            if self.outlier_streak >= 5:
                self.reset(d_mm, t_s)
            else:
                self.t_s = t_s
                self.d_mm = d_pred
                self.v_mm_s = v_pred
                self.p = [[p00, p01], [p10, p11]]
            return self.velocity()

        self.outlier_streak = 0
        s = p00 + self.meas_var
        k0 = p00 / s
        k1 = p10 / s
        self.t_s = t_s
        self.d_mm = d_pred + k0 * innovation
        self.v_mm_s = v_pred + k1 * innovation
        self.p = [
            [(1.0 - k0) * p00, (1.0 - k0) * p01],
            [p10 - k1 * p00, p11 - k1 * p01],
        ]
        return self.velocity()
