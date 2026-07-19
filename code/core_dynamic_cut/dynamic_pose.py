#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动态位姿与工作窗口判断（摘录自 stm32_potato_dynamic_cut_v2）。

对应原类/函数：
- PlaneRobotTransform
- DynamicLatchApp.compute_dynamic_pose
- DynamicLatchApp.evaluate_target_window
- DynamicLatchApp.select_ready_evaluation

作用：把「拍照时刻平面锚点 + 编码器位移」推算到当前机器人坐标，
并判断目标是否已进入可跟随/可切割的工作窗口。
本文件为算法摘录，不能单独替代完整运行脚本。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class EncoderSnapshot:
    """编码器快照：距离、速度（控制用速度建议来自卡尔曼滤波）。"""

    distance_x_mm: Optional[float]
    velocity_x_mm_s: Optional[float]


@dataclass
class DynamicTarget:
    """动态目标锚点：拍照锁存时刻的平面坐标与编码器读数。"""

    plane_xy0: Tuple[float, float]
    encoder0_mm: float


@dataclass
class WindowEvaluation:
    """单个目标在当前编码器时刻的工作窗口评估结果。"""

    target: DynamicTarget
    plane_xy: Tuple[float, float]
    robot_xy: Tuple[float, float]
    robot_vxy: Tuple[float, float]
    window_state: str
    distance_to_entry_mm: Optional[float]
    distance_to_exit_mm: Optional[float]


class PlaneRobotTransform:
    """把输送链平面坐标转换成机器人基坐标 XY，并映射速度。"""

    def __init__(self, matrix: np.ndarray) -> None:
        """保存 2x3 或 3x3 的 plane_to_robot 变换矩阵。"""
        array = np.asarray(matrix, dtype=np.float32)
        if array.shape == (3, 3):
            self.matrix = array
            self.linear = array[:2, :2]
        elif array.shape == (2, 3):
            self.matrix = np.vstack([array, np.array([0.0, 0.0, 1.0], dtype=np.float32)])
            self.linear = array[:, :2]
        else:
            raise ValueError(f"plane_to_robot_2d 形状错误: {array.shape}")

    def point(self, plane_xy: Tuple[float, float]) -> Tuple[float, float]:
        """平面点 → 机器人 XY。"""
        vector = np.array([plane_xy[0], plane_xy[1], 1.0], dtype=np.float32)
        mapped = self.matrix @ vector
        if abs(float(mapped[2])) > 1e-6:
            mapped = mapped / mapped[2]
        return float(mapped[0]), float(mapped[1])

    def velocity(self, plane_vxy: Tuple[float, float]) -> Tuple[float, float]:
        """平面速度 → 机器人 XY 速度（只用线性部分）。"""
        mapped = self.linear @ np.array([plane_vxy[0], plane_vxy[1]], dtype=np.float32)
        return float(mapped[0]), float(mapped[1])


def compute_dynamic_pose(
    target: DynamicTarget,
    encoder: EncoderSnapshot,
    plane_robot: PlaneRobotTransform,
    belt_sign: float,
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    """用 E_now - E0 计算目标当前平面坐标、机器人坐标和机器人速度。

    坐标链：plane_xy0 + belt_sign*(E_now-E0) → plane → robot。
    """
    encoder_now = 0.0 if encoder.distance_x_mm is None else encoder.distance_x_mm
    encoder_v = 0.0 if encoder.velocity_x_mm_s is None else encoder.velocity_x_mm_s
    traveled = belt_sign * (encoder_now - target.encoder0_mm)
    plane_xy = (target.plane_xy0[0] + traveled, target.plane_xy0[1])
    robot_xy = plane_robot.point(plane_xy)
    robot_vxy = plane_robot.velocity((belt_sign * encoder_v, 0.0))
    return plane_xy, robot_xy, robot_vxy


def evaluate_target_window(
    target: DynamicTarget,
    encoder: EncoderSnapshot,
    plane_robot: PlaneRobotTransform,
    belt_sign: float,
    workspace_limit_x_mm: float,
    workspace_limit_y_mm: float,
) -> WindowEvaluation:
    """判断目标相对工作区是：未到、在窗口内、已过线、还是 Y 超限。"""
    plane_xy, robot_xy, robot_vxy = compute_dynamic_pose(target, encoder, plane_robot, belt_sign)
    x_limit = workspace_limit_x_mm
    y_limit = workspace_limit_y_mm
    robot_x, robot_y = robot_xy
    velocity_x = robot_vxy[0]

    if abs(robot_y) > y_limit:
        return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "Y_OUTSIDE", None, None)

    # 正向流：从小 X 往大 X；负向流对称处理
    if velocity_x >= 0.0:
        if robot_x < -x_limit:
            return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "WAIT_UPSTREAM", -x_limit - robot_x, x_limit - robot_x)
        if robot_x > x_limit:
            return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "PASSED_DOWNSTREAM", 0.0, robot_x - x_limit)
        return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "IN_WORKSPACE", 0.0, x_limit - robot_x)

    if robot_x > x_limit:
        return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "WAIT_UPSTREAM", robot_x - x_limit, robot_x + x_limit)
    if robot_x < -x_limit:
        return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "PASSED_DOWNSTREAM", 0.0, -x_limit - robot_x)
    return WindowEvaluation(target, plane_xy, robot_xy, robot_vxy, "IN_WORKSPACE", 0.0, robot_x + x_limit)


def select_ready_evaluation(evaluations: List[WindowEvaluation]) -> Optional[WindowEvaluation]:
    """从已进入工作窗口的目标中，选最靠近下游出口的一个（follow 模式用）。"""
    if not evaluations:
        return None
    positive_flow = sum(1 for evaluation in evaluations if evaluation.robot_vxy[0] >= 0.0)
    if positive_flow >= len(evaluations) / 2.0:
        return max(evaluations, key=lambda evaluation: evaluation.robot_xy[0])
    return min(evaluations, key=lambda evaluation: evaluation.robot_xy[0])
