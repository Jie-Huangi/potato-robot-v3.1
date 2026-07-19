#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""短跟随切块状态机核心（摘录自 stm32_potato_dynamic_cut_v2.DynamicLatchApp）。

状态流：IDLE → SYNC → CUT_DOWN → HOLD → RECOVER → IDLE

本文件把「何时切换状态 / 目标 Z 是多少」整理为纯逻辑，去掉串口下发与调试复盘。
对应原方法：compute_cut_progress、effective_sync_start_mm、step_cut_state 中的转移条件、
follow_error_converged / cut_z 到位判断思路。
本文件为算法摘录，不能单独替代完整运行脚本。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class CutConfig:
    """切块相关参数（对应原 DynamicLatchConfig 子集）。"""

    cut_station_x_mm: float = 0.0
    cut_sync_start_distance_mm: float = 55.0
    cut_z_lead_ms: float = 120.0
    cut_z_lead_max_mm: float = 30.0
    cut_late_window_mm: float = 20.0
    cut_sync_min_ms: int = 200
    cut_ready_err_mm: float = 3.0
    cut_ready_err_speed_gain: float = 0.04
    cut_down_settle_ms: int = 120
    cut_hold_ms: int = 100
    cut_recover_ms: int = 280
    cut_down_timeout_ms: int = 600
    cut_cooldown_ms: int = 120
    work_z_mm: float = -300.0
    cut_z_mm: float = -380.0
    recover_z_mm: float = -300.0
    workspace_limit_x_mm: float = 95.0


@dataclass
class CutStepResult:
    """一次状态机步进后的输出（供上层决定是否发 M701）。"""

    state: str
    action: str
    z_target: float
    force_send: bool
    progress_mm: float
    lead_mm: float
    event: Optional[str] = None  # 如 cut_lock / cut_done / cut_missed / cut_down_timeout


class CutStateMachine:
    """短跟随切块状态机：同步跟随 → 提前下刀 → 保持 → 抬刀。"""

    def __init__(self, config: CutConfig) -> None:
        """初始化为空闲状态。"""
        self.config = config
        self.state = "IDLE"
        self.state_enter_s = 0.0
        self.cut_depth_reached_s: Optional[float] = None
        self.cut_last_done_s = 0.0
        self.follow_err_history: List[Tuple[float, float]] = []
        self.z_sample_history: List[Tuple[float, bool]] = []

    def enter(self, state: str, now_s: float) -> None:
        """切换状态并记录进入时间；进入 CUT_DOWN/RECOVER 时清空 Z 采样。"""
        if self.state == state:
            return
        self.state = state
        self.state_enter_s = now_s
        if state in ("CUT_DOWN", "RECOVER"):
            self.z_sample_history = []

    def elapsed_ms(self, now_s: float) -> float:
        """当前状态已持续毫秒数。"""
        return (now_s - self.state_enter_s) * 1000.0

    def compute_cut_progress(
        self,
        robot_x: float,
        robot_vx: float,
    ) -> Tuple[float, float]:
        """相对切割站位的进度（下游为正）与提前下刀距离。"""
        flow_sign = 1.0 if robot_vx >= 0.0 else -1.0
        progress_mm = flow_sign * (robot_x - self.config.cut_station_x_mm)
        lead_mm = abs(robot_vx) * self.config.cut_z_lead_ms / 1000.0
        lead_mm = min(max(0.0, lead_mm), self.config.cut_z_lead_max_mm)
        return progress_mm, lead_mm

    def effective_sync_start_mm(self, velocity_mm_s: float) -> float:
        """按带速计算同步锁定提前距离，保证高速仍有收敛与下刀时间。"""
        speed = abs(velocity_mm_s)
        lead_mm = min(speed * self.config.cut_z_lead_ms / 1000.0, self.config.cut_z_lead_max_mm)
        required = lead_mm + speed * self.config.cut_sync_min_ms / 1000.0 + 10.0
        return min(max(required, self.config.cut_sync_start_distance_mm), self.config.workspace_limit_x_mm)

    def sync_ready_tolerance_mm(self, velocity_mm_s: float) -> float:
        """按带速放宽同步收敛容差（高速纹波更大）。"""
        return self.config.cut_ready_err_mm + abs(velocity_mm_s) * self.config.cut_ready_err_speed_gain

    def note_follow_error(self, now_s: float, err_xy_mm: float) -> None:
        """记录一次跟随 XY 误差（需由上层从 M702 读入）。"""
        self.follow_err_history.append((now_s, err_xy_mm))
        if len(self.follow_err_history) > 2:
            self.follow_err_history = self.follow_err_history[-2:]

    def follow_error_converged(self, tolerance_mm: float) -> bool:
        """连续两次误差均小于容差，且采样时间都在当前状态进入之后。"""
        if len(self.follow_err_history) < 2:
            return False
        for read_s, err_mm in self.follow_err_history[-2:]:
            if read_s < self.state_enter_s or err_mm > tolerance_mm:
                return False
        return True

    def note_z_sample(self, now_s: float, sample_ok: bool) -> None:
        """记录一次 Z 到位采样（G93+ErrZ 由上层判定后传入）。"""
        self.z_sample_history.append((now_s, sample_ok))
        if len(self.z_sample_history) > 4:
            self.z_sample_history = self.z_sample_history[-4:]

    def z_reached_converged(self) -> bool:
        """Z 连续两次采样均到位。"""
        if len(self.z_sample_history) < 2:
            return False
        for read_s, sample_ok in self.z_sample_history[-2:]:
            if read_s < self.state_enter_s or not sample_ok:
                return False
        return True

    def can_lock_candidate(self, progress_mm: float, robot_vx: float, now_s: float) -> bool:
        """目标是否进入同步窗口且未过晚到线，且冷却结束。"""
        if now_s - self.cut_last_done_s < self.config.cut_cooldown_ms / 1000.0:
            return False
        if progress_mm > self.config.cut_late_window_mm:
            return False
        return progress_mm >= -self.effective_sync_start_mm(robot_vx)

    def step(
        self,
        now_s: float,
        in_workspace: bool,
        robot_x: float,
        robot_vx: float,
    ) -> CutStepResult:
        """根据进度与反馈推进状态，返回本周期应跟踪的 Z 与事件。"""
        progress_mm, lead_mm = self.compute_cut_progress(robot_x, robot_vx)

        if not in_workspace:
            self.enter("IDLE", now_s)
            return CutStepResult("IDLE", "abort", self.config.work_z_mm, False, progress_mm, lead_mm, "left_workspace")

        # 空闲：若进入同步窗口则锁定
        if self.state == "IDLE":
            if self.can_lock_candidate(progress_mm, robot_vx, now_s):
                self.enter("SYNC", now_s)
                self.cut_depth_reached_s = None
                return CutStepResult("SYNC", "sync", self.config.work_z_mm, False, progress_mm, lead_mm, "cut_lock")
            return CutStepResult("IDLE", "wait", self.config.work_z_mm, False, progress_mm, lead_mm, None)

        if self.state == "SYNC" and progress_mm > self.config.cut_late_window_mm:
            self.enter("IDLE", now_s)
            return CutStepResult("IDLE", "abort", self.config.work_z_mm, False, progress_mm, lead_mm, "cut_missed")

        force = False
        z_target = self.config.work_z_mm
        action = "sync"
        event: Optional[str] = None

        if self.state == "SYNC":
            sync_long_enough = self.elapsed_ms(now_s) >= self.config.cut_sync_min_ms
            converged = self.follow_error_converged(self.sync_ready_tolerance_mm(robot_vx))
            # 到达提前下刀线，且同步足够久、跟随已收敛 → 下刀
            if progress_mm >= -lead_mm and sync_long_enough and converged:
                self.enter("CUT_DOWN", now_s)
                self.cut_depth_reached_s = None
                z_target = self.config.cut_z_mm
                action = "cut_down"
                force = True
                event = "cut_down"
            else:
                z_target = self.config.work_z_mm
                action = "sync"

        elif self.state == "CUT_DOWN":
            z_target = self.config.cut_z_mm
            action = "cut_down"
            if self.cut_depth_reached_s is None and self.z_reached_converged():
                self.cut_depth_reached_s = now_s
                event = "cut_depth_reached"
            if self.cut_depth_reached_s is not None:
                if (now_s - self.cut_depth_reached_s) * 1000.0 >= self.config.cut_down_settle_ms:
                    self.enter("HOLD", now_s)
                    action = "hold"
                    event = "hold"
            elif self.elapsed_ms(now_s) >= self.config.cut_down_timeout_ms:
                self.enter("IDLE", now_s)
                return CutStepResult("IDLE", "abort", self.config.recover_z_mm, True, progress_mm, lead_mm, "cut_down_timeout")

        elif self.state == "HOLD":
            z_target = self.config.cut_z_mm
            action = "hold"
            if self.elapsed_ms(now_s) >= self.config.cut_hold_ms:
                self.enter("RECOVER", now_s)
                z_target = self.config.recover_z_mm
                action = "recover"
                force = True
                event = "recover"

        elif self.state == "RECOVER":
            z_target = self.config.recover_z_mm
            action = "recover"
            recovered = self.z_reached_converged()
            if recovered or self.elapsed_ms(now_s) >= self.config.cut_recover_ms:
                self.cut_last_done_s = now_s
                self.enter("IDLE", now_s)
                return CutStepResult("IDLE", "done", self.config.recover_z_mm, False, progress_mm, lead_mm, "cut_done")

        return CutStepResult(self.state, action, z_target, force, progress_mm, lead_mm, event)
