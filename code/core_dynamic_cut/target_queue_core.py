#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目标队列关联核心（摘录自 stm32_potato_dynamic_cut_v2.DynamicTargetQueue）。

包含：同帧去重、跨帧最近邻匹配、锚点小幅修正、按皮带里程清理。
已去掉 DebugLogger / 相机帧存储等运行时依赖，便于阅读算法本身。
本文件为算法摘录，不能单独替代完整运行脚本。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class DetectedPotato:
    """单帧检测结果（中心点已映射到平面坐标前，此处只保留置信度占位）。"""

    confidence: float
    center_px: Tuple[float, float] = (0.0, 0.0)


@dataclass
class DynamicTarget:
    """跨帧跟踪的动态目标。"""

    debug_id: int
    plane_xy0: Tuple[float, float]
    encoder0_mm: float
    last_seen_encoder_mm: float
    confidence: float = 0.0
    match_count: int = 0
    last_match_residual_mm: float = 0.0
    action_done: bool = False
    action_done_reason: str = ""


@dataclass
class DuplicateCheck:
    """与视觉记忆中最近目标的距离检查结果。"""

    duplicate: bool
    target_debug_id: Optional[int]
    distance_mm: Optional[float]
    threshold_mm: Optional[float]


@dataclass
class PruneResult:
    """被清理目标及其原因。"""

    target: DynamicTarget
    reason: str
    traveled_mm: float


@dataclass
class QueueConfig:
    """队列关联相关参数（对应原 DynamicLatchConfig 子集）。"""

    association_gate_mm: float = 30.0
    association_gate_min_mm: float = 12.0
    association_gate_noise_mm: float = 18.0
    same_frame_merge_mm: float = 10.0
    vision_memory_mm: float = 300.0
    action_memory_mm: float = 550.0
    target_anchor_update_alpha: float = 0.25
    max_anchor_correction_mm: float = 5.0
    anchor_update_max_abs_x_mm: float = 55.0
    belt_sign: float = -1.0


class TargetQueueCore:
    """管理视觉目标记忆、跨帧匹配和动作候选生命周期。"""

    def __init__(self, config: QueueConfig) -> None:
        """保存匹配门限、记忆距离和输送带方向符号。"""
        self.targets: List[DynamicTarget] = []
        self.association_gate_mm = max(1.0, config.association_gate_mm)
        self.association_gate_min_mm = max(1.0, config.association_gate_min_mm)
        self.association_gate_noise_mm = max(1.0, config.association_gate_noise_mm)
        self.vision_memory_mm = max(10.0, config.vision_memory_mm)
        self.action_memory_mm = max(self.vision_memory_mm, config.action_memory_mm)
        self.anchor_update_alpha = min(1.0, max(0.0, config.target_anchor_update_alpha))
        self.max_anchor_correction_mm = max(0.0, config.max_anchor_correction_mm)
        self.anchor_update_max_abs_x_mm = max(1.0, config.anchor_update_max_abs_x_mm)
        self.belt_sign = -1.0 if config.belt_sign < 0.0 else 1.0
        self._next_debug_id = 1

    def allocate_debug_id(self) -> int:
        """分配单调递增的调试编号。"""
        debug_id = self._next_debug_id
        self._next_debug_id += 1
        return debug_id

    def merge_same_frame_plane_points(
        self,
        items: List[Tuple[DetectedPotato, Tuple[float, float]]],
        merge_distance_mm: float,
    ) -> List[Tuple[DetectedPotato, Tuple[float, float]]]:
        """合并同一帧内平面坐标过近的重复检测，只保留置信度最高的框。"""
        if len(items) <= 1:
            return items
        threshold = max(0.1, merge_distance_mm)
        kept_indices: List[int] = []
        dropped_indices = set()
        sorted_indices = sorted(range(len(items)), key=lambda index: items[index][0].confidence, reverse=True)
        for index in sorted_indices:
            if index in dropped_indices:
                continue
            _detection, plane_xy0 = items[index]
            nearest_kept_distance: Optional[float] = None
            for kept_index in kept_indices:
                _kept, kept_plane = items[kept_index]
                distance = math.hypot(plane_xy0[0] - kept_plane[0], plane_xy0[1] - kept_plane[1])
                if nearest_kept_distance is None or distance < nearest_kept_distance:
                    nearest_kept_distance = distance
            if nearest_kept_distance is not None and nearest_kept_distance < threshold:
                dropped_indices.add(index)
                continue
            kept_indices.append(index)
        return [items[index] for index in range(len(items)) if index not in dropped_indices]

    def match_frame_items(
        self,
        items: List[Tuple[DetectedPotato, Tuple[float, float]]],
        encoder0_mm: float,
    ) -> Dict[int, DynamicTarget]:
        """对一帧多个检测做一对一最近邻匹配，避免两个检测抢同一旧目标。"""
        candidates: List[Tuple[float, int, DynamicTarget]] = []
        for index, (_detection, plane_xy0) in enumerate(items):
            for target in self.targets:
                distance = self.distance_to_target_at_encoder(plane_xy0, target, encoder0_mm)
                threshold = self.association_threshold_for(target, encoder0_mm)
                if distance <= threshold:
                    candidates.append((distance, index, target))
        assignments: Dict[int, DynamicTarget] = {}
        used_detections = set()
        used_targets = set()
        for _distance, index, target in sorted(candidates, key=lambda item: item[0]):
            if index in used_detections or target.debug_id in used_targets:
                continue
            detection = items[index][0]
            assignments[id(detection)] = target
            used_detections.add(index)
            used_targets.add(target.debug_id)
        return assignments

    def update_target_from_detection(
        self,
        target: DynamicTarget,
        detection: DetectedPotato,
        plane_xy0: Tuple[float, float],
        encoder_mm: float,
    ) -> bool:
        """用新检测小幅修正锚点，不重置触发时刻 E0。

        视野边缘 (|x| 过大) 的观测受视差影响，只刷新置信度，不修正锚点。
        返回是否允许了锚点修正。
        """
        predicted_x = target.plane_xy0[0] + self.belt_sign * (encoder_mm - target.encoder0_mm)
        predicted_y = target.plane_xy0[1]
        residual_x = plane_xy0[0] - predicted_x
        residual_y = plane_xy0[1] - predicted_y
        residual_mm = math.hypot(residual_x, residual_y)
        anchor_allowed = abs(plane_xy0[0]) <= self.anchor_update_max_abs_x_mm
        correction_x = residual_x * self.anchor_update_alpha if anchor_allowed else 0.0
        correction_y = residual_y * self.anchor_update_alpha if anchor_allowed else 0.0
        correction_mm = math.hypot(correction_x, correction_y)
        if correction_mm > self.max_anchor_correction_mm > 0.0:
            scale = self.max_anchor_correction_mm / correction_mm
            correction_x *= scale
            correction_y *= scale
        target.confidence = float(detection.confidence)
        target.plane_xy0 = (target.plane_xy0[0] + correction_x, target.plane_xy0[1] + correction_y)
        target.last_seen_encoder_mm = encoder_mm
        target.match_count += 1
        target.last_match_residual_mm = residual_mm
        return anchor_allowed

    def add_or_match_frame(
        self,
        plane_detections: List[Tuple[DetectedPotato, Tuple[float, float]]],
        encoder_mm: float,
        same_frame_merge_mm: float,
    ) -> int:
        """一帧入队：同帧去重 → 跨帧匹配 → 新目标创建。返回新增目标数。"""
        items = self.merge_same_frame_plane_points(plane_detections, same_frame_merge_mm)
        assignments = self.match_frame_items(items, encoder_mm)
        added = 0
        for detection, plane_xy0 in items:
            target = assignments.get(id(detection))
            if target is not None:
                self.update_target_from_detection(target, detection, plane_xy0, encoder_mm)
                continue
            self.targets.append(
                DynamicTarget(
                    debug_id=self.allocate_debug_id(),
                    plane_xy0=plane_xy0,
                    encoder0_mm=encoder_mm,
                    last_seen_encoder_mm=encoder_mm,
                    confidence=float(detection.confidence),
                )
            )
            added += 1
        return added

    def distance_to_target_at_encoder(
        self,
        plane_xy0: Tuple[float, float],
        target: DynamicTarget,
        encoder0_mm: float,
    ) -> float:
        """把历史目标推算到当前锁存编码器时刻，返回与检测点的平面距离。"""
        existing_x, existing_y = self.target_plane_at_encoder(target, encoder0_mm)
        return math.hypot(plane_xy0[0] - existing_x, plane_xy0[1] - existing_y)

    def target_plane_at_encoder(self, target: DynamicTarget, encoder_mm: float) -> Tuple[float, float]:
        """锚点 + 皮带位移 → 指定编码器时刻的平面坐标。"""
        dx = self.belt_sign * (encoder_mm - target.encoder0_mm)
        return target.plane_xy0[0] + dx, target.plane_xy0[1]

    def association_threshold_for(self, target: DynamicTarget, encoder_mm: float) -> float:
        """根据丢帧距离给跨帧关联门限留少量余量，并限制最大值。"""
        traveled_since_seen = abs(encoder_mm - target.last_seen_encoder_mm)
        drift_allowance = min(8.0, traveled_since_seen * 0.02)
        threshold = max(self.association_gate_min_mm, self.association_gate_noise_mm + drift_allowance)
        return min(self.association_gate_mm, threshold)

    def prune(self, encoder_now_mm: float) -> List[PruneResult]:
        """删除已超出视觉记忆或动作生命周期的旧目标。"""
        kept: List[DynamicTarget] = []
        removed: List[PruneResult] = []
        for target in self.targets:
            traveled = abs(encoder_now_mm - target.encoder0_mm)
            if target.action_done:
                if traveled <= self.vision_memory_mm:
                    kept.append(target)
                else:
                    removed.append(PruneResult(target, "vision_memory_expired", traveled))
                continue
            if traveled <= self.action_memory_mm:
                kept.append(target)
            else:
                target.action_done = True
                target.action_done_reason = "ACTION_MEMORY_EXPIRED"
                removed.append(PruneResult(target, "action_memory_expired", traveled))
        self.targets = kept
        return removed

    def action_candidates(self) -> List[DynamicTarget]:
        """返回尚未完成或放弃动作的目标。"""
        return [target for target in self.targets if not target.action_done]
