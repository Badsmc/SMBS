# -*- coding: utf-8 -*-
"""
autosequencer/scoring.py
Объективные метрики ранжирования последовательностей гибки.

Перенесено из BendSpec (core/sequence_scoring.py).
Работает поверх FrozensetSequenceState.

ВАЖНО: без физической модели станка (backgauge, punch, die)
«правильность» последовательности не определяется. Метрики здесь —
это ПРОКСИ-признаки, не критерий оптимальности. Ранжирование
комбинирует их с мастерскими штрафами (autosequencer.solver),
а окончательное решение остаётся за пользователем.

Метрики:
  * orientation_changes — смена поддерева anchor между соседними гибами.
    Прокси для «переворота детали оператором».
  * depth_ping_pong — сумма |Δ depth ребёнка| между соседними гибами.
    Прокси для «перескоков между слоями гибки».
  * mean_leaf_position — средняя нормированная позиция листьев (deg=1).
    Прокси для «листья в конце». Шкала 0..1.

Все метрики — lower-is-better (кроме mean_leaf_position, где выше лучше,
поэтому в total она идёт со знаком минус).

Версия: BENDBEQ_SCORING_V1
"""

from __future__ import annotations

from collections import deque
from typing import Dict, FrozenSet, Iterable, List, Optional, Tuple

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


BendKey = Tuple[int, int]
State = FrozenSet[BendKey]


# =====================================================================
# Веса
# =====================================================================

class ScoringWeights(object):
    """Веса метрик.

    total = w_orientation * orient
          + w_depth_ping_pong * depth
          - w_leaf * leaf_norm

    Дефолты читаются из SEQUENCE_CONFIG, если ключи заданы.
    """

    def __init__(self,
                 w_orientation: Optional[float] = None,
                 w_depth_ping_pong: Optional[float] = None,
                 w_leaf: Optional[float] = None):
        if w_orientation is None:
            w_orientation = SEQUENCE_CONFIG.get(
                "scoring_w_orientation", 2.0)
        if w_depth_ping_pong is None:
            w_depth_ping_pong = SEQUENCE_CONFIG.get(
                "scoring_w_depth_ping_pong", 0.5)
        if w_leaf is None:
            w_leaf = SEQUENCE_CONFIG.get("scoring_w_leaf", 1.0)

        self.w_orientation = float(w_orientation)
        self.w_depth_ping_pong = float(w_depth_ping_pong)
        self.w_leaf = float(w_leaf)

    def to_dict(self) -> dict:
        return {
            "w_orientation": self.w_orientation,
            "w_depth_ping_pong": self.w_depth_ping_pong,
            "w_leaf": self.w_leaf,
        }

    def __repr__(self):
        return ("ScoringWeights(w_orient={}, w_depth={}, "
                "w_leaf={})").format(
            self.w_orientation, self.w_depth_ping_pong, self.w_leaf)


# =====================================================================
# Топологические помощники
# =====================================================================

def _child_of_bend(ss, bend_key: BendKey) -> Optional[int]:
    """face_index ребёнка гиба (дальняя от anchor панель), или None."""
    a, b = bend_key
    parent_of = ss._build_tree()
    if parent_of.get(a) == b:
        return a
    if parent_of.get(b) == a:
        return b
    return None


def _depth_from_anchor(ss) -> Dict[int, int]:
    """BFS-глубина каждой панели от anchor."""
    anchor = ss.g2.anchor_face_index
    if anchor is None:
        return {}
    depth = {anchor: 0}
    q = deque([anchor])
    while q:
        n = q.popleft()
        for nb in sorted(ss.g2.neighbors(n)):
            if nb in depth:
                continue
            depth[nb] = depth[n] + 1
            q.append(nb)
    return depth


def _subtree_root_of(ss, node: int) -> Optional[int]:
    """Первый ребёнок anchor на пути от anchor к node.

    Для самого anchor возвращает anchor.
    Если node не в дереве от anchor — None.
    """
    anchor = ss.g2.anchor_face_index
    if node == anchor:
        return anchor
    parent_of = ss._build_tree()
    path: List[int] = []
    n = node
    while parent_of.get(n) is not None:
        path.append(n)
        n = parent_of[n]
    if n != anchor:
        return None
    if path:
        return path[-1]
    return anchor


# =====================================================================
# Метрики
# =====================================================================

def _orientation_changes(order_forward: Iterable[BendKey], ss) -> int:
    """Число смен поддерева anchor между соседними гибами."""
    prev_sub = None
    changes = 0
    for k in order_forward:
        child = _child_of_bend(ss, k)
        if child is None:
            continue
        sub = _subtree_root_of(ss, child)
        if prev_sub is not None and sub != prev_sub:
            changes += 1
        prev_sub = sub
    return changes


def _depth_ping_pong(order_forward: Iterable[BendKey], ss) -> int:
    """Сумма |Δ depth| между соседними гибами."""
    depth = _depth_from_anchor(ss)
    prev_d = None
    total = 0
    for k in order_forward:
        child = _child_of_bend(ss, k)
        if child is None:
            continue
        d = depth.get(child, 0)
        if prev_d is not None:
            total += abs(d - prev_d)
        prev_d = d
    return total


def _mean_leaf_position(order_forward: Iterable[BendKey], ss) -> float:
    """Средняя нормированная позиция листьев (deg=1) в order_forward.

    Шкала 0..1 (0 = все листья первыми, 1 = все листья последними).
    Если листьев нет — 0.0.
    """
    order = list(order_forward)
    n = len(order)
    if n <= 1:
        return 0.0
    positions = []
    for i, k in enumerate(order):
        child = _child_of_bend(ss, k)
        if child is None:
            continue
        deg = len(ss.g2.neighbors(child))
        if deg == 1:
            positions.append(i)
    if not positions:
        return 0.0
    return sum(positions) / (len(positions) * (n - 1))


# =====================================================================
# Public API
# =====================================================================

def score_sequence(order_forward: List[BendKey],
                   ss,
                   weights: Optional[ScoringWeights] = None) -> dict:
    """Посчитать метрики и total score.

    Args:
        order_forward: list[(a, b)] — порядок гибки (empty -> full).
        ss:            FrozensetSequenceState.
        weights:       ScoringWeights или None (default из config).

    Returns:
        dict с метриками и total.
    """
    if weights is None:
        weights = ScoringWeights()
    orient = _orientation_changes(order_forward, ss)
    depth = _depth_ping_pong(order_forward, ss)
    leaf = _mean_leaf_position(order_forward, ss)

    total = (weights.w_orientation * orient
             + weights.w_depth_ping_pong * depth
             - weights.w_leaf * leaf)

    return {
        "order_length": len(order_forward),
        "orientation_changes": orient,
        "depth_ping_pong": depth,
        "mean_leaf_position": round(leaf, 4),
        "total": round(total, 4),
        "weights": weights.to_dict(),
    }


def rank_sequences(orders: List[List[BendKey]],
                   ss,
                   weights: Optional[ScoringWeights] = None
                   ) -> List[dict]:
    """Ранжировать список последовательностей по total (меньше = лучше).

    Returns:
        list[dict]: [{"order": [...], "score": {...}}, ...] отсортированный.
    """
    scored = []
    for o in orders:
        s = score_sequence(o, ss, weights=weights)
        scored.append({"order": list(o), "score": s})
    scored.sort(key=lambda x: x["score"]["total"])
    return scored


def deduplicate_orders(orders: List[List[BendKey]]) -> List[List[BendKey]]:
    """Убрать дубликаты последовательностей по каноническому ключу."""
    seen: set = set()
    out: List[List[BendKey]] = []
    for o in orders:
        key = tuple((min(a, b), max(a, b)) for (a, b) in o)
        if key in seen:
            continue
        seen.add(key)
        out.append(o)
    return out