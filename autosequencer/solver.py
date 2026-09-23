# -*- coding: utf-8 -*-
"""
autosequencer/solver.py
Алгоритмы forward-поиска: greedy + weighted A*.

v2.2 (workshop penalties + weighted A*):
  * Штрафы за переналадки станка: flip, rotation, tool_change,
    tool_length, backgauge, feature, contact.
  * Weighted A* (W=2).
  * Коллизия — отсечка, а не штраф.

Версия: BENDBEQ_AUTOSEQ_SOLVER_V1
"""

from __future__ import annotations

import heapq
import itertools
import time
from typing import Dict, List, Optional, Set, Tuple

from .topology import feature_dependency_graph

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


_DEFAULT_PENALTIES = {
    "base": 1.0,
    "flip": 2.0,
    "rotation": 1.0,
    "tool_change": 2.0,
    "tool_length": 1.0,
    "backgauge": 0.5,
    "feature": 0.3,
    "contact": 3.0,
}


def _penalties() -> dict:
    cfg = dict(_DEFAULT_PENALTIES)
    for k in list(cfg.keys()):
        v = SEQUENCE_CONFIG.get("auto_penalty_{}".format(k))
        if v is None:
            continue
        try:
            cfg[k] = float(v)
        except (TypeError, ValueError):
            pass
    return cfg


def _is_ready(bend, bends, mask, feature_of_bend, known_features) -> bool:
    """Готов ли bend к выполнению: все предки его feature выполнены."""
    meta = getattr(bend, "metadata", {}) or {}
    parent = str(meta.get("parent_feature") or "").strip()
    if not parent or parent not in known_features:
        return True
    for i, b in enumerate(bends):
        if feature_of_bend.get(b.id) == parent and not mask[i]:
            return False
    return True


def _axis_parallel_factor(a, b) -> float:
    """|cos| между осями (1.0 — параллельны, 0 — перпендикулярны)."""
    try:
        if hasattr(a, "dot") and hasattr(a, "Length"):
            la = float(a.Length)
            lb = float(b.Length)
            if la < 1e-9 or lb < 1e-9:
                return 1.0
            return abs(float((a / la).dot(b / lb)))
    except Exception:
        pass
    try:
        ax, ay, az = float(a[0]), float(a[1]), float(a[2])
        bx, by, bz = float(b[0]), float(b[1]), float(b[2])
        la = (ax * ax + ay * ay + az * az) ** 0.5
        lb = (bx * bx + by * by + bz * bz) ** 0.5
        if la < 1e-9 or lb < 1e-9:
            return 1.0
        return abs((ax * bx + ay * by + az * bz) / (la * lb))
    except Exception:
        return 1.0


def _radius_of(bend) -> Optional[float]:
    meta = getattr(bend, "metadata", {}) or {}
    try:
        v = meta.get("radius")
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def _flange_of(bend) -> Optional[float]:
    meta = getattr(bend, "metadata", {}) or {}
    try:
        v = meta.get("flange_length_mm")
        return float(v) if v is not None else None
    except (TypeError, ValueError):
        return None


def _feature_of(bend) -> str:
    meta = getattr(bend, "metadata", {}) or {}
    return str(meta.get("feature") or "").strip()


def _pre_cost(prev_bend, next_bend, cfg: dict) -> float:
    """Стоимость перехода от prev_bend к next_bend."""
    base = cfg["base"]
    if prev_bend is None:
        return base

    cost = base

    pd = str(getattr(prev_bend, "direction", "up")).lower()
    nd = str(getattr(next_bend, "direction", "up")).lower()
    if pd != nd:
        cost += cfg["flip"]

    try:
        a1 = getattr(prev_bend, "axis", None)
        a2 = getattr(next_bend, "axis", None)
        if a1 is not None and a2 is not None:
            parallel = _axis_parallel_factor(a1, a2)
            cost += (1.0 - parallel) * cfg["rotation"]
    except Exception:
        pass

    pr = _radius_of(prev_bend)
    nr = _radius_of(next_bend)
    if pr is not None and nr is not None and abs(pr - nr) > 1e-3:
        cost += cfg["tool_change"]

    try:
        pl = float(getattr(prev_bend, "length", 0.0) or 0.0)
        nl = float(getattr(next_bend, "length", 0.0) or 0.0)
        if abs(pl - nl) > 1.0:
            cost += cfg["tool_length"]
    except Exception:
        pass

    pfl = _flange_of(prev_bend)
    nfl = _flange_of(next_bend)
    if pfl is not None and nfl is not None and abs(pfl - nfl) > 1.0:
        cost += cfg["backgauge"]

    pf = _feature_of(prev_bend)
    nf = _feature_of(next_bend)
    if pf and nf and pf != nf:
        cost += cfg["feature"]

    return cost


def _post_cost(step_result, cfg: dict) -> float:
    if step_result is None:
        return 0.0
    if getattr(step_result, "is_contact", False):
        return cfg["contact"]
    return 0.0


def _make_topological_heuristic(bends):
    """Счётчик невыполненных предков feature — admissible."""
    graph, feature_of_bend = feature_dependency_graph(bends)

    feature_indices: Dict[str, List[int]] = {}
    for i, b in enumerate(bends):
        feat = feature_of_bend.get(b.id)
        if feat:
            feature_indices.setdefault(feat, []).append(i)

    def feat_done(mask, feat):
        idxs = feature_indices.get(feat)
        if not idxs:
            return True
        return all(mask[i] for i in idxs)

    def h(mask):
        total = 0
        for feat, parents in graph.items():
            if feat_done(mask, feat):
                continue
            for p in parents:
                if not feat_done(mask, p):
                    total += 1
                    break
        return total

    return h


# =====================================================================
# Greedy forward
# =====================================================================

def greedy_sequence(bends, flat_shape, engine,
                     allow_partial: bool = True,
                     logger=None) -> List[str]:
    """Forward greedy со штрафами.

    Returns:
        List[bend_id].
    """
    _log = logger if callable(logger) else (lambda m: None)
    n = len(bends)
    if n == 0:
        return []

    cfg = _penalties()
    _, feature_of_bend = feature_dependency_graph(bends)
    known_features = set(feature_of_bend.values())

    mask = [False] * n
    current_shape = flat_shape
    order: List[str] = []
    already_done: List = []
    step_idx = 0
    last_bend = None

    while not all(mask):
        candidates = []
        for i, bend in enumerate(bends):
            if mask[i]:
                continue
            if not _is_ready(bend, bends, mask,
                              feature_of_bend, known_features):
                continue
            pen = _pre_cost(last_bend, bend, cfg)
            candidates.append((pen, i, bend))

        if not candidates:
            _log("[Greedy] Тупик: готовых кандидатов нет. Осталось: "
                 "{}".format([b.id for i, b in enumerate(bends)
                               if not mask[i]]))
            break

        candidates.sort(key=lambda x: (x[0], x[1]))

        advanced = False
        for _pen, i, bend in candidates:
            res = engine.can_apply(current_shape, bend, step_idx,
                                    already_done)
            if res.ok and res.shape_after is not None:
                mask[i] = True
                order.append(bend.id)
                current_shape = res.shape_after
                already_done.append(bend)
                step_idx += 1
                last_bend = bend
                advanced = True
                contact_tag = ""
                if getattr(res, "is_contact", False):
                    contact_tag = " [contact]"
                _log("[Greedy] шаг {}: {} ({:.1f}° {}){}".format(
                    step_idx, bend.id, bend.angle, bend.direction,
                    contact_tag))
                break

        if not advanced:
            _log("[Greedy] Тупик после {}/{} гибов. Осталось: "
                 "{}".format(len(order), n,
                              [b.id for i, b in enumerate(bends)
                               if not mask[i]]))
            break

    if not allow_partial and len(order) != n:
        _log("[Greedy] Частичная последовательность — возврат [].")
        return []
    return order


# =====================================================================
# Weighted A* forward
# =====================================================================

class _State:
    __slots__ = ("mask", "shape", "order", "last_bend")

    def __init__(self, mask, shape, order, last_bend=None):
        self.mask = mask
        self.shape = shape
        self.order = order
        self.last_bend = last_bend


def astar_sequence(bends, flat_shape, engine,
                    max_iterations: int = 2000,
                    max_seconds: Optional[float] = 60.0,
                    allow_partial: bool = True,
                    logger=None,
                    cancel_check=None,
                    progress_callback=None,
                    heuristic_weight: float = 2.0) -> List[str]:
    """Weighted A* forward.

    Returns:
        List[bend_id].
    """
    _log = logger if callable(logger) else (lambda m: None)
    n = len(bends)
    if n == 0:
        return []

    cfg = _penalties()
    _, feature_of_bend = feature_dependency_graph(bends)
    known_features = set(feature_of_bend.values())
    heuristic = _make_topological_heuristic(bends)

    start_time = time.time()
    initial = _State(tuple([False] * n), flat_shape, tuple(),
                      last_bend=None)

    h0 = heuristic(initial.mask)
    tie_breaker = itertools.count()
    heap = [(heuristic_weight * h0, 0.0, next(tie_breaker), initial)]

    visited: Set[tuple] = set()
    iterations = 0
    best_partial: List[str] = []
    stop_reason: Optional[str] = None

    while heap and iterations < max_iterations:
        if max_seconds is not None and max_seconds > 0:
            elapsed = time.time() - start_time
            if elapsed > max_seconds:
                stop_reason = "time budget {:.0f}s exhausted".format(
                    max_seconds)
                _log("[A*] Остановлено: {}".format(stop_reason))
                break

        if cancel_check and cancel_check():
            stop_reason = "cancelled by user"
            _log("[A*] Остановлено: {}".format(stop_reason))
            break

        iterations += 1
        _priority, g, _, state = heapq.heappop(heap)

        if state.mask in visited:
            continue
        visited.add(state.mask)

        if len(state.order) > len(best_partial):
            best_partial = list(state.order)
            elapsed = time.time() - start_time
            _log("[A*] iter={} лучший префикс: {}/{} g={:.2f} "
                 "({:.1f}s)".format(iterations, len(best_partial), n,
                                     g, elapsed))
            if progress_callback:
                try:
                    progress_callback(iterations, len(best_partial), n)
                except Exception:
                    pass

        if all(state.mask):
            elapsed = time.time() - start_time
            _log("[A*] Полная последовательность: {} "
                 "(cost={:.2f}, {:.1f}s)".format(
                     list(state.order), g, elapsed))
            return list(state.order)

        step_idx = len(state.order)
        already_done = [b for i, b in enumerate(bends) if state.mask[i]]

        for i, bend in enumerate(bends):
            if state.mask[i]:
                continue
            if not _is_ready(bend, bends, state.mask,
                              feature_of_bend, known_features):
                continue

            res = engine.can_apply(state.shape, bend, step_idx,
                                    already_done)
            if not res.ok or res.shape_after is None:
                continue

            new_mask = list(state.mask)
            new_mask[i] = True
            new_order = state.order + (bend.id,)
            new_state = _State(tuple(new_mask), res.shape_after,
                                new_order, last_bend=bend)

            step_cost = (_pre_cost(state.last_bend, bend, cfg)
                          + _post_cost(res, cfg))
            g_new = g + step_cost
            h_new = heuristic(new_state.mask)
            heapq.heappush(
                heap,
                (g_new + heuristic_weight * h_new,
                 g_new, next(tie_breaker), new_state))

    _log("[A*] Завершено: итераций={}, время={:.1f}s, "
         "лучший префикс={}/{}".format(
             iterations, time.time() - start_time,
             len(best_partial), n)
         + (" ({})".format(stop_reason) if stop_reason else ""))

    if not allow_partial and len(best_partial) != n:
        return []
    return best_partial