# -*- coding: utf-8 -*-
"""
autosequencer/physical_solver.py

Единственный физически-корректный поиск последовательности гибки.

Версия: BENDBEQ_PHYSICAL_SOLVER_V1

Принцип:
    Оракул = engine.can_apply(shape, bend, step_idx, done_specs)
    — тот же, что использует validate_full_sequence. Каждый шаг
    проходит настоящую OCC-симуляцию (подвод пуансона + гиб +
    матрица + станина + self-collision + упор).

    Трансформация спеков будущих гибов делается ВНУТРИ поиска,
    как в validate_full_sequence. Без неё оракул видит устаревшие
    координаты и ложно падает на PUNCH_BEFORE.

Алгоритм:
    DFS с backtracking, dead-state memoization по frozenset.
    На каждом узле — перебор невыполненных кандидатов в
    эвристическом порядке; первый прошедший — рекурсия; провал
    всех — backtrack.

Гарантии:
    * Любая найденная последовательность физически валидна
      по построению (каждый шаг прошёл оракул).
    * Time budget: если не успели — partial=True, вернуть лучший
      префикс. Правильного ответа не выдумываем.
"""

from __future__ import annotations

import copy
import time
from typing import Any, Callable, Dict, FrozenSet, List, Optional

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}

try:
    import FreeCAD as App
    from FreeCAD import Vector
    _FC_OK = True
except ImportError:
    App = None
    Vector = None
    _FC_OK = False

# Единственная зависимость от физики — та же, что у валидатора.
from core.bend_simulator import transform_bend_spec_inplace


MODULE_VERSION = "BENDBEQ_PHYSICAL_SOLVER_V1"
if _FC_OK:
    try:
        App.Console.PrintMessage(
            "[physical_solver] Загружена версия: {}\n".format(
                MODULE_VERSION))
    except Exception:
        pass


# =====================================================================
# Вспомогательные
# =====================================================================

def _clone_spec(bend):
    """Клон BendSpec с независимыми Vector'ами и metadata.

    Дублирует _clone_bend_spec из sequence_validator — чтобы не
    тянуть validator в solver (циклическая зависимость). При
    рефакторинге вынести в core/bend_feature.py.
    """
    BendSpecCls = type(bend)
    return BendSpecCls(
        id=bend.id,
        name=bend.name,
        center=Vector(bend.center),
        axis=Vector(bend.axis),
        angle=float(bend.angle),
        length=float(bend.length),
        direction=getattr(bend, "direction", "up"),
        normal=Vector(getattr(bend, "normal", Vector(0, 0, 1))),
        up_hint=Vector(getattr(bend, "up_hint", Vector(0, 0, 1))),
        source=getattr(bend, "source", None),
        metadata=copy.deepcopy(getattr(bend, "metadata", {}) or {}),
    )


def _compute_force_ids(cur_bend, future_specs):
    """Sibling-forcing для multi-edge фич.

    Копирует логику из SequenceValidator.validate_full_sequence:
    будущие гибы, чей parent_feature совпадает с feature текущего
    и чей body_direction совпадает, должны быть трансформированы
    вместе с текущим.
    """
    cur_meta = getattr(cur_bend, "metadata", {}) or {}
    cur_feature = str(cur_meta.get("feature") or "")
    cur_side = str(cur_meta.get("body_direction") or "")
    force = set()
    if not cur_feature:
        return force
    for spec in future_specs:
        m = getattr(spec, "metadata", {}) or {}
        if str(m.get("parent_feature") or "") != cur_feature:
            continue
        f_side = str(m.get("body_direction") or "")
        if cur_side and f_side == cur_side:
            force.add(spec.id)
    return force


def _heuristic_key(bend):
    """Порядок кандидатов — только для скорости, не для корректности.

    Дефолт: по feature-depth DESC (глубокие раньше) — эвристика
    «от фланцев к базе», которая неплохо работает на U/C-каналах.
    Если depth в metadata нет — все получают 0, сортировка по id.
    """
    meta = getattr(bend, "metadata", {}) or {}
    try:
        depth = int(meta.get("depth", 0) or 0)
    except (TypeError, ValueError):
        depth = 0
    return (-depth, str(getattr(bend, "id", "")))


# =====================================================================
# Результат
# =====================================================================

class PhysicalSolution:
    """Результат physical_solver.

    Attributes:
        order:   List[str] — id гибов в порядке выполнения.
        partial: True, если time budget истёк и решение неполное.
        success: True, если найден полный физически валидный порядок.
        stats:   диагностика (oracle_calls, backtracks, nodes, ...).
    """

    def __init__(self, order, stats, partial):
        self.order = list(order)
        self.stats = dict(stats)
        self.partial = bool(partial)
        self.success = (not partial) and (len(order) > 0)

    def to_dict(self):
        return {
            "order": list(self.order),
            "partial": self.partial,
            "success": self.success,
            "stats": dict(self.stats),
        }

    def __repr__(self):
        return ("PhysicalSolution(success={}, partial={}, steps={}, "
                "stats={})").format(
            self.success, self.partial, len(self.order), self.stats)


# =====================================================================
# Главный вход
# =====================================================================

def solve_physical(
    bends: List[Any],
    flat_shape: Any,
    engine: Any,
    logger: Optional[Callable[[str], None]] = None,
    time_budget_sec: float = 120.0,
    on_progress: Optional[Callable[[int, int, int], None]] = None,
) -> PhysicalSolution:
    """DFS с backtracking поверх физического оракула.

    Args:
        bends:          List[BendSpec] — все гибы в системе flat.
        flat_shape:     Part.Shape — исходная плоская развёртка.
        engine:         объект с методом
                        can_apply(shape, bend, step_idx, done_specs)
                        -> res с .ok, .shape_after, .details
                        (тот же интерфейс, что использует solver.py).
        logger:         callable(str) — куда писать логи.
        time_budget_sec: жёсткий лимит времени поиска.
        on_progress:    callable(done, total, oracle_calls) — прогресс.

    Returns:
        PhysicalSolution.
    """
    _log = logger if callable(logger) else (lambda m: None)
    n = len(bends)
    if n == 0:
        return PhysicalSolution([], {"reason": "empty"}, partial=False)

    if not _FC_OK:
        _log("[PhysicalDFS] FreeCAD недоступен — поиск невозможен.")
        return PhysicalSolution([], {"reason": "no_fc"}, partial=True)

    bend_ids = [b.id for b in bends]
    id_to_bend = {b.id: b for b in bends}

    t0 = time.time()
    stats = {
        "oracle_calls": 0,
        "oracle_ok": 0,
        "backtracks": 0,
        "nodes": 0,
        "dead_states": 0,
        "timeout": False,
    }

    def _time_exceeded():
        return (time.time() - t0) >= time_budget_sec

    dead_states: set = set()
    best_partial: List[str] = []

    def _sorted_candidates(working_specs, done_set):
        cands = [working_specs[bid] for bid in bend_ids
                 if bid not in done_set and bid in working_specs]
        cands.sort(key=_heuristic_key)
        return cands

    def _dfs(done_set: FrozenSet[str],
             current_shape,
             working_specs: Dict[str, Any],
             path: List[str],
             done_specs: List[Any]):
        nonlocal best_partial
        stats["nodes"] += 1

        if _time_exceeded():
            stats["timeout"] = True
            return None

        if len(done_set) == n:
            return list(path)

        if len(path) > len(best_partial):
            best_partial = list(path)
            if on_progress:
                try:
                    on_progress(len(path), n, stats["oracle_calls"])
                except Exception:
                    pass

        if done_set in dead_states:
            stats["dead_states"] += 1
            return None

        candidates = _sorted_candidates(working_specs, done_set)

        for cand in candidates:
            if _time_exceeded():
                stats["timeout"] = True
                return None

            step_idx = len(path)
            stats["oracle_calls"] += 1
            res = engine.can_apply(
                current_shape, cand, step_idx, done_specs)

            if (not getattr(res, "ok", False)
                    or getattr(res, "shape_after", None) is None):
                continue
            stats["oracle_ok"] += 1

            # Готовим трансформированные спеки для поддерева.
            new_specs: Dict[str, Any] = {}
            for bid in bend_ids:
                if bid in done_set or bid == cand.id:
                    continue
                new_specs[bid] = _clone_spec(working_specs[bid])

            transform_info = (getattr(res, "details", {}) or {}
                              ).get("_transform_info")
            transform_ok = True
            if transform_info:
                future_specs = [new_specs[bid] for bid in bend_ids
                                if bid in new_specs]
                force_ids = _compute_force_ids(cand, future_specs)
                transform_info["force_ids"] = force_ids
                if force_ids:
                    _log("[PhysicalDFS] force_ids для [{}]: {}".format(
                        cand.id, sorted(force_ids)))
                for spec2 in future_specs:
                    try:
                        transform_bend_spec_inplace(spec2, transform_info)
                    except Exception as exc:
                        _log(
                            "[PhysicalDFS] transform failed for {}: {}"
                            .format(spec2.id, exc))
                        transform_ok = False
                        break
            if not transform_ok:
                continue

            new_done = done_set | {cand.id}
            new_done_specs = done_specs + [cand]
            path.append(cand.id)
            solution = _dfs(
                new_done, res.shape_after, new_specs,
                path, new_done_specs)
            if solution is not None:
                return solution
            path.pop()
            stats["backtracks"] += 1

        dead_states.add(done_set)
        return None

    initial_specs = {bid: _clone_spec(id_to_bend[bid])
                     for bid in bend_ids}

    _log("[PhysicalDFS] старт: bends={}, budget={:.0f}s".format(
        n, time_budget_sec))

    solution = _dfs(frozenset(), flat_shape, initial_specs, [], [])

    elapsed = time.time() - t0
    stats["elapsed_sec"] = elapsed

    if solution is not None:
        _log("[PhysicalDFS] OK: {} за {:.1f}s "
             "(oracle={}, ok={}, backtracks={}, nodes={}, dead={})"
             .format(
                 solution, elapsed,
                 stats["oracle_calls"], stats["oracle_ok"],
                 stats["backtracks"], stats["nodes"],
                 stats["dead_states"]))
        return PhysicalSolution(solution, stats, partial=False)

    _log("[PhysicalDFS] НЕ НАЙДЕНО за {:.1f}s "
         "(лучший префикс {}/{}, oracle={}, backtracks={}, "
         "nodes={}, dead={})".format(
             elapsed, len(best_partial), n,
             stats["oracle_calls"], stats["backtracks"],
             stats["nodes"], stats["dead_states"]))
    return PhysicalSolution(best_partial, stats, partial=True)