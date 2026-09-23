# -*- coding: utf-8 -*-
"""
autosequencer/__init__.py
Фасад Auto-Sequencer.

Версия: BENDBEQ_AUTOSEQ_INIT_V2

Стратегии:
    "physical"        -> DFS с backtracking поверх физического оракула
                         (sequence_validator.validate_step). Требует
                         bends_flat + flat_shape + tool adapters.
                         ЕДИНСТВЕННЫЙ алгоритм, дающий физически
                         валидный результат по построению.
    "forward_greedy"  -> forward greedy (legacy)
    "forward_astar"   -> forward weighted A* (legacy)
    "backward_greedy" -> backward greedy (legacy)
    "backward_astar"  -> backward A* (legacy)
    "hybrid"          -> physical при наличии bends_flat, иначе
                         пробует forward и backward.

Контракт:
    Если переданы bends_flat и flat_shape, ветка "physical" запускается
    в первую очередь. Никаких хардкодов под топологию детали.

Публичный API:
    calculate_auto_sequence(...) -> List[str]
    calculate_auto_sequence_ex(...) -> dict
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}

# --- backward всегда доступен ---
from . import backward as _backward
from . import scoring as _scoring

# --- forward опционален ---
try:
    from . import solver as _solver
    from . import topology as _topology
    _FORWARD_AVAILABLE = True
except ImportError:
    _solver = None
    _topology = None
    _FORWARD_AVAILABLE = False

# --- physical опционален ---
try:
    from . import physical_solver as _physical
    _PHYSICAL_AVAILABLE = True
except ImportError:
    _physical = None
    _PHYSICAL_AVAILABLE = False


BendKey = Tuple[int, int]


# =====================================================================
# Оценка покрытия
# =====================================================================

def _coverage_score(order_steps, all_bends) -> float:
    """Доля выполненных гибов (0..1)."""
    if not all_bends:
        return 0.0
    return len(order_steps) / float(len(all_bends))


# =====================================================================
# Physical-ветка — единственная физически корректная
# =====================================================================

def _run_physical(bends_flat,
                  flat_shape,
                  punch_adapter,
                  die_adapter,
                  press_shapes,
                  gauge_adapter,
                  time_budget_sec: float = 120.0,
                  logger=None,
                  on_progress=None) -> Optional[Dict[str, Any]]:
    """DFS с backtracking поверх sequence_validator.

    Args:
        bends_flat: List[BendSpec] в СИСТЕМЕ ПЛОСКОГО ЛИСТА. Перевод
                    из Body-системы делает task_panel (_apply_flat_coords).
        flat_shape: Part.Shape плоской развёртки.
        punch_adapter, die_adapter: ToolAdapter.
        press_shapes: list[Part.Shape] или None.
        gauge_adapter: ToolAdapter или None.
        time_budget_sec: жёсткий лимит DFS.
        logger, on_progress: callbacks.

    Returns:
        dict | None
    """
    _log = logger if callable(logger) else (lambda m: None)

    if not _PHYSICAL_AVAILABLE:
        _log("[AutoSeq] physical_solver недоступен")
        return None
    if not bends_flat:
        _log("[AutoSeq] bends_flat пуст — physical пропущен")
        return None
    if flat_shape is None:
        _log("[AutoSeq] flat_shape=None — physical пропущен")
        return None

    try:
        from core.sequence_validator import SequenceValidator
        from core.bend_kinematics import BendKinematics
        from .kinematics import ReverseKinematicsEngine
    except ImportError as e:
        _log("[AutoSeq] physical: импорт не удался: {}".format(e))
        return None

    validator = SequenceValidator(
        punch_adapter=punch_adapter,
        die_adapter=die_adapter,
        press_shapes=press_shapes or [],
        backgauge_adapter=gauge_adapter,
        logger=logger,
    )
    try:
        validator._flat_ref = flat_shape
    except Exception:
        pass

    try:
        kinematics = BendKinematics.from_config()
    except Exception:
        kinematics = None

    engine = ReverseKinematicsEngine(
        validator=validator,
        kinematics=kinematics,
        logger=logger,
    )

    sol = _physical.solve_physical(
        bends=list(bends_flat),
        flat_shape=flat_shape,
        engine=engine,
        logger=logger,
        time_budget_sec=time_budget_sec,
        on_progress=on_progress,
    )

    if not sol.order:
        _log("[AutoSeq] physical: пустой результат")
        return None

    return {
        "source": "physical_dfs",
        "order_ids": list(sol.order),
        "partial": bool(sol.partial),
        "stats": dict(sol.stats),
    }


# =====================================================================
# Backward-ветка (legacy)
# =====================================================================

def _run_backward(panel_graph_2d,
                  bend_info,
                  thickness_mm,
                  exclude_pairs,
                  strategy: str,
                  logger=None) -> Optional[Dict[str, Any]]:
    """Прогнать backward-поиск. Возвращает dict или None."""
    _log = logger if callable(logger) else (lambda m: None)

    if panel_graph_2d is None:
        return None

    try:
        from core.sequence_state import (
            FrozensetSequenceState, is_free_aabb, adjacent_pairs,
        )
    except ImportError as e:
        _log("[AutoSeq] backward: core.sequence_state недоступен: "
             "{}".format(e))
        return None

    ss = FrozensetSequenceState(panel_graph_2d, bend_info, thickness_mm)

    if exclude_pairs is None:
        exclude_pairs = adjacent_pairs(panel_graph_2d)

    def is_free_fn(state):
        return is_free_aabb(ss, state, exclude_pairs=exclude_pairs)

    if strategy == "backward_greedy":
        res = _backward.search_greedy(ss, is_free_fn)
    else:
        res = _backward.search_astar(ss, is_free_fn)

    if not res.success:
        _log("[AutoSeq] backward {}: fail ({}), visited={}".format(
            res.source, res.reason, res.states_visited))
        return None

    verify = _backward.verify_sequence(ss, res.order_forward, is_free_fn)
    return {
        "source": res.source,
        "ss": ss,
        "order_forward": res.order_forward,
        "search_result": res,
        "verify": verify,
    }


# =====================================================================
# Forward-ветка (legacy)
# =====================================================================

def _run_forward(unfold_obj,
                 adapter,
                 flat_shape,
                 punch_adapter,
                 die_adapter,
                 press_shapes,
                 gauge_adapter,
                 strategy: str,
                 max_iterations: int,
                 max_seconds: Optional[float],
                 allow_partial: bool,
                 logger=None,
                 cancel_check=None,
                 progress_callback=None) -> Optional[Dict[str, Any]]:
    """Прогнать forward-поиск. Возвращает dict или None."""
    _log = logger if callable(logger) else (lambda m: None)

    if not _FORWARD_AVAILABLE:
        _log("[AutoSeq] forward-модули не установлены")
        return None

    if adapter is None:
        _log("[AutoSeq] forward: adapter=None")
        return None

    try:
        from core.sequence_validator import SequenceValidator
        from core.bend_kinematics import BendKinematics
        from .kinematics import ReverseKinematicsEngine
    except ImportError as e:
        _log("[AutoSeq] forward: импорт SM-модулей не удался: "
             "{}".format(e))
        return None

    bends = _topology.collect_bend_specs(adapter, logger=logger)
    if not bends:
        _log("[AutoSeq] forward: гибы не найдены")
        return None

    bends_sorted = _topology.topological_order(bends)

    validator = SequenceValidator(
        punch_adapter=punch_adapter,
        die_adapter=die_adapter,
        press_shapes=press_shapes or [],
        backgauge_adapter=gauge_adapter,
        logger=logger,
    )
    try:
        validator._flat_ref = flat_shape
    except Exception:
        pass

    try:
        kinematics = BendKinematics.from_config()
    except Exception:
        kinematics = None

    engine = ReverseKinematicsEngine(
        validator=validator,
        kinematics=kinematics,
        logger=logger,
    )

    greedy_restarts = int(SEQUENCE_CONFIG.get("auto_greedy_restarts", 1))
    seq = _solver.greedy_sequence(
        bends_sorted, flat_shape, engine,
        allow_partial=allow_partial, logger=logger,
    )
    if (not allow_partial or len(seq) < len(bends_sorted)) \
            and greedy_restarts > 0:
        pass

    if strategy == "forward_astar" or \
            (strategy == "hybrid" and len(seq) < len(bends_sorted)):
        astar_weight = float(SEQUENCE_CONFIG.get("auto_astar_weight", 2.0))
        seq_astar = _solver.astar_sequence(
            bends=bends_sorted,
            flat_shape=flat_shape,
            engine=engine,
            max_iterations=max_iterations,
            max_seconds=max_seconds,
            allow_partial=allow_partial,
            logger=logger,
            cancel_check=cancel_check,
            progress_callback=progress_callback,
            heuristic_weight=astar_weight,
        )
        if len(seq_astar) > len(seq):
            seq = seq_astar

    return {
        "source": "forward_" + strategy.replace("forward_", ""),
        "order_ids": seq,
        "all_bends": bends_sorted,
    }


# =====================================================================
# Публичный API
# =====================================================================

def calculate_auto_sequence(
    unfold_obj=None,
    adapter=None,
    flat_shape=None,
    punch_adapter=None,
    die_adapter=None,
    press_shapes=None,
    gauge_adapter=None,
    # --- physical-режим (СИСТЕМА ПЛОСКОГО ЛИСТА!) ---
    bends_flat=None,
    time_budget_sec: float = 120.0,
    # --- backward-режим ---
    panel_graph_2d=None,
    panel_graph=None,
    bend_info=None,
    thickness_mm=None,
    exclude_pairs=None,
    # --- управление ---
    strategy: str = "hybrid",
    max_iterations: int = 2000,
    max_seconds: Optional[float] = None,
    allow_partial: bool = True,
    logger=None,
    cancel_check=None,
    progress_callback=None,
) -> List[str]:
    """Совместимый API: вернуть список bend_id.

    Стратегии:
        "physical"        -> только physical_dfs
        "forward_greedy"  -> legacy forward greedy
        "forward_astar"   -> legacy forward A*
        "backward_greedy" -> legacy backward greedy
        "backward_astar"  -> legacy backward A*
        "hybrid" (default) -> physical (если bends_flat), иначе
                             forward + backward
    """
    result = calculate_auto_sequence_ex(
        unfold_obj=unfold_obj,
        adapter=adapter,
        flat_shape=flat_shape,
        punch_adapter=punch_adapter,
        die_adapter=die_adapter,
        press_shapes=press_shapes,
        gauge_adapter=gauge_adapter,
        bends_flat=bends_flat,
        time_budget_sec=time_budget_sec,
        panel_graph_2d=panel_graph_2d,
        panel_graph=panel_graph,
        bend_info=bend_info,
        thickness_mm=thickness_mm,
        exclude_pairs=exclude_pairs,
        strategy=strategy,
        max_iterations=max_iterations,
        max_seconds=max_seconds,
        allow_partial=allow_partial,
        logger=logger,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )
    return result.get("order_ids", [])


def calculate_auto_sequence_ex(
    unfold_obj=None,
    adapter=None,
    flat_shape=None,
    punch_adapter=None,
    die_adapter=None,
    press_shapes=None,
    gauge_adapter=None,
    # --- physical ---
    bends_flat=None,
    time_budget_sec: float = 120.0,
    # --- backward ---
    panel_graph_2d=None,
    panel_graph=None,
    bend_info=None,
    thickness_mm=None,
    exclude_pairs=None,
    # --- управление ---
    strategy: str = "hybrid",
    max_iterations: int = 2000,
    max_seconds: Optional[float] = None,
    allow_partial: bool = True,
    logger=None,
    cancel_check=None,
    progress_callback=None,
) -> Dict[str, Any]:
    """Расширенный API с диагностикой.

    Returns:
        {
          "ok": bool,
          "order_ids": List[str],
          "order_face_pairs": List[(a, b)],
          "order_step_keys": List[(feat, idx)],
          "source": str,
          "score": dict | None,
          "verify": dict | None,
          "attempts": [ {source, ok, n_steps, reason}, ... ],
          "notes": [str],
          "stats": dict | None,
          "partial": bool,
        }
    """
    _log = logger if callable(logger) else (lambda m: None)
    _log("[AutoSeq] strategy={!r}".format(strategy))

    attempts: List[Dict[str, Any]] = []
    candidates: List[Dict[str, Any]] = []

    # --- Нормализация алиасов ---
    st = strategy.lower()
    if st in ("greedy", "forward", "auto"):
        st = "forward_greedy"
    if st in ("astar",):
        st = "forward_astar"
    if st in ("backward",):
        st = "backward_astar"

    # ============================================================
    # PHYSICAL — единственная физически корректная ветка.
    # ============================================================
    physical_wanted = (st in ("physical", "hybrid",
                              "forward_greedy", "forward_astar"))
    if physical_wanted and bends_flat and flat_shape is not None:
        phys = _run_physical(
            bends_flat=bends_flat,
            flat_shape=flat_shape,
            punch_adapter=punch_adapter,
            die_adapter=die_adapter,
            press_shapes=press_shapes,
            gauge_adapter=gauge_adapter,
            time_budget_sec=time_budget_sec,
            logger=logger,
            on_progress=progress_callback,
        )
        if phys and phys.get("order_ids"):
            attempts.append({
                "source": phys["source"],
                "ok": True,
                "n_steps": len(phys["order_ids"]),
                "reason": ("partial" if phys.get("partial") else "ok"),
            })
            candidates.append(phys)
            if st == "physical":
                return _finalize(
                    best=phys,
                    order_fp=[],
                    attempts=attempts,
                    adapter=adapter,
                    panel_graph=panel_graph,
                    panel_graph_2d=panel_graph_2d,
                    logger=logger,
                )
        else:
            attempts.append({
                "source": "physical_dfs",
                "ok": False,
                "n_steps": 0,
                "reason": "physical_failed",
            })

    # ============================================================
    # LEGACY FORWARD (только если physical недоступен)
    # ============================================================
    forward_wanted = st in ("forward_greedy", "forward_astar", "hybrid")
    if forward_wanted and not candidates:
        fwd = _run_forward(
            unfold_obj=unfold_obj,
            adapter=adapter,
            flat_shape=flat_shape,
            punch_adapter=punch_adapter,
            die_adapter=die_adapter,
            press_shapes=press_shapes,
            gauge_adapter=gauge_adapter,
            strategy=("forward_astar"
                      if st == "forward_astar" else "forward_greedy"),
            max_iterations=max_iterations,
            max_seconds=max_seconds,
            allow_partial=allow_partial,
            logger=logger,
            cancel_check=cancel_check,
            progress_callback=progress_callback,
        )
        if fwd and fwd.get("order_ids"):
            attempts.append({
                "source": fwd["source"],
                "ok": True,
                "n_steps": len(fwd["order_ids"]),
                "reason": "ok",
            })
            candidates.append(fwd)
        else:
            attempts.append({
                "source": "forward",
                "ok": False,
                "n_steps": 0,
                "reason": "forward_unavailable_or_failed",
            })

    # ============================================================
    # LEGACY BACKWARD (только если physical и forward не дали)
    # ============================================================
    backward_wanted = st in ("backward_greedy", "backward_astar", "hybrid")
    if backward_wanted and not candidates:
        bwd_strategy = "backward_astar"
        if st == "backward_greedy":
            bwd_strategy = "backward_greedy"
        elif st == "hybrid":
            bwd_strategy = "backward_greedy"

        bwd = _run_backward(
            panel_graph_2d=panel_graph_2d,
            bend_info=bend_info or {},
            thickness_mm=thickness_mm,
            exclude_pairs=exclude_pairs,
            strategy=bwd_strategy,
            logger=logger,
        )

        if bwd and st == "hybrid":
            verify = bwd.get("verify", {})
            if not verify.get("valid"):
                _log("[AutoSeq] hybrid: backward_greedy partial, "
                     "пробую backward_astar")
                bwd2 = _run_backward(
                    panel_graph_2d=panel_graph_2d,
                    bend_info=bend_info or {},
                    thickness_mm=thickness_mm,
                    exclude_pairs=exclude_pairs,
                    strategy="backward_astar",
                    logger=logger,
                )
                if bwd2:
                    bwd = bwd2

        if bwd:
            order_fp = bwd["order_forward"]
            attempts.append({
                "source": bwd["source"],
                "ok": True,
                "n_steps": len(order_fp),
                "reason": "ok",
            })
            candidates.append({
                "source": bwd["source"],
                "order_face_pairs": order_fp,
                "ss": bwd["ss"],
                "verify": bwd.get("verify"),
            })
        else:
            attempts.append({
                "source": "backward",
                "ok": False,
                "n_steps": 0,
                "reason": "backward_unavailable_or_failed",
            })

    # ============================================================
    # Ранжирование кандидатов
    # ============================================================
    if not candidates:
        _log("[AutoSeq] ни одна стратегия не дала результата")
        return {
            "ok": False,
            "order_ids": [],
            "order_face_pairs": [],
            "order_step_keys": [],
            "source": None,
            "score": None,
            "verify": None,
            "attempts": attempts,
            "notes": ["no_candidates"],
            "stats": None,
            "partial": True,
        }

    best = None
    best_key = None
    for c in candidates:
        if "order_face_pairs" in c:
            n = len(c["order_face_pairs"])
        else:
            n = len(c.get("order_ids") or [])

        is_physical = (c.get("source") == "physical_dfs")
        partial_penalty = 1 if c.get("partial") else 0

        scoring_total = None
        if c.get("ss") is not None:
            try:
                sc = _scoring.score_sequence(
                    c["order_face_pairs"], c["ss"])
                scoring_total = sc["total"]
                c["score"] = sc
            except Exception as e:
                _log("[AutoSeq] scoring error: {}".format(e))

        key = (
            0 if is_physical else 1,
            partial_penalty,
            -n,
            scoring_total if scoring_total is not None else 0.0,
        )
        if best is None or key < best_key:
            best = c
            best_key = key

    if best is None:
        return {
            "ok": False,
            "order_ids": [],
            "order_face_pairs": [],
            "order_step_keys": [],
            "source": None,
            "score": None,
            "verify": None,
            "attempts": attempts,
            "notes": ["best_is_none"],
            "stats": None,
            "partial": True,
        }

    # ============================================================
    # Трансляция в step_keys / ids
    # ============================================================
    order_fp = best.get("order_face_pairs") or []
    order_sk: List[Tuple[str, int]] = []
    order_ids: List[str] = []
    notes: List[str] = []

    if best.get("source") == "physical_dfs":
        order_ids = list(best.get("order_ids") or [])
    elif panel_graph is not None and panel_graph_2d is not None:
        try:
            from core.sequence_state import (
                build_face_pair_to_step_key_mapping, order_to_step_keys,
            )
            mapping = build_face_pair_to_step_key_mapping(
                panel_graph, panel_graph_2d)
            order_sk = order_to_step_keys(order_fp, mapping)

            step_to_bend_id = {}
            try:
                all_bends = adapter.get_bend_specs() if adapter else []
            except Exception:
                all_bends = []
            feat_to_ids = OrderedDict()
            for b in all_bends:
                meta = getattr(b, "metadata", {}) or {}
                feat = str(meta.get("feature") or "")
                if feat:
                    feat_to_ids.setdefault(feat, []).append(b.id)
            for feat, ids in feat_to_ids.items():
                for k, bid in enumerate(ids):
                    step_to_bend_id[(feat, k)] = bid

            for (feat_id, li) in order_sk:
                bid = step_to_bend_id.get((feat_id, li))
                if bid is None:
                    bid = feat_id if li == 0 else "{}_{}".format(feat_id, li)
                    notes.append(
                        "no bend_id for step_key ({}, {}); "
                        "fallback {}".format(feat_id, li, bid))
                order_ids.append(bid)

            if len(order_sk) < len(order_fp):
                notes.append(
                    "translation partial: {}/{} step_keys "
                    "resolved".format(len(order_sk), len(order_fp)))
        except Exception as e:
            notes.append("translation failed: {}".format(e))
            _log("[AutoSeq] трансляция в step_keys failed: {}".format(e))
    else:
        notes.append(
            "panel_graph / panel_graph_2d not provided — "
            "order_ids not generated.")

    if not order_ids and best.get("order_ids"):
        order_ids = list(best["order_ids"])

    result: Dict[str, Any] = {
        "ok": bool(order_ids or order_fp),
        "order_ids": order_ids,
        "order_face_pairs": [list(k) for k in order_fp],
        "order_step_keys": [list(k) for k in order_sk],
        "source": best.get("source"),
        "score": best.get("score"),
        "verify": best.get("verify"),
        "attempts": attempts,
        "notes": notes,
        "stats": best.get("stats"),
        "partial": bool(best.get("partial", False)),
    }

    _log("[AutoSeq] источник: {} ({} шагов){}".format(
        result["source"],
        len(order_ids) or len(order_fp),
        " [partial]" if result["partial"] else ""))

    return result


# =====================================================================
# Хелпер финализации
# =====================================================================

def _finalize(best, order_fp, attempts, adapter, panel_graph,
              panel_graph_2d, logger=None) -> Dict[str, Any]:
    """Собрать результат, когда strategy=physical и он уже найден."""
    return {
        "ok": bool(best.get("order_ids")),
        "order_ids": list(best.get("order_ids") or []),
        "order_face_pairs": [list(k) for k in (order_fp or [])],
        "order_step_keys": [],
        "source": best.get("source"),
        "score": None,
        "verify": None,
        "attempts": attempts,
        "notes": [],
        "stats": best.get("stats"),
        "partial": bool(best.get("partial", False)),
    }