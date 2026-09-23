# -*- coding: utf-8 -*-
"""
autosequencer/backward.py
Backward search последовательности гибки на frozenset-состояниях.

Перенесено из BendSpec (core/sequence_search.py).
Работает поверх FrozensetSequenceState (core/sequence_state.py).

Идея: идём от полного состояния (full) к пустому (empty), на каждом
шаге отгибая листовой гиб. Каждый undo обязан давать collision-free
состояние (жёсткое ограничение).

Публичный API:
    search_greedy(ss, is_free_fn, ...)     -> SearchResult
    search_astar(ss, is_free_fn, ...)      -> SearchResult
    search_sequence(ss, is_free_fn, strategy=...) -> SearchResult
    verify_sequence(ss, order_forward, is_free_fn) -> dict

Не импортирует FreeCAD. Полностью тестируем на моках.

Версия: BENDBEQ_BACKWARD_V1
"""

from __future__ import annotations

import time
from heapq import heappush, heappop
from typing import Callable, Dict, FrozenSet, List, Optional, Tuple

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


BendKey = Tuple[int, int]
State = FrozenSet[BendKey]


# =====================================================================
# Результат
# =====================================================================

class SearchResult(object):
    """Результат backward-поиска.

    Attributes:
        success:        True, если дошли до empty без коллизий.
        source:         "backward_greedy" | "backward_astar".
        order_backward: [(a, b), ...] — порядок разгиба (full -> empty).
        order_forward:  [(a, b), ...] — порядок гибки (empty -> full).
        states_visited: число состояний, поставленных в очередь.
        collision_checks: сколько раз вызывали is_free_fn.
        time_ms:        время работы, мс.
        reason:         "success" | "initial_collision" | "no_undoable"
                        | "all_undoable_collide" | "max_steps"
                        | "max_nodes" | "exhausted".
        trace:          пошаговая трасса для отладки.
    """

    def __init__(self):
        self.success = False
        self.source = "backward"
        self.order_backward: List[BendKey] = []
        self.order_forward: List[BendKey] = []
        self.states_visited = 0
        self.collision_checks = 0
        self.time_ms = 0.0
        self.reason = ""
        self.trace: List[dict] = []

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "source": self.source,
            "order_backward": [list(k) for k in self.order_backward],
            "order_forward": [list(k) for k in self.order_forward],
            "states_visited": self.states_visited,
            "collision_checks": self.collision_checks,
            "time_ms": round(self.time_ms, 3),
            "reason": self.reason,
        }

    def __repr__(self):
        return ("SearchResult(source={}, success={}, steps={}, "
                "visited={}, checks={}, reason={!r})").format(
            self.source, self.success, len(self.order_forward),
            self.states_visited, self.collision_checks, self.reason)


# =====================================================================
# Heuristic по умолчанию
# =====================================================================

def _depth_of_bend(ss, bend_key: BendKey) -> int:
    """Глубина ребёнка этого bend в дереве от anchor."""
    child = ss._child_side(bend_key)
    if child is None:
        return -1
    parent_of = ss._build_tree()
    d = 0
    n = child
    while parent_of.get(n) is not None:
        d += 1
        n = parent_of[n]
    return d


def default_heuristic(ss):
    """Backward: сначала undo самые глубокие листья.

    Возвращает меньшее число = выше приоритет.
    Tie-break по каноническому (min, max) ключу — детерминизм.
    """
    def _score(state, bend_key):
        d = _depth_of_bend(ss, bend_key)
        a, b = bend_key
        ka = (min(a, b), max(a, b))
        return (-d, ka)
    return _score


# =====================================================================
# Greedy backward
# =====================================================================

def search_greedy(ss,
                  is_free_fn: Callable[[State], bool],
                  heuristic_fn: Optional[Callable] = None,
                  max_steps: Optional[int] = None) -> SearchResult:
    """Greedy backward: full -> empty.

    На каждом шаге:
      1. undoable(state) — hard constraint (листовой гиб).
      2. Отфильтровать по is_free_fn(undo(state)).
      3. Выбрать лучший по heuristic_fn.
      4. Если ни один не проходит — fail.

    Args:
        ss:          FrozensetSequenceState
        is_free_fn:  state -> bool
        heuristic_fn: (state, bend_key) -> sortable; None -> default
        max_steps:   лимит итераций; None -> из config

    Returns:
        SearchResult.
    """
    if max_steps is None:
        max_steps = int(SEQUENCE_CONFIG.get(
            "auto_backward_max_steps", 10000))

    result = SearchResult()
    result.source = "backward_greedy"
    t0 = time.time()

    if heuristic_fn is None:
        heuristic_fn = default_heuristic(ss)

    state = ss.full_state()
    result.collision_checks += 1
    if not is_free_fn(state):
        result.reason = "initial_collision"
        result.time_ms = (time.time() - t0) * 1000.0
        return result
    result.states_visited = 1
    result.trace.append({"state_size": len(state), "action": "start"})

    while state:
        if result.states_visited > max_steps:
            result.reason = "max_steps"
            result.time_ms = (time.time() - t0) * 1000.0
            return result

        u = ss.undoable(state)
        if not u:
            result.reason = "no_undoable"
            result.time_ms = (time.time() - t0) * 1000.0
            return result

        u_sorted = sorted(u, key=lambda k: heuristic_fn(state, k))
        chosen = None
        for b in u_sorted:
            new_state = ss.undo(state, b)
            result.collision_checks += 1
            if is_free_fn(new_state):
                chosen = (b, new_state)
                break
        if chosen is None:
            result.reason = "all_undoable_collide"
            result.time_ms = (time.time() - t0) * 1000.0
            return result

        b, state = chosen
        result.order_backward.append(b)
        result.states_visited += 1
        result.trace.append({
            "state_size": len(state),
            "action": "undo",
            "bend": list(b),
        })

    if not state:
        result.success = True
        result.reason = "success"
        result.order_forward = list(reversed(result.order_backward))
    result.time_ms = (time.time() - t0) * 1000.0
    return result


# =====================================================================
# A* backward
# =====================================================================

def _state_key(state: State) -> State:
    """Канонический ключ состояния."""
    return frozenset((min(a, b), max(a, b)) for (a, b) in state)


def search_astar(ss,
                 is_free_fn: Callable[[State], bool],
                 heuristic_fn: Optional[Callable] = None,
                 max_nodes: Optional[int] = None,
                 fail_early: bool = True) -> SearchResult:
    """A* backward. h(state) = |state| (число оставшихся гибов).

    g = число шагов (уже выполненных undo).
    f = g + h — admissible и consistent.

    Гарантирует последовательность с минимальным числом шагов (при
    корректном is_free_fn).

    Args:
        ss:          FrozensetSequenceState
        is_free_fn:  state -> bool
        heuristic_fn: не используется (для совместимости API)
        max_nodes:   лимит раскрытых узлов; None -> из config
        fail_early:  не используется (A* всегда fail_early)

    Returns:
        SearchResult.
    """
    if max_nodes is None:
        max_nodes = int(SEQUENCE_CONFIG.get(
            "auto_backward_max_nodes", 20000))

    result = SearchResult()
    result.source = "backward_astar"
    t0 = time.time()

    start = ss.full_state()
    result.collision_checks += 1
    if not is_free_fn(start):
        result.reason = "initial_collision"
        result.time_ms = (time.time() - t0) * 1000.0
        return result

    counter = 0
    open_heap: list = []
    # (f, tie, g, state, path)
    heappush(open_heap, (0.0, counter, 0, start, []))
    visited_keys: set = set()
    nodes = 0

    while open_heap:
        if nodes > max_nodes:
            result.reason = "max_nodes"
            result.time_ms = (time.time() - t0) * 1000.0
            return result

        _f, _tie, g, state, path = heappop(open_heap)
        nodes += 1
        result.states_visited += 1

        key = _state_key(state)
        if key in visited_keys:
            continue
        visited_keys.add(key)

        if not state:
            result.success = True
            result.reason = "success"
            result.order_backward = list(path)
            result.order_forward = list(reversed(path))
            result.time_ms = (time.time() - t0) * 1000.0
            return result

        u = ss.undoable(state)
        if not u:
            continue

        for b in u:
            new_state = ss.undo(state, b)
            result.collision_checks += 1
            if not is_free_fn(new_state):
                continue
            new_key = _state_key(new_state)
            if new_key in visited_keys:
                continue
            new_g = g + 1
            new_path = path + [b]
            new_f = new_g + len(new_state)   # h = |state|
            counter += 1
            heappush(open_heap,
                     (new_f, counter, new_g, new_state, new_path))

    result.reason = "exhausted"
    result.time_ms = (time.time() - t0) * 1000.0
    return result


# =====================================================================
# Универсальный entry point
# =====================================================================

def search_sequence(ss,
                    is_free_fn: Callable[[State], bool],
                    strategy: str = "backward_astar",
                    heuristic_fn: Optional[Callable] = None,
                    max_steps: Optional[int] = None,
                    max_nodes: Optional[int] = None) -> SearchResult:
    """Универсальный entry point.

    strategy: 'backward_greedy' | 'backward_astar'
              алиасы 'greedy' / 'astar' тоже принимаются.
    """
    if strategy in ("backward_greedy", "greedy"):
        return search_greedy(ss, is_free_fn,
                             heuristic_fn=heuristic_fn,
                             max_steps=max_steps)
    if strategy in ("backward_astar", "astar"):
        return search_astar(ss, is_free_fn,
                            heuristic_fn=heuristic_fn,
                            max_nodes=max_nodes)
    raise ValueError("unknown backward strategy: {!r}".format(strategy))


# =====================================================================
# Verify: пройти по order_forward и проверить каждый шаг
# =====================================================================

def verify_sequence(ss,
                    order_forward: List[BendKey],
                    is_free_fn: Callable[[State], bool]) -> dict:
    """Проверить, что order_forward приводит от empty к full без коллизий.

    Returns:
        {
          "valid": bool,
          "reason": str,
          "states": [{step, applied, done, free}, ...],
        }
    """
    state = ss.empty_state()
    out = {"valid": True, "reason": "ok", "states": []}

    if not is_free_fn(state):
        out["valid"] = False
        out["reason"] = "initial_empty_not_free"
        return out

    for step, b in enumerate(order_forward):
        fold = ss.foldable(state)
        if b not in fold and (b[1], b[0]) not in fold:
            out["valid"] = False
            out["reason"] = "step {}: {} not foldable from {}".format(
                step, b, sorted(state))
            return out
        state = ss.apply(state, b)
        free = is_free_fn(state)
        out["states"].append({
            "step": step,
            "applied": list(b),
            "done": len(state),
            "free": free,
        })
        if not free:
            out["valid"] = False
            out["reason"] = "step {}: collision after apply {}".format(
                step, b)
            return out

    if len(state) != len(ss.bend_ids):
        out["valid"] = False
        out["reason"] = "final state incomplete: {}/{}".format(
            len(state), len(ss.bend_ids))
    return out