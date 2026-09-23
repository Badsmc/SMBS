# -*- coding: utf-8 -*-
"""
autosequencer/topology.py
Сбор BendSpec у адаптера + топологический порядок гибки.

Версия: BENDBEQ_TOPOLOGY_V1
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional, Set, Tuple


def collect_bend_specs(adapter, logger=None) -> list:
    """Собрать BendSpec у адаптера с логированием."""
    _log = logger if callable(logger) else (lambda m: None)
    try:
        specs = adapter.get_bend_specs()
    except Exception as e:
        _log("[Topology] Ошибка get_bend_specs: {}".format(e))
        return []
    _log("[Topology] Получено {} BendSpec.".format(len(specs)))
    for b in specs:
        _log("[Topology]   {}: angle={:.1f}° dir={} len={:.1f}".format(
            b.id, b.angle, b.direction, b.length))
    return list(specs)


def _sub_index(sub_name: str) -> int:
    if not sub_name:
        return 0
    m = re.search(r"(\d+)$", str(sub_name))
    return int(m.group(1)) if m else 0


def feature_dependency_graph(bends):
    """Граф зависимостей фич: {feat: set(parent_feat)}."""
    known_features: Set[str] = set()
    feature_of_bend: Dict[str, str] = {}
    for b in bends:
        meta = getattr(b, "metadata", {}) or {}
        feat = str(meta.get("feature") or "").strip()
        if feat:
            known_features.add(feat)
            feature_of_bend[b.id] = feat

    graph: Dict[str, Set[str]] = {feat: set() for feat in known_features}
    for b in bends:
        meta = getattr(b, "metadata", {}) or {}
        feat = str(meta.get("feature") or "").strip()
        parent = str(meta.get("parent_feature") or "").strip()
        if not feat or not parent:
            continue
        if parent in known_features and parent != feat:
            graph[feat].add(parent)
    return graph, feature_of_bend


def topological_order(bends) -> list:
    """Отсортировать BendSpec: сначала базовые фичи, потом производные."""
    graph, _ = feature_dependency_graph(bends)
    depth_cache: Dict[str, int] = {}

    def depth_of(feat, visiting=None):
        if feat in depth_cache:
            return depth_cache[feat]
        if visiting is None:
            visiting = set()
        if feat in visiting:
            return 0
        visiting.add(feat)
        parents = graph.get(feat, set())
        d = (0 if not parents
             else 1 + max(depth_of(p, visiting) for p in parents))
        visiting.discard(feat)
        depth_cache[feat] = d
        return d

    def key(b):
        meta = getattr(b, "metadata", {}) or {}
        feat = str(meta.get("feature") or "").strip()
        sub = meta.get("sub_element") or ""
        bid = getattr(b, "id", "") or ""
        if not feat:
            return (10 ** 6, "", 0, bid)
        return (depth_of(feat), feat, _sub_index(sub), bid)

    return sorted(bends, key=key)