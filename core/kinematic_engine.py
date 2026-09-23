# -*- coding: utf-8 -*-
"""
core/kinematic_engine.py
KinematicEngine v4.0 (per-line steps) + support for fractions.

Изменения относительно SM v4.0:
  * compute_step_placements / build_step_compound принимают
    fractions: dict[(feat_id, line_idx) -> 0..1]. Если задано —
    активный шаг поворачивается на fraction * angle.
    Если fractions=None — поведение как в SM v4.0 (moment_factor).
  * panel_face_index(pid) -> face_index панели (для трансляции).
  * step_key_to_panel_id(feat_id, line_idx) -> node_id ребёнка.

Сохранено из v4.0:
  * Публичный API работает с парами (feat_id, line_idx) —
    multi-edge фичи (Bend004, Bend005) дают 10 шагов вместо 6.
  * Двухуровневый резолвер знаков: CoM delta + post-check BBox.
  * Pivot с фиксированным центром.
  * fuse в build_step_compound.

Версия: BENDBEQ_KINEMATIC_V5_0_FRACTIONS
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector, Rotation, Placement
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    Vector = None
    Rotation = None
    Placement = None
    _FC_OK = False

from core.panel_graph import PanelGraph, PanelNode


class KinematicEngineError(Exception):
    pass


StepKey = Tuple[str, int]
U4 = List[List[float]]


# =====================================================================
# KinematicEngine
# =====================================================================

class KinematicEngine:
    """Кинематический движок гибки по дереву PanelGraph.

    Публичный API работает с парами (feat_id, line_idx):
        - all_step_keys() -> список всех шагов в топологическом порядке.
        - compute_step_placements(active_keys, moment_factor, fractions)
          -> Dict[panel_id, Placement].
        - build_step_compound(active_keys, ...) -> Part.Shape.
        - state_at(step_idx, ordered_keys) -> Part.Shape.

    Знаки для каждой пары (feat_id, line_idx) разрешаются один раз
    при инициализации через:
        1. _resolve_signs_via_body — минимальная CoM-ошибка.
        2. _post_check_bbox — флип знака при выходе панели за BBox Body.
    """

    def __init__(self, panel_graph: PanelGraph, bend_features: List,
                 body=None):
        if not _FC_OK:
            raise KinematicEngineError(
                "KinematicEngine требует FreeCAD (Part.Vector)")

        self.graph = panel_graph
        self.features_map = {f.id: f for f in bend_features}
        self._resolved_signs: Dict[StepKey, float] = {}

        # Проверка связности: feat_id в дереве должны быть в features_map
        used = set()
        for n in self.graph.nodes.values():
            for (_c, feat_id, _li) in n.children:
                if feat_id:
                    used.add(feat_id)
        missing = used - set(self.features_map.keys())
        if missing:
            raise KinematicEngineError(
                "feat_id в дереве отсутствуют в features_map: "
                "{}".format(sorted(missing)))

        self.body_bbox = None
        if body is not None:
            try:
                self._resolve_signs_via_body(body)
                self.body_bbox = body.Shape.BoundBox
                self._post_check_bbox()
            except Exception as e:
                try:
                    App.Console.PrintWarning(
                        "[Kinematic] sign resolution failed: {}\n".format(e))
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # Sign resolution
    # ------------------------------------------------------------------

    def _resolve_signs_via_body(self, body) -> None:
        """Разрешить знаки для каждой пары (feat_id, line_idx).

        Для каждого шага пробуем sign=+1 и sign=-1, сравниваем CoM
        child-панели с target_com из BendLine. Выбираем знак с
        минимальной ошибкой.
        """
        order = self._feature_topo_order()

        targets: Dict[StepKey, Vector] = {}
        for fid in order:
            feat = self.features_map.get(fid)
            if feat is None:
                continue
            for li, ln in enumerate(feat.lines):
                tc = getattr(ln, "target_com", None)
                if tc is not None:
                    targets[(fid, li)] = Vector(tc)

        if not targets:
            try:
                App.Console.PrintWarning("[Kinematic] нет target_com\n")
            except Exception:
                pass
            return

        resolved: Dict[StepKey, float] = {}

        for fid in order:
            feat = self.features_map.get(fid)
            if feat is None:
                continue

            for li in range(len(feat.lines)):
                key = (fid, li)
                target = targets.get(key)
                if target is None:
                    resolved[key] = 1.0
                    continue

                best_sign = 1.0
                best_err = float("inf")

                for sign in (+1.0, -1.0):
                    trial = dict(resolved)
                    trial[key] = sign
                    sim_com = self._child_panel_com_world(
                        fid, li, trial, order
                    )
                    if sim_com is None:
                        continue
                    err = (sim_com - target).Length
                    if err < best_err:
                        best_err = err
                        best_sign = sign

                resolved[key] = best_sign
                try:
                    App.Console.PrintMessage(
                        "[Kinematic] {}[{}]: sign={:+.0f} "
                        "err={:.3f}mm target=({:.2f},{:.2f},{:.2f})\n"
                        .format(fid, li, best_sign, best_err,
                                target.x, target.y, target.z))
                except Exception:
                    pass

        self._resolved_signs = resolved
        try:
            App.Console.PrintMessage(
                "[Kinematic] разрешено знаков: {}\n".format(len(resolved)))
        except Exception:
            pass

    def _post_check_bbox(self) -> None:
        """Итеративно флипать знаки при выходе панели за BBox Body."""
        if self.body_bbox is None:
            return

        tol = 5.0
        b = self.body_bbox
        max_iter = 5

        for it in range(max_iter):
            ordered_keys = self._feature_keys_topo_order()
            placements = self.compute_step_placements(ordered_keys)

            flipped_any = False
            for pid, node in self.graph.nodes.items():
                if node.feature_id is None:
                    continue
                pl = placements.get(pid)
                if pl is None:
                    continue
                shape = node.shape.copy()
                shape.Placement = pl.multiply(shape.Placement)
                bb = shape.BoundBox

                out_x = (bb.XMax > b.XMax + tol
                         or bb.XMin < b.XMin - tol)
                out_y = (bb.YMax > b.YMax + tol
                         or bb.YMin < b.YMin - tol)
                out_z = (bb.ZMax > b.ZMax + tol
                         or bb.ZMin < b.ZMin - tol)

                if not (out_x or out_y or out_z):
                    continue

                key = self._panel_key(pid)
                if key is None:
                    continue

                old = self._resolved_signs.get(key, 1.0)
                new = -old
                self._resolved_signs[key] = new
                flipped_any = True
                try:
                    App.Console.PrintMessage(
                        "[Kinematic] post-check iter {}: {} flip "
                        "{:+.0f} -> {:+.0f} (panel={}, "
                        "X[{:.2f},{:.2f}] Y[{:.2f},{:.2f}] "
                        "Z[{:.2f},{:.2f}])\n".format(
                            it, key, old, new, pid,
                            bb.XMin, bb.XMax,
                            bb.YMin, bb.YMax,
                            bb.ZMin, bb.ZMax))
                except Exception:
                    pass

            if not flipped_any:
                try:
                    App.Console.PrintMessage(
                        "[Kinematic] post-check: OK (iter={})\n".format(it))
                except Exception:
                    pass
                return

        try:
            App.Console.PrintWarning(
                "[Kinematic] post-check: не сошлось за {} итераций\n"
                .format(max_iter))
        except Exception:
            pass

    def _panel_key(self, pid: str) -> Optional[StepKey]:
        """Найти (feat_id, line_idx) для панели-ребёнка."""
        node = self.graph.nodes.get(pid)
        if node is None or node.parent_id is None:
            return None
        parent = self.graph.nodes.get(node.parent_id)
        if parent is None:
            return None
        for (cid, ffid, li) in parent.children:
            if cid == pid:
                if not ffid:
                    return None
                return (ffid, li)
        return None

    def _feature_topo_order(self) -> List[str]:
        """Топологический порядок feat_id (родитель раньше ребёнка)."""
        parent_of = {f.id: f.parent_feature
                     for f in self.features_map.values()}

        def depth(fid):
            d = 0
            cur = parent_of.get(fid, "")
            seen = set()
            while cur and cur in parent_of and cur not in seen:
                seen.add(cur)
                d += 1
                cur = parent_of.get(cur, "")
            return d

        return sorted(self.features_map.keys(),
                      key=lambda x: (depth(x), x))

    def _feature_keys_topo_order(self) -> List[StepKey]:
        """Все (feat_id, line_idx) в топологическом порядке фич."""
        result: List[StepKey] = []
        for fid in self._feature_topo_order():
            feat = self.features_map.get(fid)
            if feat is None:
                continue
            for li in range(len(feat.lines)):
                result.append((fid, li))
        return result

    def _child_panel_com_world(
        self, fid: str, line_idx: int,
        sign_map: Dict[StepKey, float],
        order: List[str],
    ):
        """CoM child-панели (fid, line_idx) при заданных знаках."""
        active_keys = set()
        for f in order:
            if f in self.features_map:
                n_lines = len(self.features_map[f].lines)
                for li in range(n_lines):
                    if (f, li) in sign_map:
                        active_keys.add((f, li))
            if f == fid:
                break

        placements: Dict[str, Placement] = {}
        root = self.graph.nodes[self.graph.root_id]
        placements[root.id] = Placement()
        self._propagate_with_signs(root, placements, active_keys, sign_map)

        for pid, node in self.graph.nodes.items():
            if node.feature_id != fid:
                continue
            if node.parent_id is None:
                continue
            parent = self.graph.nodes.get(node.parent_id)
            if parent is None:
                continue
            this_li = None
            for (cid, _ffid, li) in parent.children:
                if cid == pid:
                    this_li = li
                    break
            if this_li != line_idx:
                continue
            pl = placements.get(pid)
            if pl is None:
                continue
            com_local = self._safe_com(node.shape)
            if com_local is None:
                continue
            return pl.multVec(com_local)
        return None

    def _safe_com(self, shape):
        """CenterOfMass формы с несколькими fallback."""
        if shape is None or shape.isNull():
            return None
        try:
            return Vector(shape.CenterOfMass)
        except Exception:
            pass
        try:
            solids = list(shape.Solids)
            if solids:
                total_v = 0.0
                weighted = Vector(0, 0, 0)
                for s in solids:
                    if s is None or s.isNull():
                        continue
                    v = float(s.Volume)
                    if v < 1e-9:
                        continue
                    weighted = weighted + Vector(s.CenterOfMass) * v
                    total_v += v
                if total_v > 1e-9:
                    return weighted / total_v
        except Exception:
            pass
        try:
            return Vector(shape.BoundBox.Center)
        except Exception:
            return None

    def _propagate_with_signs(
        self, node: PanelNode,
        placements: Dict[str, Placement],
        active_keys: set,
        sign_map: Dict[StepKey, float],
    ) -> None:
        """BFS с фиксированными знаками (для _child_panel_com_world)."""
        parent_pl = placements[node.id]

        for (child_id, feat_id, line_idx) in node.children:
            child = self.graph.nodes.get(child_id)
            if child is None:
                continue

            key = (feat_id, line_idx) if feat_id else None
            if key is not None and key in active_keys:
                feat = self.features_map.get(feat_id)
                if (feat is None or child.hinge_center is None
                        or child.hinge_axis is None):
                    placements[child_id] = Placement(parent_pl)
                    self._propagate_with_signs(
                        child, placements, active_keys, sign_map)
                    continue

                sign = sign_map.get(key, 1.0)
                angle_deg = feat.angle_deg * sign
                if getattr(feat, "invert", False):
                    angle_deg = -angle_deg

                center = Vector(child.hinge_center)
                axis = Vector(child.hinge_axis)
                axis.normalize()

                rot = Rotation(axis, angle_deg)
                base = center - rot.multVec(center)
                local = Placement(base, rot)
                child_pl = parent_pl.multiply(local)
            else:
                child_pl = Placement(parent_pl)

            placements[child_id] = child_pl
            self._propagate_with_signs(
                child, placements, active_keys, sign_map)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_step_placements(
        self,
        active_keys: List[StepKey],
        moment_factor: float = 1.0,
        fractions: Optional[Dict[StepKey, float]] = None,
    ) -> Dict[str, Placement]:
        """Placement для каждой панели при выполнении active_keys.

        Args:
            active_keys:   список step_key в порядке выполнения.
            moment_factor: доля угла для последнего (активного) ключа,
                           если fractions не задан.
            fractions:     dict[(feat_id, line_idx) -> 0..1]. Если задан,
                           его значения перекрывают moment_factor
                           для соответствующего ключа.

        Returns:
            Dict[node_id -> Placement].
        """
        active_set = set(active_keys)
        last_key = active_keys[-1] if active_keys else None

        placements: Dict[str, Placement] = {}
        root = self.graph.nodes[self.graph.root_id]
        placements[root.id] = Placement()
        self._propagate(
            root, placements, active_set, last_key,
            moment_factor, fractions)
        return placements

    def _propagate(
        self, node: PanelNode,
        placements: Dict[str, Placement],
        active_set: set,
        last_key: Optional[StepKey],
        moment_factor: float,
        fractions: Optional[Dict[StepKey, float]],
    ) -> None:
        """BFS с учётом активных ключей и fractions."""
        parent_pl = placements[node.id]

        for (child_id, feat_id, line_idx) in node.children:
            child = self.graph.nodes.get(child_id)
            if child is None:
                continue

            key = (feat_id, line_idx) if feat_id else None
            if key is not None and key in active_set:
                feat = self.features_map.get(feat_id)
                sign = self._resolved_signs.get(key, 1.0)

                if (feat is None or child.hinge_center is None
                        or child.hinge_axis is None):
                    placements[child_id] = Placement(parent_pl)
                    self._propagate(
                        child, placements, active_set, last_key,
                        moment_factor, fractions)
                    continue

                # fraction: из fractions, иначе moment_factor для последнего
                if fractions is not None and key in fractions:
                    factor = float(fractions[key])
                elif key == last_key:
                    factor = float(moment_factor)
                else:
                    factor = 1.0
                factor = max(0.0, min(1.0, factor))

                angle_deg = feat.angle_deg * factor * sign
                if getattr(feat, "invert", False):
                    angle_deg = -angle_deg

                center = Vector(child.hinge_center)
                axis = Vector(child.hinge_axis)
                axis.normalize()

                rot = Rotation(axis, angle_deg)
                base = center - rot.multVec(center)
                local = Placement(base, rot)
                child_pl = parent_pl.multiply(local)
            else:
                child_pl = Placement(parent_pl)

            placements[child_id] = child_pl
            self._propagate(
                child, placements, active_set, last_key,
                moment_factor, fractions)

    def build_step_compound(
        self,
        active_keys: List[StepKey],
        moment_factor: float = 1.0,
        fractions: Optional[Dict[StepKey, float]] = None,
    ):
        """Собрать Part.Shape всех панелей при active_keys.

        Пытается fuse + removeSplitter; при неудаче возвращает compound.
        """
        placements = self.compute_step_placements(
            active_keys, moment_factor, fractions)
        transformed = []
        for panel_id, placement in placements.items():
            node = self.graph.nodes.get(panel_id)
            if node is None:
                continue
            shape = node.shape.copy()
            shape.Placement = placement.multiply(shape.Placement)
            transformed.append(shape)

        if not transformed:
            raise KinematicEngineError("Нет панелей для compound")

        try:
            result = transformed[0]
            for s in transformed[1:]:
                try:
                    fused = result.fuse(s)
                    if fused is not None and not fused.isNull():
                        result = fused
                except Exception:
                    continue

            try:
                result = result.removeSplitter()
            except Exception:
                pass

            try:
                if result.ShapeType == "Solid":
                    return result
                solids = list(result.Solids)
                if len(solids) == 1:
                    return result
            except Exception:
                pass
        except Exception as e:
            try:
                App.Console.PrintWarning(
                    "[Kinematic] fuse failed: {}\n".format(e))
            except Exception:
                pass

        return Part.makeCompound(transformed)

    def state_at(
        self,
        step_idx: int,
        ordered_keys: List[StepKey],
    ):
        """Состояние после step_idx шагов (первых step_idx ключей)."""
        active = ordered_keys[:step_idx]
        return self.build_step_compound(active, moment_factor=1.0)

    def all_step_keys(self) -> List[StepKey]:
        """Все (feat_id, line_idx) в топологическом порядке.

        Для U-канала с multi-edge фичами: 10 шагов.
        """
        return self._feature_keys_topo_order()

    # ------------------------------------------------------------------
    # Мост в face-pairs / node_id
    # ------------------------------------------------------------------

    def panel_face_index(self, pid: str) -> Optional[int]:
        """face_index панели по её node_id."""
        node = self.graph.nodes.get(pid)
        if node is None:
            return None
        return node.resolved_face_index()

    def step_key_to_panel_id(self, feat_id: str, line_idx: int
                             ) -> Optional[str]:
        """node_id ребёнка, соответствующего шагу (feat_id, line_idx)."""
        for nid, node in self.graph.nodes.items():
            if node.feature_id != feat_id:
                continue
            parent = self.graph.nodes.get(node.parent_id)
            if parent is None:
                continue
            for (cid, ffid, li) in parent.children:
                if cid == nid and ffid == feat_id and int(li) == int(line_idx):
                    return nid
        return None

    def build_face_pair_map(self) -> Dict[StepKey, Tuple[int, int]]:
        """Обратная карта: step_key -> (parent_face, child_face).

        Нужна фасаду `calculate_auto_sequence` для трансляции
        результатов backward-поиска в шаги forward-валидатора.
        """
        out: Dict[StepKey, Tuple[int, int]] = {}
        for nid, node in self.graph.nodes.items():
            if node.feature_id is None:
                continue
            parent = self.graph.nodes.get(node.parent_id)
            if parent is None:
                continue
            line_idx = 0
            for (cid, ffid, li) in parent.children:
                if cid == nid:
                    line_idx = int(li) if li is not None else 0
                    break
            child_fi = node.resolved_face_index()
            parent_fi = parent.resolved_face_index()
            if child_fi is None or parent_fi is None:
                continue
            out[(node.feature_id, line_idx)] = (parent_fi, child_fi)
        return out

    # ------------------------------------------------------------------
    # Диагностика
    # ------------------------------------------------------------------

    def __repr__(self):
        return ("KinematicEngine(panels={}, features={}, signs={}, "
                "steps={})").format(
            len(self.graph.nodes),
            len(self.features_map),
            len(self._resolved_signs),
            len(self.all_step_keys()))

    def describe_signs(self) -> str:
        lines = ["resolved signs:"]
        for k, v in sorted(self._resolved_signs.items()):
            lines.append("  {}[{}] = {:+.0f}".format(k[0], k[1], v))
        return "\n".join(lines)