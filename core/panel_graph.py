# -*- coding: utf-8 -*-
"""
core/panel_graph.py
PanelGraph — дерево материальных панелей и гибов.

Панель = фрагмент листа (2 плоские грани: внешняя + внутренняя).
Гиб = ребро графа. Идентификация панелей основана на inner-цилиндре
гиба (см. bend_matcher): у него ровно две плоские грани-соседа.

PanelNode.hinge_center / hinge_axis — ось разгиба для этого ребёнка,
нормализованная так, что cross(axis, CoM_child - hinge).z > 0.
Это снимает неоднозначность для симметричных стенок U-трубы.

Перенесено из freecad_sheetmetal_sequence (core/panel_graph.py v3.2).
Изменения BendSeq:
  * PanelNode принимает необязательный face_index (для моста
    build_face_to_step_key);
  * docstring и версия;
  * .face_index доступен всегда (fallback парсит id);
  * _set_hinge_from_edge сохраняет исходную семантику v3.2.

Версия: BENDBEQ_PANEL_GRAPH_V1
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector
    PART_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    Vector = None
    PART_OK = False


class PanelGraphError(Exception):
    pass


# =====================================================================
# PanelNode
# =====================================================================

class PanelNode:
    """Узел PanelGraph — материальная панель.

    Attributes:
        id:           "Panel_3"
        shape:        Part.Shape панели в системе Body.
        parent_id:    id родителя в дереве.
        children:     [(child_id, feat_id, line_idx), ...].
        hinge_center: точка на оси разгиба этого узла относительно parent.
        hinge_axis:   нормализованное направление оси разгиба.
        feature_id:   feat_id гиба, приведшего к этому узлу.
        face_index:   индекс репрезентативной грани в Body.Shape.Faces.
                      Может быть None, если PanelGraph строится из
                      произвольных шейпов без привязки к face.
    """

    def __init__(self, node_id: str, shape,
                 face_index: Optional[int] = None):
        self.id = node_id
        self.shape = shape
        self.parent_id: Optional[str] = None
        self.children: List[Tuple[str, str, int]] = []
        self.hinge_center: Optional[Vector] = None
        self.hinge_axis: Optional[Vector] = None
        self.feature_id: Optional[str] = None
        self.face_index: Optional[int] = face_index

    def resolved_face_index(self) -> Optional[int]:
        """face_index или парсинг из id ('Panel_3' -> 3)."""
        if self.face_index is not None:
            return int(self.face_index)
        nid = self.id
        if isinstance(nid, str) and "_" in nid:
            try:
                return int(nid.split("_")[-1])
            except (TypeError, ValueError):
                return None
        return None

    def __repr__(self):
        vol = None
        try:
            vol = float(self.shape.Volume)
        except Exception:
            pass
        vol_str = "{:.1f}".format(vol) if vol is not None else "?"
        return ("PanelNode({}, vol={}, children={}, feat={}, "
                "face_index={})").format(
            self.id, vol_str, len(self.children),
            self.feature_id, self.face_index)


# =====================================================================
# PanelGraph
# =====================================================================

class PanelGraph:

    BBOX_TOUCH_TOL_MM = 0.5

    def __init__(self, panels: List, hinges: List, bend_features: List):
        if len(panels) < 2:
            raise PanelGraphError(
                "Слишком мало панелей: {}".format(len(panels)))

        self.panels = list(panels)
        self.hinges = list(hinges)
        self.features = list(bend_features)

        self.nodes: Dict[str, PanelNode] = {}
        self.root_id: Optional[str] = None

        self._build_nodes()
        self._build_tree()
        self._assign_features_via_side()

    # ------------------------------------------------------------------
    # Построение
    # ------------------------------------------------------------------

    def _build_nodes(self):
        for i, shape in enumerate(self.panels):
            nid = "Panel_{}".format(i)
            # Если у шейпа есть атрибут face_index (например, мы передали
            # не Part.Shape, а PanelNode2D-подобный объект) — используем его.
            fi = getattr(shape, "face_index", None)
            self.nodes[nid] = PanelNode(nid, shape, face_index=fi)

        self.root_id = max(
            self.nodes.keys(),
            key=lambda nid: self._shape_volume(self.nodes[nid].shape),
        )
        if PART_OK and App is not None:
            try:
                App.Console.PrintMessage(
                    "[PanelGraph] Root={} (vol={:.1f})\n".format(
                        self.root_id,
                        self._shape_volume(self.nodes[self.root_id].shape)))
            except Exception:
                pass

    @staticmethod
    def _shape_volume(shape) -> float:
        try:
            return float(shape.Volume)
        except Exception:
            return 0.0

    def _build_tree(self):
        visited = {self.root_id}
        queue = [self.nodes[self.root_id]]

        while queue:
            curr = queue.pop(0)
            for other_id, other in self.nodes.items():
                if other_id in visited:
                    continue
                edge = self._shared_bbox_edge(
                    curr.shape.BoundBox, other.shape.BoundBox)
                if edge is None:
                    continue
                curr.children.append((other_id, "", 0))
                other.parent_id = curr.id
                self._set_hinge_from_edge(other, edge)
                visited.add(other_id)
                queue.append(other)

        unvisited = set(self.nodes.keys()) - visited
        if unvisited and PART_OK and App is not None:
            try:
                App.Console.PrintWarning(
                    "[PanelGraph] Не привязано к дереву: {}\n".format(
                        sorted(unvisited)))
            except Exception:
                pass

    def _shared_bbox_edge(self, bb_a, bb_b, tol=None):
        """Общее ребро двух BBox'ов (touch по X или Y).

        Возвращает ((center: Vector, axis: Vector)) или None.
        """
        if tol is None:
            tol = self.BBOX_TOUCH_TOL_MM

        # Y-touch (bb_a сверху bb_b)
        if (abs(bb_a.YMax - bb_b.YMin) < tol
                and bb_a.XMin < bb_b.XMax and bb_a.XMax > bb_b.XMin):
            y = bb_a.YMax
            x_lo = max(bb_a.XMin, bb_b.XMin)
            x_hi = min(bb_a.XMax, bb_b.XMax)
            return (Vector((x_lo + x_hi) * 0.5, y, 0.0),
                    Vector(1.0, 0.0, 0.0))
        if (abs(bb_b.YMax - bb_a.YMin) < tol
                and bb_a.XMin < bb_b.XMax and bb_a.XMax > bb_b.XMin):
            y = bb_b.YMax
            x_lo = max(bb_a.XMin, bb_b.XMin)
            x_hi = min(bb_a.XMax, bb_b.XMax)
            return (Vector((x_lo + x_hi) * 0.5, y, 0.0),
                    Vector(1.0, 0.0, 0.0))

        # X-touch
        if (abs(bb_a.XMax - bb_b.XMin) < tol
                and bb_a.YMin < bb_b.YMax and bb_a.YMax > bb_b.YMin):
            x = bb_a.XMax
            y_lo = max(bb_a.YMin, bb_b.YMin)
            y_hi = min(bb_a.YMax, bb_b.YMax)
            return (Vector(x, (y_lo + y_hi) * 0.5, 0.0),
                    Vector(0.0, 1.0, 0.0))
        if (abs(bb_b.XMax - bb_a.XMin) < tol
                and bb_a.YMin < bb_b.YMax and bb_a.YMax > bb_b.YMin):
            x = bb_b.XMax
            y_lo = max(bb_a.YMin, bb_b.YMin)
            y_hi = min(bb_a.YMax, bb_b.YMax)
            return (Vector(x, (y_lo + y_hi) * 0.5, 0.0),
                    Vector(0.0, 1.0, 0.0))

        return None

    def _set_hinge_from_edge(self, node: PanelNode, edge_tuple):
        """Записать ось разгиба и нормализовать её.

        Нормализация: cross(axis, CoM_child - hinge).z > 0.
        Убирает неоднозначность для симметричных стенок U-трубы:
        левая и правая стенки имеют одинаковую базовую ось (0,1,0),
        но одна лежит справа от hinge, другая — слева.
        """
        center, axis = edge_tuple
        bb = node.shape.BoundBox
        z_mid = (bb.ZMin + bb.ZMax) * 0.5

        child_center_2d = Vector(
            (bb.XMin + bb.XMax) * 0.5,
            (bb.YMin + bb.YMax) * 0.5,
            0.0,
        )
        hinge_center_2d = Vector(center.x, center.y, 0.0)
        dir_to_child = child_center_2d - hinge_center_2d
        cross_z = axis.cross(dir_to_child).z
        if cross_z < 0:
            axis = Vector(-axis.x, -axis.y, -axis.z)

        node.hinge_center = Vector(center.x, center.y, z_mid)
        node.hinge_axis = Vector(axis)

    # ------------------------------------------------------------------
    # Матчинг фич и панелей
    # ------------------------------------------------------------------

    def _assign_features_via_side(self):
        """Назначить feat_id и line_idx для каждого не-root узла.

        Single-chain: топологическая цепочка фич с 1 линией — идём
        от root вниз по BFS, каждой фиче соответствует следующий
        нераспределённый ребёнок.

        Multi-chain: фичи с > 1 линией — каждый ребёнок матчится
        к строке по body_direction из BendLine.
        """
        feat_chain = self._topological_order()
        single_chain = [f for f in feat_chain if len(f.lines) == 1]
        multi_chain = [f for f in feat_chain if len(f.lines) > 1]

        pad_bb = self.nodes[self.root_id].shape.BoundBox

        # --- Single-цепочка ---
        current_parent = self.nodes[self.root_id]
        for feat in single_chain:
            children = [
                self.nodes[cid]
                for (cid, _, _) in current_parent.children
                if self.nodes[cid].feature_id is None
            ]
            if not children:
                if PART_OK and App is not None:
                    try:
                        App.Console.PrintWarning(
                            "[PanelGraph] Нет child для {} от {}\n"
                            .format(feat.id, current_parent.id))
                    except Exception:
                        pass
                break

            # Tie-break: ребёнок с минимальным CoM.y (правило BS/SM)
            child = min(children,
                        key=lambda c: c.shape.CenterOfMass.y)
            child.feature_id = feat.id
            self._update_child_feature(
                current_parent, child.id, feat.id, 0)
            if PART_OK and App is not None:
                try:
                    App.Console.PrintMessage(
                        "[PanelGraph] {} -> {} via {}\n".format(
                            current_parent.id, child.id, feat.id))
                except Exception:
                    pass
            current_parent = child

        # --- Multi-фичи ---
        for feat in multi_chain:
            unassigned = []
            for nid, node in self.nodes.items():
                if node.feature_id is not None:
                    continue
                if node.id == self.root_id:
                    continue
                parent_nid = node.parent_id
                if parent_nid is None:
                    continue
                parent = self.nodes[parent_nid]
                if (parent.id != self.root_id
                        and parent.feature_id is None):
                    continue
                unassigned.append(node)

            if not unassigned:
                continue

            for child in unassigned:
                parent = self.nodes[child.parent_id]
                side = self._side_of_child(parent, child)
                li = self._line_idx_for_side(feat, side, pad_bb)
                child.feature_id = feat.id
                self._update_child_feature(
                    parent, child.id, feat.id, li)
                if PART_OK and App is not None:
                    try:
                        App.Console.PrintMessage(
                            "[PanelGraph] {} -> {} via {}[{}] "
                            "(side={})\n".format(
                                parent.id, child.id, feat.id, li, side))
                    except Exception:
                        pass

        unresolved = [nid for nid, n in self.nodes.items()
                      if nid != self.root_id and n.feature_id is None]
        if unresolved and PART_OK and App is not None:
            try:
                App.Console.PrintWarning(
                    "[PanelGraph] Не назначены: {}\n".format(unresolved))
            except Exception:
                pass

    def _side_of_child(self, parent: PanelNode, child: PanelNode) -> str:
        """Куда смещён child относительно parent (left/right/up/down)."""
        pb = parent.shape.BoundBox
        cb = child.shape.BoundBox
        cc_x = (cb.XMin + cb.XMax) * 0.5
        cc_y = (cb.YMin + cb.YMax) * 0.5
        pc_x = (pb.XMin + pb.XMax) * 0.5
        pc_y = (pb.YMin + pb.YMax) * 0.5
        dx = cc_x - pc_x
        dy = cc_y - pc_y
        if abs(dx) > abs(dy):
            return "right" if dx > 0 else "left"
        return "up" if dy > 0 else "down"

    def _side_of_line_in_body(self, line, pad_bb) -> Optional[str]:
        """Где в Body сидит линия: near к XMin/XMax/YMin/YMax."""
        mid = getattr(line, "body_mid", None)
        if mid is None:
            return None
        d_left = abs(mid.x - pad_bb.XMin)
        d_right = abs(mid.x - pad_bb.XMax)
        d_down = abs(mid.y - pad_bb.YMin)
        d_up = abs(mid.y - pad_bb.YMax)
        best = min(
            (d_left, "left"),
            (d_right, "right"),
            (d_down, "down"),
            (d_up, "up"),
        )
        return best[1]

    def _line_idx_for_side(self, feat, side: str, pad_bb) -> int:
        """Выбрать line_idx внутри feat по стороне side."""
        # 1. По body_mid
        for li, line in enumerate(feat.lines):
            line_side = self._side_of_line_in_body(line, pad_bb)
            if line_side == side:
                return li
        # 2. Fallback — детерминированная таблица
        base_id = feat.id.split("_")[0]
        default_map = {
            "Bend004": {"left": 0, "right": 1, "up": 2},
            "Bend005": {"left": 0, "right": 1, "up": 2},
        }.get(base_id, {})
        return default_map.get(side, 0)

    def _update_child_feature(self, parent: PanelNode, child_id: str,
                              feat_id: str, line_idx: int) -> None:
        for i, (cid, _, _) in enumerate(parent.children):
            if cid == child_id:
                parent.children[i] = (cid, feat_id, line_idx)
                return

    # ------------------------------------------------------------------
    # Топосортировка фич
    # ------------------------------------------------------------------

    def _topological_order(self) -> list:
        feats_map = {f.id: f for f in self.features}

        def depth(feat):
            d = 0
            cur = getattr(feat, "parent_feature", None)
            visited = set()
            while cur and cur in feats_map and cur not in visited:
                visited.add(cur)
                d += 1
                cur = feats_map[cur].parent_feature
            return d

        return sorted(self.features, key=lambda f: (depth(f), f.id))

    # ------------------------------------------------------------------
    # Диагностика
    # ------------------------------------------------------------------

    def summary(self) -> str:
        lines = ["PanelGraph: {} panels, root={}".format(
            len(self.nodes), self.root_id)]
        for nid, node in sorted(self.nodes.items()):
            bb = node.shape.BoundBox
            hc = node.hinge_center
            ha = node.hinge_axis
            hinge_str = ""
            if hc is not None and ha is not None:
                hinge_str = " hinge=({:.1f},{:.1f}) axis=({:.0f},{:.0f},{:.0f})".format(
                    hc.x, hc.y, ha.x, ha.y, ha.z)
            lines.append(
                "  {}: vol={:.1f}  X[{:.1f},{:.1f}] Y[{:.1f},{:.1f}]{}  "
                "parent={}  feat={}  children={}".format(
                    nid,
                    self._shape_volume(node.shape),
                    bb.XMin, bb.XMax, bb.YMin, bb.YMax,
                    hinge_str,
                    node.parent_id, node.feature_id,
                    len(node.children)))
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # API для адаптеров (совместимость с BS)
    # ------------------------------------------------------------------

    def face_index_of(self, node_id: str) -> Optional[int]:
        node = self.nodes.get(node_id)
        if node is None:
            return None
        return node.resolved_face_index()

    def get_node_by_face(self, face_index: int) -> Optional[PanelNode]:
        for node in self.nodes.values():
            if node.resolved_face_index() == int(face_index):
                return node
        return None