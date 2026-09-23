# -*- coding: utf-8 -*-
"""
core/panel_graph_2d.py
2D-PanelGraph — плоские панели и оси гибов в системе Unfold.Shape.

Совместим с двумя версиями PanelGraph:
  * SM v3.2: graph.nodes = {nid: PanelNode(shape, children, feature_id)},
             graph.panels = List[Part.Solid], graph.edges отсутствует.
  * BS:      graph.panels = List[PanelNode(face_index, area_mm2, ...)],
             graph.edges = List[PanelEdge(parent_panel_face_index, ...)].

Версия: BENDBEQ_PANEL_GRAPH_2D_V2
"""

from __future__ import annotations

import math
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

try:
    from shapely.geometry import Polygon, LineString
    from shapely.ops import polygonize, unary_union
    SHAPELY_OK = True
except ImportError:
    Polygon = None
    LineString = None
    polygonize = None
    unary_union = None
    SHAPELY_OK = False


BEND_EXTEND_LEN = 0.2
MIN_SHARED_BOUNDARY_LEN = 1e-3
MAX_CELLS = 200


# =====================================================================
# Чтение геометрии Unfold.Shape
# =====================================================================

def _normal_of_face(face):
    surf = getattr(face, "Surface", None)
    if surf is None or surf.__class__.__name__ != "Plane":
        return None
    try:
        ax = surf.Axis
    except Exception:
        return None
    x, y, z = float(ax.x), float(ax.y), float(ax.z)
    if str(getattr(face, "Orientation", "")) == "Reversed":
        x, y, z = -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return None
    return (x / n, y / n, z / n)


def _face_area(face):
    try:
        return float(face.Area)
    except Exception:
        return 0.0


def find_top_face(unfold_shape):
    top = None
    for i, f in enumerate(unfold_shape.Faces or []):
        n = _normal_of_face(f)
        if n is None or n[2] < 0.9:
            continue
        a = _face_area(f)
        if top is None or a > top[2]:
            top = (i, f, a)
    return top


def read_outer_ring_2d(face):
    outer = getattr(face, "OuterWire", None)
    if outer is None:
        return None
    try:
        edges = getattr(outer, "OrderedEdges", None) or list(outer.Edges or [])
    except Exception:
        edges = list(outer.Edges or [])
    pts = []
    for e in edges:
        try:
            p = e.valueAt(e.FirstParameter)
            pts.append((float(p.x), float(p.y)))
        except Exception:
            continue
    out = []
    for p in pts:
        if (not out
                or abs(p[0] - out[-1][0]) > 1e-6
                or abs(p[1] - out[-1][1]) > 1e-6):
            out.append(p)
    if len(out) >= 2 and (abs(out[0][0] - out[-1][0]) <= 1e-6
                           and abs(out[0][1] - out[-1][1]) <= 1e-6):
        out.pop()
    return out if len(out) >= 3 else None


def read_bend_segments_2d(bends_sketch, unfold_shape):
    raw = []
    sh = getattr(bends_sketch, "Shape", None)
    if sh is not None:
        for e in sh.Edges or []:
            c = getattr(e, "Curve", None)
            if c is None or c.__class__.__name__ != "Line":
                continue
            p1 = e.valueAt(e.FirstParameter)
            p2 = e.valueAt(e.LastParameter)
            raw.append((float(p1.x), float(p1.y),
                        float(p2.x), float(p2.y)))

    bb = getattr(unfold_shape, "BoundBox", None)
    ox = float(bb.XMin) if bb is not None else 0.0
    oy = float(bb.YMin) if bb is not None else 0.0

    return ([(x1 + ox, y1 + oy, x2 + ox, y2 + oy)
             for (x1, y1, x2, y2) in raw],
            (ox, oy))


# =====================================================================
# Polygonize
# =====================================================================

def polygonize_cells(outer_ring, bend_segments_2d,
                     extend_len: float = BEND_EXTEND_LEN):
    if not SHAPELY_OK:
        return None, "shapely not available"

    try:
        ring = list(outer_ring)
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        outer_poly = Polygon(ring)
        if not outer_poly.is_valid:
            outer_poly = outer_poly.buffer(0)

        lines = [outer_poly.exterior]
        for (x1, y1, x2, y2) in bend_segments_2d:
            dx, dy = x2 - x1, y2 - y1
            L = math.hypot(dx, dy)
            if L < 1e-9:
                continue
            ux, uy = dx / L, dy / L
            lines.append(LineString([
                (x1 - ux * extend_len, y1 - uy * extend_len),
                (x2 + ux * extend_len, y2 + uy * extend_len),
            ]))

        noded = unary_union(lines)
        all_cells = list(polygonize(noded.geoms))
        cells = [p for p in all_cells if outer_poly.contains(p.centroid)]

        if len(cells) > MAX_CELLS:
            return None, ("polygonize produced too many cells: {} "
                          "(inner {} of {})".format(
                              len(cells), len(cells), len(all_cells)))
        return cells, None
    except Exception as e:
        return None, "polygonize failed: {}: {}".format(
            type(e).__name__, e)


def build_cell_adjacency(cells,
                         min_shared_len: float = MIN_SHARED_BOUNDARY_LEN
                         ) -> Dict[int, Set[int]]:
    adj = {i: set() for i in range(len(cells))}
    for i in range(len(cells)):
        for j in range(i + 1, len(cells)):
            try:
                inter = cells[i].intersection(cells[j])
            except Exception:
                continue
            if inter.length > min_shared_len:
                adj[i].add(j)
                adj[j].add(i)
    return adj


# =====================================================================
# Совместимость с двумя версиями PanelGraph
# =====================================================================

def _graph_panels(graph_3d):
    """Список PanelNode — из nodes (SM) или panels (BS)."""
    nodes = getattr(graph_3d, "nodes", None)
    if nodes:
        try:
            return list(nodes.values())
        except Exception:
            pass
    return list(getattr(graph_3d, "panels", []) or [])


def _node_face_index(node):
    """face_index панели: реальный или fallback из 'Panel_N' -> N."""
    fi = getattr(node, "face_index", None)
    if fi is not None:
        try:
            return int(fi)
        except (TypeError, ValueError):
            pass
    if hasattr(node, "resolved_face_index"):
        try:
            fi = node.resolved_face_index()
            if fi is not None:
                return int(fi)
        except Exception:
            pass
    nid = getattr(node, "id", None)
    if isinstance(nid, str) and "_" in nid:
        try:
            return int(nid.split("_")[-1])
        except (ValueError, IndexError):
            pass
    return None


def _node_area(node):
    """area_mm2 панели: из поля или из shape.Area."""
    a = getattr(node, "area_mm2", None)
    if a is not None:
        try:
            return float(a)
        except (TypeError, ValueError):
            pass
    shape = getattr(node, "shape", None)
    if shape is not None:
        try:
            return float(shape.Area)
        except Exception:
            pass
    return 0.0


def _graph_edges(graph_3d):
    """Список (parent_face, child_face) — совместимо с SM и BS."""
    edges = getattr(graph_3d, "edges", None)
    if edges:
        out = []
        for e in edges:
            a = getattr(e, "parent_panel_face_index", None)
            b = getattr(e, "child_panel_face_index", None)
            if a is None or b is None:
                continue
            out.append((int(a), int(b)))
        return out

    # SM v3.2 — edges нет; строим из parent_id у nodes
    nodes = getattr(graph_3d, "nodes", None)
    if not nodes:
        return []
    out = []
    for nid, node in nodes.items():
        parent_id = getattr(node, "parent_id", None)
        if parent_id is None:
            continue
        parent = nodes.get(parent_id)
        if parent is None:
            continue
        a = _node_face_index(parent)
        b = _node_face_index(node)
        if a is None or b is None:
            continue
        out.append((a, b))
    return out


# =====================================================================
# Матчинг
# =====================================================================

def _panel_ids_and_degrees(graph_3d):
    panels = _graph_panels(graph_3d)

    panel_ids: List[int] = []
    for p in panels:
        fi = _node_face_index(p)
        if fi is None:
            continue
        if fi not in panel_ids:
            panel_ids.append(fi)

    panel_adj: Dict[int, Set[int]] = {pid: set() for pid in panel_ids}
    for a, b in _graph_edges(graph_3d):
        if a not in panel_adj:
            panel_adj[a] = set()
            if a not in panel_ids:
                panel_ids.append(a)
        if b not in panel_adj:
            panel_adj[b] = set()
            if b not in panel_ids:
                panel_ids.append(b)
        panel_adj[a].add(b)
        panel_adj[b].add(a)

    panel_deg = {pid: len(panel_adj.get(pid, ())) for pid in panel_ids}
    return panel_ids, panel_adj, panel_deg


def match_cells_to_panels(cells, graph_3d):
    panel_ids, panel_adj, panel_deg = _panel_ids_and_degrees(graph_3d)
    cell_adj = build_cell_adjacency(cells)
    cell_deg = {ci: len(cell_adj.get(ci, ())) for ci in range(len(cells))}

    if len(cells) != len(panel_ids):
        return ({}, {}, "count mismatch: cells={} panels={}".format(
            len(cells), len(panel_ids)))
    if sorted(cell_deg.values()) != sorted(panel_deg.values()):
        return ({}, {}, "degree mismatch: cells={} panels={}".format(
            sorted(cell_deg.values()), sorted(panel_deg.values())))

    panel_area = {}
    for p in _graph_panels(graph_3d):
        fi = _node_face_index(p)
        if fi is not None:
            panel_area[fi] = _node_area(p)

    ordered = sorted(panel_ids, key=lambda pid: -panel_deg[pid])
    used: Set[int] = set()
    cur: Dict[int, int] = {}

    def consistent(pid, ci):
        for pnb in panel_adj.get(pid, ()):
            if pnb in cur and cur[pnb] not in cell_adj.get(ci, ()):
                return False
        for cnb in cell_adj.get(ci, ()):
            if cnb in used:
                if not any(cur.get(p2) == cnb
                           and p2 in panel_adj.get(pid, ())
                           for p2 in cur):
                    return False
        return True

    def bt(idx):
        if idx == len(ordered):
            return True
        pid = ordered[idx]
        target = panel_area.get(pid, 0.0)
        cands = sorted(
            (ci for ci in range(len(cells))
             if ci not in used and cell_deg[ci] == panel_deg[pid]),
            key=lambda ci: abs(cells[ci].area - target),
        )
        for ci in cands:
            if not consistent(pid, ci):
                continue
            cur[pid] = ci
            used.add(ci)
            if bt(idx + 1):
                return True
            del cur[pid]
            used.discard(ci)
        return False

    if not bt(0):
        return ({}, {}, "no isomorphism found")

    matches = {pid: (cur[pid], abs(cells[cur[pid]].area - panel_area[pid]))
               for pid in ordered}
    return matches, {}, None


# =====================================================================
# Refine (опционально, работает только если body_shape даёт реальные
# face_index; для SM v3.2 передавать body_shape=None)
# =====================================================================

def _face_normal_outward_3d(face):
    surf = getattr(face, "Surface", None)
    if surf is None or surf.__class__.__name__ != "Plane":
        return None
    try:
        ax = surf.Axis
    except Exception:
        return None
    x, y, z = float(ax.x), float(ax.y), float(ax.z)
    if str(getattr(face, "Orientation", "")) == "Reversed":
        x, y, z = -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return None
    return (x / n, y / n, z / n)


def _refine_root_children_matching(matches, cells, graph_3d, body_shape):
    """Sweep поддеревьев прямых детей root. Требует реальные face_index."""
    if body_shape is None or not matches:
        return

    panel_adj: Dict[int, Set[int]] = {}
    for a, b in _graph_edges(graph_3d):
        panel_adj.setdefault(a, set()).add(b)
        panel_adj.setdefault(b, set()).add(a)

    panel_deg = {fi: len(panel_adj.get(fi, ())) for fi in panel_adj}
    if not panel_deg:
        return
    root = min(panel_deg, key=lambda fi: (-panel_deg[fi], fi))
    if root not in matches:
        return

    parent_of = {root: None}
    children_of: Dict[int, List[int]] = {root: []}
    q = deque([root])
    while q:
        n = q.popleft()
        for nb in sorted(panel_adj.get(n, ())):
            if nb in parent_of:
                continue
            parent_of[nb] = n
            children_of.setdefault(n, []).append(nb)
            children_of.setdefault(nb, [])
            q.append(nb)

    try:
        root_face = body_shape.Faces[root]
    except Exception:
        return

    C3r = root_face.CenterOfMass
    C3_root = (float(C3r.x), float(C3r.y), float(C3r.z))

    root_cell = matches[root][0]
    C2_root = (float(cells[root_cell].centroid.x),
               float(cells[root_cell].centroid.y))

    panel_area = {}
    for p in _graph_panels(graph_3d):
        fi = _node_face_index(p)
        if fi is not None:
            panel_area[fi] = _node_area(p)

    def collect_subtree(node):
        out = [node]
        for c in children_of.get(node, ()):
            out.extend(collect_subtree(c))
        return out

    root_children = list(children_of.get(root, []))
    pairs: List[Tuple[int, int]] = []
    used: Set[int] = set()
    for i, c in enumerate(root_children):
        if c in used:
            continue
        for c2 in root_children[i + 1:]:
            if c2 in used:
                continue
            if abs(panel_area.get(c, 0.0) - panel_area.get(c2, 0.0)) > 0.01:
                continue
            sa = collect_subtree(c)
            sb = collect_subtree(c2)
            if len(sa) != len(sb):
                continue
            pairs.append((c, c2))
            used.add(c)
            used.add(c2)
            break

    for (a, b) in pairs:
        if a not in matches or b not in matches:
            continue
        try:
            face_a = body_shape.Faces[a]
        except Exception:
            continue
        C3a = face_a.CenterOfMass
        v3_a = (float(C3a.x) - C3_root[0],
                float(C3a.y) - C3_root[1])
        ca = matches[a][0]
        C2a = (float(cells[ca].centroid.x),
               float(cells[ca].centroid.y))
        v2_a = (C2a[0] - C2_root[0], C2a[1] - C2_root[1])
        dot_a = v3_a[0] * v2_a[0] + v3_a[1] * v2_a[1]
        if dot_a >= 0:
            continue

        sa = collect_subtree(a)
        sb = collect_subtree(b)
        if len(sa) != len(sb):
            continue
        for x, y in zip(sa, sb):
            matches[x], matches[y] = matches[y], matches[x]


# =====================================================================
# Модель 2D-графа
# =====================================================================

class PanelNode2D(object):
    __slots__ = ("face_index", "cell_index", "exterior_2d",
                 "centroid_2d", "area_mm2", "cell_z")

    def __init__(self, face_index, cell_index, exterior_2d,
                 centroid_2d, area_mm2, cell_z):
        self.face_index = int(face_index)
        self.cell_index = int(cell_index)
        self.exterior_2d = list(exterior_2d)
        self.centroid_2d = (float(centroid_2d[0]), float(centroid_2d[1]))
        self.area_mm2 = float(area_mm2)
        self.cell_z = float(cell_z)

    def __repr__(self):
        return ("PanelNode2D(face={}, cell={}, area={:.3f}, "
                "centroid=({:.3f}, {:.3f}))").format(
            self.face_index, self.cell_index, self.area_mm2,
            self.centroid_2d[0], self.centroid_2d[1])


class BendAxis2D(object):
    __slots__ = ("parent_face_index", "child_face_index",
                 "cell_p", "cell_c",
                 "p1", "p2", "direction", "length_mm")

    def __init__(self, parent_face_index, child_face_index,
                 cell_p, cell_c, p1, p2):
        self.parent_face_index = int(parent_face_index)
        self.child_face_index = int(child_face_index)
        self.cell_p = int(cell_p)
        self.cell_c = int(cell_c)
        self.p1 = (float(p1[0]), float(p1[1]))
        self.p2 = (float(p2[0]), float(p2[1]))
        dx = self.p2[0] - self.p1[0]
        dy = self.p2[1] - self.p1[1]
        L = math.hypot(dx, dy)
        self.length_mm = L
        if L > 1e-12:
            self.direction = (dx / L, dy / L)
        else:
            self.direction = (0.0, 0.0)

    def __repr__(self):
        return ("BendAxis2D(p={} -> c={}, L={:.3f}, p1=({:.3f}, {:.3f}), "
                "dir=({:.4f}, {:.4f}))").format(
            self.parent_face_index, self.child_face_index, self.length_mm,
            self.p1[0], self.p1[1],
            self.direction[0], self.direction[1])


class PanelGraph2D(object):
    def __init__(self):
        self.panels: Dict[int, PanelNode2D] = {}
        self.bends: List[BendAxis2D] = []
        self.adjacency: Dict[int, Set[int]] = {}
        self.panel_to_cell: Dict[int, int] = {}
        self.cell_to_panel: Dict[int, int] = {}
        self.cell_z: float = 0.0
        self.thickness_mm: float = 0.0
        self._bend_by_pair: Dict[Tuple[int, int], BendAxis2D] = {}
        self._anchor: Optional[int] = None
        self.build_log: Dict = {}

    def get_bend_between(self, parent_face, child_face):
        return self._bend_by_pair.get((int(parent_face), int(child_face)))

    def bend_by_face_pair(self, a, b):
        axis = self._bend_by_pair.get((int(a), int(b)))
        if axis is None:
            axis = self._bend_by_pair.get((int(b), int(a)))
        return axis

    def get_bend_by_cell_pair(self, cell_p, cell_c):
        a = self.cell_to_panel.get(int(cell_p))
        b = self.cell_to_panel.get(int(cell_c))
        if a is None or b is None:
            return None
        return self._bend_by_pair.get((a, b))

    def neighbors(self, face_index):
        return set(self.adjacency.get(int(face_index), ()))

    @property
    def anchor_face_index(self):
        if self._anchor is not None:
            return self._anchor
        if not self.panels:
            return None
        best = min(self.panels.keys(),
                   key=lambda fi: (-len(self.adjacency.get(fi, ())), fi))
        self._anchor = best
        return best

    def validate(self):
        n_panels = len(self.panels)
        n_bends = len(self.bends)

        adj_edges = 0
        for face, nbrs in self.adjacency.items():
            adj_edges += len(nbrs)
        adj_edges //= 2

        panel_deg = sorted(len(v) for v in self.adjacency.values())

        ok = (
            n_bends == adj_edges
            and len(self.panel_to_cell) == n_panels
            and len(self.cell_to_panel) == n_panels
            and len(self._bend_by_pair) == 2 * n_bends
        )

        return {
            "panels_n": n_panels,
            "bends_n": n_bends,
            "adjacency_edges": adj_edges,
            "panel_degrees": panel_deg,
            "anchor_face_index": self.anchor_face_index,
            "cell_z": self.cell_z,
            "thickness_mm": self.thickness_mm,
            "ok": ok,
            "build_log": dict(self.build_log),
        }


# =====================================================================
# Точка входа
# =====================================================================

def _common_boundary_segment(cell_a, cell_b):
    try:
        inter = cell_a.boundary.intersection(cell_b.boundary)
    except Exception:
        return None
    if inter.is_empty:
        return None
    geoms = list(getattr(inter, "geoms", [inter]))
    best = None
    best_len = 0.0
    for g in geoms:
        if g.geom_type != "LineString":
            continue
        L = float(g.length)
        if L > best_len:
            best_len = L
            best = g
    if best is None or best_len < MIN_SHARED_BOUNDARY_LEN:
        return None
    return best


def build_panel_graph_2d(unfold_obj, bends_sketch, graph_3d,
                          thickness_mm=None, body_shape=None):
    """Собрать PanelGraph2D через shapely.

    Совместим с SM v3.2 и BS. При body_shape=None refine-эвристика
    пропускается (для SM v3.2 face_index панелей не совпадает с
    body.Shape.Faces, refine не применим).
    """
    if not SHAPELY_OK:
        raise RuntimeError(
            "build_panel_graph_2d: shapely не установлен")
    if unfold_obj is None:
        raise RuntimeError("build_panel_graph_2d: unfold_obj is None")
    if bends_sketch is None:
        raise RuntimeError("build_panel_graph_2d: bends_sketch is None")
    if graph_3d is None:
        raise RuntimeError("build_panel_graph_2d: graph_3d is None")

    unfold_shape = getattr(unfold_obj, "Shape", None)
    if unfold_shape is None:
        raise RuntimeError("build_panel_graph_2d: unfold_obj.Shape is None")

    top = find_top_face(unfold_shape)
    if top is None:
        raise RuntimeError(
            "build_panel_graph_2d: no top planar face (+Z)")
    top_idx, top_face, top_area = top

    outer_ring = read_outer_ring_2d(top_face)
    if outer_ring is None:
        raise RuntimeError(
            "build_panel_graph_2d: cannot read outer ring")

    bend_segments, (off_x, off_y) = read_bend_segments_2d(
        bends_sketch, unfold_shape)

    cells, err = polygonize_cells(outer_ring, bend_segments)
    if cells is None:
        raise RuntimeError("build_panel_graph_2d: " + err)

    sum_cell_area = sum(c.area for c in cells)
    area_diff = abs(top_area - sum_cell_area)

    matches, ambiguous, err = match_cells_to_panels(cells, graph_3d)
    if err is not None:
        raise RuntimeError(
            "build_panel_graph_2d: matching failed: " + err)
    if ambiguous:
        raise RuntimeError(
            "build_panel_graph_2d: ambiguous: {}".format(
                sorted(ambiguous.keys())))

    if body_shape is not None:
        _refine_root_children_matching(matches, cells, graph_3d, body_shape)

    z_u = float(top_face.CenterOfMass.z)

    g2 = PanelGraph2D()
    g2.cell_z = z_u
    g2.thickness_mm = (float(thickness_mm)
                       if thickness_mm is not None else 0.0)
    g2.build_log = {
        "top_face_index": top_idx,
        "top_face_area": top_area,
        "sum_cell_area": sum_cell_area,
        "area_conservation_diff": area_diff,
        "offset_2d": (off_x, off_y),
        "n_cells": len(cells),
        "n_bend_segments_2d": len(bend_segments),
        "refinement_applied": body_shape is not None,
    }

    for face_index, (ci, area_delta) in matches.items():
        c = cells[ci]
        ext = [(float(x), float(y)) for (x, y) in list(c.exterior.coords)]
        node = PanelNode2D(
            face_index=face_index,
            cell_index=ci,
            exterior_2d=ext,
            centroid_2d=(float(c.centroid.x), float(c.centroid.y)),
            area_mm2=float(c.area),
            cell_z=z_u,
        )
        g2.panels[face_index] = node
        g2.panel_to_cell[face_index] = ci
        g2.cell_to_panel[ci] = face_index

    edges_list = _graph_edges(graph_3d)

    for a, b in edges_list:
        g2.adjacency.setdefault(a, set()).add(b)
        g2.adjacency.setdefault(b, set()).add(a)

    for a, b in edges_list:
        ci_p = g2.panel_to_cell.get(a)
        ci_c = g2.panel_to_cell.get(b)
        if ci_p is None or ci_c is None:
            raise RuntimeError(
                "build_panel_graph_2d: edge ({}, {}) refers to "
                "unmapped panel".format(a, b))
        seg = _common_boundary_segment(cells[ci_p], cells[ci_c])
        if seg is None:
            raise RuntimeError(
                "build_panel_graph_2d: no common boundary between "
                "panels {} and {} (cells {} and {})".format(
                    a, b, ci_p, ci_c))
        coords = list(seg.coords)
        p1, p2 = coords[0], coords[-1]
        axis = BendAxis2D(
            parent_face_index=a,
            child_face_index=b,
            cell_p=ci_p,
            cell_c=ci_c,
            p1=(p1[0], p1[1]),
            p2=(p2[0], p2[1]),
        )
        g2.bends.append(axis)
        g2._bend_by_pair[(a, b)] = axis
        g2._bend_by_pair[(b, a)] = axis

    if len(g2.bends) != len(edges_list):
        raise RuntimeError(
            "build_panel_graph_2d: bends_n={} != edges={}".format(
                len(g2.bends), len(edges_list)))

    val = g2.validate()
    g2.build_log.update({
        "validate_ok": val["ok"],
    })

    return g2