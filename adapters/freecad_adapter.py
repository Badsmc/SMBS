# -*- coding: utf-8 -*-
"""
adapters/freecad_adapter.py
FreeCADSheetMetalAdapter — extraction BendSpec и BendFeature из Body.

Версия: BENDBEQ_ADAPTER_V12_4

Возможности:
  * get_flat_shape / get_reference_body_shape / get_pad_shape
  * get_bend_specs         — List[BendSpec], single- и multi-edge
  * build_bend_features    — List[BendFeature] с BendLine и target_com
                             (формула v12.4: tc = (bm.x, bm.y, delta_cz))

BendSeq-изменения относительно SM v12.4:
  * metadata["matching_confidence"] — hook для core.bend_matcher
    (заполняется, если модуль доступен).
  * Headless-safe: без FreeCAD импорт не падает, все методы возвращают [].
  * Typed, docstrings.

Версия зафиксирована на уровне V12.4 — сохранена вся логика SM.
"""

from __future__ import annotations

from typing import List, Optional, Callable, TYPE_CHECKING

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    Vector = None
    _FC_OK = False

if TYPE_CHECKING:
    from core.bend_feature import BendFeature, BendLine

# Hook для MatchingConfidence (BS)
try:
    from core import bend_matcher as _bend_matcher
    _BM_AVAILABLE = True
except ImportError:
    _bend_matcher = None
    _BM_AVAILABLE = False


_FEAT_PREFIX = "[ADAPTER-FEAT]"
_UNFOLD_PREFIX = "[ADAPTER-UNFOLD]"
DIR_DIAG = True


def _feat_diag(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_FEAT_PREFIX, msg))
    except Exception:
        pass


def _unfold_diag(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_UNFOLD_PREFIX, msg))
    except Exception:
        pass


def _fmt_v(v, nd: int = 3) -> str:
    try:
        return "({:.{}f},{:.{}f},{:.{}f})".format(
            float(v.x), nd, float(v.y), nd, float(v.z), nd)
    except Exception:
        return "(?,?,?)"


def _quantity_deg(q, default: float = 90.0) -> float:
    try:
        if hasattr(q, "getValueAs"):
            return float(q.getValueAs("deg"))
        if hasattr(q, "Value"):
            return float(q.Value)
        return float(q)
    except Exception:
        return default


def _quantity_mm(q, default: float = 1.0) -> float:
    try:
        if hasattr(q, "getValueAs"):
            return float(q.getValueAs("mm"))
        if hasattr(q, "Value"):
            return float(q.Value)
        return float(q)
    except Exception:
        return default


def _edge_endpoints(edge):
    """(p1, p2) концов ребра, или (None, None)."""
    try:
        verts = getattr(edge, "Vertexes", None) or []
        if len(verts) >= 2:
            return (Vector(verts[0].Point), Vector(verts[-1].Point))
    except Exception:
        pass
    try:
        return (Vector(edge.StartPoint), Vector(edge.EndPoint))
    except Exception:
        return None, None


def _rotate_point_around_axis(point, center, axis, angle_deg):
    """Повернуть точку вокруг оси, проходящей через center."""
    rot = App.Rotation(Vector(axis), float(angle_deg))
    rel = Vector(point) - Vector(center)
    rotated = rot.multVec(rel)
    return rotated + Vector(center)


def _safe_com(shape):
    """CoM формы с несколькими fallback."""
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


# =====================================================================
# BendSpec
# =====================================================================

class BendSpec:
    """Описание одного физического гиба.

    Attributes:
        id:        "Bend001" или "Bend004_1".
        name:      то же, что id.
        center:    центр линии гиба.
        axis:      единичная ось гиба.
        angle:     внутренний угол в градусах.
        length:    длина линии гиба.
        direction: "up" | "down".
        normal:    нормаль листа (0,0,1).
        up_hint:   перпендикуляр к оси в плоскости листа.
        source:    ссылка на feature-объект.
        metadata:  dict с feature, parent_feature, sub_element,
                   body_direction, invert, radius, flange_length_mm,
                   length_list, edge_length_mm, matching_confidence.
    """

    def __init__(self, id, name, center, axis, angle, length,
                 direction="up", normal=None, up_hint=None,
                 source=None, metadata=None):
        self.id = id
        self.name = name
        self.center = Vector(center)
        self.axis = Vector(axis)
        self.angle = float(angle)
        self.length = float(length)
        self.direction = direction
        self.normal = Vector(normal) if normal else Vector(0, 0, 1)
        self.up_hint = Vector(up_hint) if up_hint else Vector(0, 0, 1)
        self.source = source
        self.metadata = metadata or {}
        self.key = id

        if self.axis.Length > 1e-9:
            self.axis.normalize()
        if self.up_hint.Length > 1e-9:
            self.up_hint.normalize()

    def __repr__(self):
        return ("BendSpec(id={!r}, angle={:.1f}, length={:.1f}, "
                "dir={!r})").format(
            self.id, self.angle, self.length, self.direction)


# =====================================================================
# FreeCADSheetMetalAdapter
# =====================================================================

class FreeCADSheetMetalAdapter:

    INVERT_DIRECTION_MAP = False

    def __init__(self, unfold_obj,
                 logger: Optional[Callable[[str], None]] = None):
        self.unfold_obj = unfold_obj
        self.logger = logger
        self._log = logger if callable(logger) else (lambda m: None)
        self._bends_cache: Optional[List[BendSpec]] = None
        self._features_cache: Optional[List] = None

    # ------------------------------------------------------------------
    # Плоская развёртка / Body
    # ------------------------------------------------------------------

    def get_flat_shape(self):
        if not _FC_OK or self.unfold_obj is None:
            return None
        if (hasattr(self.unfold_obj, "Shape")
                and not self.unfold_obj.Shape.isNull()):
            return self.unfold_obj.Shape.copy()
        return None

    def get_reference_body_shape(self):
        if not _FC_OK:
            return None
        body = self._find_body()
        if body is None or not hasattr(body, "Shape"):
            return None
        try:
            if body.Shape is None or body.Shape.isNull():
                return None
            return body.Shape.copy()
        except Exception:
            return None

    def get_pad_shape(self):
        if not _FC_OK:
            return None
        body = self._find_body()
        if body is None:
            return None
        for o in getattr(body, "Group", []) or []:
            try:
                if o.isDerivedFrom("PartDesign::Pad"):
                    if hasattr(o, "Shape") and not o.Shape.isNull():
                        return o.Shape.copy()
            except Exception:
                continue
        return None

    def invalidate_cache(self) -> None:
        self._bends_cache = None
        self._features_cache = None

    # ------------------------------------------------------------------
    # BendSpec
    # ------------------------------------------------------------------

    def get_bend_specs(self) -> List[BendSpec]:
        """Собрать List[BendSpec] из SMBendWall-фич Body."""
        if self._bends_cache is not None:
            return list(self._bends_cache)

        if not _FC_OK:
            self._bends_cache = []
            return []

        body = self._find_body()
        bends: List[BendSpec] = []
        if body is not None:
            bends = self._bend_specs_from_features(body)
            if bends:
                _feat_diag("из фич: {} BendSpec (Body='{}')".format(
                    len(bends), body.Name))

        if not bends:
            _feat_diag("Bend-фич нет — BendSpec пуст.")

        self._bends_cache = list(bends)
        return list(bends)

    def _bend_specs_from_features(self, body) -> List[BendSpec]:
        feat_list = []
        try:
            group = list(getattr(body, "Group", []) or [])
        except Exception:
            group = []
        for o in group:
            if self._is_smbendwall(o):
                feat_list.append(o)
        if not feat_list:
            return []

        bends = []
        idx = 0
        for feat in feat_list:
            specs = self._bend_specs_from_one_feature(feat, idx)
            for s in specs:
                bends.append(s)
                idx += 1
        return bends

    def _bend_specs_from_one_feature(self, feat, start_idx) -> List[BendSpec]:
        out: List[BendSpec] = []
        try:
            raw_angle = getattr(feat, "angle", None)
            raw_invert = bool(getattr(feat, "invert", False))
            raw_radius = getattr(feat, "radius", None)
            raw_lengths = list(getattr(feat, "LengthList", []) or [])

            angle_deg = _quantity_deg(raw_angle, 90.0)
            radius = _quantity_mm(raw_radius, 1.0)

            bo = getattr(feat, "baseObject", None)
            if bo is None:
                return out
            try:
                parent, subs = bo[0], list(bo[1])
            except Exception:
                return out
            if parent is None or not subs:
                return out

            try:
                parent_shape = parent.Shape
                if parent_shape is None or parent_shape.isNull():
                    return out
            except Exception:
                return out

            if self.INVERT_DIRECTION_MAP:
                direction = "up" if raw_invert else "down"
            else:
                direction = "down" if raw_invert else "up"

            multi = len(subs) > 1
            for k, sub in enumerate(subs):
                try:
                    edge = parent_shape.getElement(sub)
                except Exception:
                    edge = None
                if edge is None:
                    continue
                spec = self._build_spec_from_edge(
                    feat, parent, sub, edge,
                    angle_deg, direction, radius,
                    raw_lengths, multi, start_idx + k)
                if spec is not None:
                    out.append(spec)
        except Exception:
            pass
        return out

    def _build_spec_from_edge(self, feat, parent, sub, edge,
                               angle_deg, direction, radius,
                               length_list, multi, idx):
        p1, p2 = _edge_endpoints(edge)
        if p1 is None or p2 is None:
            return None

        center = (p1 + p2) * 0.5
        axis = p2 - p1
        if axis.Length < 1e-9:
            return None
        length = axis.Length
        axis.normalize()

        sheet_normal = Vector(0, 0, 1)
        up_hint = axis.cross(sheet_normal)
        if up_hint.Length < 1e-8:
            up_hint = Vector(0, 0, 1)
        else:
            up_hint.normalize()

        if multi:
            spec_id = "{}_{}".format(feat.Name, idx)
        else:
            spec_id = feat.Name

        flange_len = 0.0
        if length_list:
            try:
                flange_len = _quantity_mm(length_list[0], 0.0)
            except Exception:
                flange_len = 0.0

        body_direction = None
        try:
            parent_com = Vector(parent.Shape.CenterOfMass)
            delta = center - parent_com
            if abs(delta.x) > abs(delta.y):
                body_direction = "right" if delta.x > 0 else "left"
            else:
                body_direction = "up" if delta.y > 0 else "down"
        except Exception:
            body_direction = None

        # Hook: MatchingConfidence (BS)
        matching_confidence = None
        if _BM_AVAILABLE:
            try:
                matching_confidence = _compute_matching_confidence(
                    parent.Shape, edge, radius)
            except Exception:
                matching_confidence = None

        return BendSpec(
            id=spec_id, name=spec_id,
            center=center, axis=axis,
            angle=angle_deg, length=length, direction=direction,
            normal=sheet_normal, up_hint=up_hint, source=feat,
            metadata={
                "source": "feature",
                "feature": feat.Name,
                "parent_feature": parent.Name,
                "sub_element": sub,
                "body_direction": body_direction,
                "invert": bool(getattr(feat, "invert", False)),
                "radius": radius,
                "flange_length_mm": flange_len,
                "length_list": [str(x) for x in length_list],
                "edge_length_mm": length,
                "matching_confidence": matching_confidence,
            },
        )

    # ------------------------------------------------------------------
    # BendFeature
    # ------------------------------------------------------------------

    def build_bend_features(self):
        """List[BendFeature] с BendLine и target_com (SM v12.4)."""
        if not _FC_OK:
            return []
        from core.bend_feature import BendFeature, BendLine

        if self._features_cache is not None:
            return list(self._features_cache)

        doc = App.ActiveDocument
        if doc is None:
            return []
        body = self._find_body()
        if body is None:
            return []

        sm_features = self._get_smbendwall_features_ordered(body)
        if not sm_features:
            _unfold_diag("SMBendWall фич не найдено")
            return []

        _unfold_diag("SMBendWall фич: {}".format(len(sm_features)))

        flat_bend_info = {}
        features: List = []

        for feat in sm_features:
            feat_name = feat.Name
            parent_name = self._get_parent_name(feat)
            subs = self._get_feat_subs(feat)

            parent_obj = self._get_parent_object(feat)
            if parent_obj is None:
                continue
            try:
                parent_shape = parent_obj.Shape
                if parent_shape is None or parent_shape.isNull():
                    continue
            except Exception:
                continue

            try:
                parent_com = parent_shape.CenterOfMass
            except Exception:
                bb = parent_shape.BoundBox
                parent_com = Vector(
                    (bb.XMin + bb.XMax) * 0.5,
                    (bb.YMin + bb.YMax) * 0.5,
                    (bb.ZMin + bb.ZMax) * 0.5,
                )

            cur_edges = []
            for sub in subs:
                try:
                    edge = parent_shape.getElement(sub)
                except Exception:
                    edge = None
                if edge is not None:
                    cur_edges.append((sub, edge))
            if not cur_edges:
                continue

            # Цепочка предков SMBendWall для пересчёта в плоскую систему
            ancestors = []
            cur = parent_obj
            visited = set()
            while cur is not None and self._is_smbendwall(cur):
                if id(cur) in visited:
                    break
                visited.add(id(cur))
                ancestors.append(cur)
                bo = getattr(cur, "baseObject", None)
                if bo is None:
                    break
                try:
                    cur = bo[0]
                except Exception:
                    break

            flat_edges = list(cur_edges)
            for anc in ancestors:
                info = flat_bend_info.get(anc.Name)
                if info is None:
                    continue
                fc = info["center"]
                fa = info["axis"]
                fangle = info["angle_deg"]
                flat_edges = [
                    (sub, self._rotate_edge(edge, fc, fa, +fangle))
                    for sub, edge in flat_edges
                ]

            angle_deg = _quantity_deg(getattr(feat, "angle", None), 90.0)
            radius = _quantity_mm(getattr(feat, "radius", None), 1.0)
            invert = bool(getattr(feat, "invert", False))
            direction = "down" if invert else "up"
            lengths = list(getattr(feat, "LengthList", []) or [])
            flange_lengths = []
            for lq in lengths:
                try:
                    flange_lengths.append(_quantity_mm(lq, 0.0))
                except Exception:
                    flange_lengths.append(0.0)

            is_multi = len(flat_edges) > 1
            bend_lines = []
            for k, (sub, edge) in enumerate(flat_edges):
                p1, p2 = _edge_endpoints(edge)
                if p1 is None or p2 is None:
                    continue
                axis_vec = p2 - p1
                length = axis_vec.Length
                if length < 1e-6:
                    continue
                axis_norm = Vector(axis_vec)
                axis_norm.normalize()

                body_center = None
                body_mid = None
                body_direction = None
                for csub, cedge in cur_edges:
                    if csub == sub:
                        bp1, bp2 = _edge_endpoints(cedge)
                        if bp1 is not None and bp2 is not None:
                            body_center = (bp1 + bp2) * 0.5
                            body_mid = body_center
                            delta = body_center - parent_com
                            if abs(delta.x) > abs(delta.y):
                                body_direction = (
                                    "right" if delta.x > 0 else "left")
                            else:
                                body_direction = (
                                    "up" if delta.y > 0 else "down")
                        break

                segment_id = ("{}_{}".format(feat_name, k)
                              if is_multi else feat_name)
                bend_lines.append(BendLine(
                    index=k, flat_start=p1, flat_end=p2,
                    axis_flat=axis_norm, length_mm=length,
                    sub_element=sub, segment_id=segment_id,
                    body_center=body_center,
                    body_direction=body_direction,
                    body_mid=body_mid,
                ))

            if not bend_lines:
                continue

            # --- target_com ---
            delta_solids = []
            delta_full = None
            try:
                delta_full = feat.Shape.cut(parent_shape)
                if delta_full is not None and not delta_full.isNull():
                    delta_solids = list(delta_full.Solids)
            except Exception as e:
                _unfold_diag("{}: delta cut failed: {}".format(
                    feat_name, e))

            if len(bend_lines) == 1 and delta_solids:
                try:
                    com = _safe_com(delta_full)
                    if com is not None:
                        bend_lines[0].target_com = com
                except Exception:
                    pass

            elif len(bend_lines) > 1 and delta_full is not None:
                # V12.4: tc = (bm.x, bm.y, delta_cz)
                try:
                    delta_bb = delta_full.BoundBox
                    delta_cz = (delta_bb.ZMin + delta_bb.ZMax) * 0.5
                    for ln in bend_lines:
                        bm = getattr(ln, "body_mid", None)
                        if bm is None:
                            continue
                        tc = Vector(bm.x, bm.y, delta_cz)
                        ln.target_com = tc
                except Exception as e:
                    _unfold_diag(
                        "{}: multi target_com failed: {}".format(
                            feat_name, e))

            prim = bend_lines[0]
            flat_bend_info[feat_name] = {
                "center": prim.center(),
                "axis": Vector(prim.axis_flat),
                "angle_deg": angle_deg,
                "direction": direction,
            }

            flip_sign = angle_deg < 89.9

            bf = BendFeature(
                id=feat_name,
                angle_deg=angle_deg,
                direction=direction,
                radius_mm=radius,
                invert=invert,
                parent_feature=parent_name,
                lines=bend_lines,
                flange_lengths_mm=flange_lengths,
                flip_sign=flip_sign,
            )
            features.append(bf)

        _unfold_diag("Итого BendFeature: {}".format(len(features)))
        self._features_cache = features
        return list(features)

    # ------------------------------------------------------------------
    # Вспомогательные
    # ------------------------------------------------------------------

    def _rotate_edge(self, edge, center, axis, angle_deg):
        p1, p2 = _edge_endpoints(edge)
        if p1 is None or p2 is None:
            return edge
        rp1 = _rotate_point_around_axis(p1, center, axis, angle_deg)
        rp2 = _rotate_point_around_axis(p2, center, axis, angle_deg)
        seg = Part.LineSegment(rp1, rp2)
        return seg.toShape()

    def _get_parent_object(self, feat):
        try:
            bo = getattr(feat, "baseObject", None)
            if bo is None:
                return None
            try:
                return bo[0]
            except Exception:
                return None
        except Exception:
            return None

    def _get_smbendwall_features_ordered(self, body) -> list:
        features = []
        try:
            group = list(getattr(body, "Group", []) or [])
        except Exception:
            group = []
        for o in group:
            if self._is_smbendwall(o):
                features.append(o)

        def _depth(feat):
            d = 0
            cur = feat
            visited = set()
            while True:
                if id(cur) in visited:
                    break
                visited.add(id(cur))
                bo = getattr(cur, "baseObject", None)
                if bo is None:
                    break
                try:
                    parent = bo[0] if hasattr(bo, "__getitem__") else bo
                except Exception:
                    break
                if parent is None:
                    break
                d += 1
                cur = parent
            return d

        try:
            features.sort(key=lambda f: (_depth(f),
                                          getattr(f, "Name", "")))
        except Exception:
            pass
        return features

    def _get_feat_subs(self, feat) -> list:
        try:
            bo = getattr(feat, "baseObject", None)
            if bo is None:
                return []
            try:
                subs = list(bo[1])
            except Exception:
                return []
            return [str(s) for s in subs]
        except Exception:
            return []

    def _get_parent_name(self, feat) -> str:
        try:
            bo = getattr(feat, "baseObject", None)
            if bo is None:
                return ""
            try:
                parent = bo[0]
            except Exception:
                return ""
            return str(getattr(parent, "Name", "") or "")
        except Exception:
            return ""

    def _is_smbendwall(self, obj) -> bool:
        try:
            if obj is None:
                return False
            if getattr(obj, "TypeId", "") != "PartDesign::FeaturePython":
                return False
            proxy = getattr(obj, "Proxy", None)
            cls = getattr(proxy, "__class__", None)
            if cls is None:
                return False
            return "SMBendWall" in str(cls)
        except Exception:
            return False

    def _find_body(self):
        if not _FC_OK:
            return None
        try:
            base = getattr(self.unfold_obj, "baseObject", None)
            if base and len(base) >= 1:
                cand = base[0]
                if cand is not None and hasattr(cand, "Shape"):
                    try:
                        if cand.isDerivedFrom("PartDesign::Body"):
                            return cand
                    except Exception:
                        pass
        except Exception:
            pass

        doc = App.ActiveDocument
        if doc is None:
            return None
        visited = set()

        def walk(obj):
            if obj is None or id(obj) in visited:
                return None
            visited.add(id(obj))
            try:
                if obj.isDerivedFrom("PartDesign::Body"):
                    if hasattr(obj, "Shape") and not obj.Shape.isNull():
                        return obj
            except Exception:
                pass
            for attr in ("InList", "OutList"):
                for c in getattr(obj, attr, []) or []:
                    r = walk(c)
                    if r is not None:
                        return r
            return None

        try:
            for parent in getattr(self.unfold_obj, "InList", []) or []:
                r = walk(parent)
                if r is not None:
                    return r
        except Exception:
            pass
        for o in doc.Objects:
            try:
                if o.isDerivedFrom("PartDesign::Body"):
                    if hasattr(o, "Shape") and not o.Shape.isNull():
                        return o
            except Exception:
                continue
        return None


# =====================================================================
# MatchingConfidence — hook (упрощённый; полный bend_matcher в Фазе 5)
# =====================================================================

def _compute_matching_confidence(parent_shape, edge, expected_radius):
    """Упрощённый MatchingConfidence: сравнивает радиусы соседних
    цилиндрических граней с ожидаемым.

    Возвращает dict или None. Полная версия (axis_angle_error,
    axis_offset_mm, shared midpoint) — в core/bend_matcher.py
    (добавим в Фазе 5).
    """
    try:
        # Найти цилиндры, примыкающие к edge
        for f in parent_shape.Faces or []:
            surf = getattr(f, "Surface", None)
            if surf is None or surf.__class__.__name__ != "Cylinder":
                continue
            r = float(getattr(surf, "Radius", 0.0))
            if abs(r - expected_radius) < 1e-3:
                return {
                    "radius_match": True,
                    "radius_actual": r,
                    "radius_expected": float(expected_radius),
                    "source": "simple_inline",
                }
        return {
            "radius_match": False,
            "radius_expected": float(expected_radius),
            "source": "simple_inline",
        }
    except Exception:
        return None