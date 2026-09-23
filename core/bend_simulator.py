# -*- coding: utf-8 -*-
"""
core/bend_simulator.py
Надёжная non-destructive OCC-симуляция гиба.

Публичный API:
    simulate_bend_occ(part_shape, bend_spec, kinematics=None,
                      angle_factor=1.0) -> (new_shape, transform_info)
    point_on_moving_side(point, transform_info) -> bool
    transform_bend_spec_inplace(bend_spec, transform_info) -> bool

Алгоритм simulate_bend_occ:
    1. Построить плоскость гиба (_make_bend_plane).
    2. Разрезать деталь на two half-spaces (_slice_by_plane).
    3. Классифицировать: какая часть — moving (_classify).
    4. Повернуть moving вокруг оси гиба.
    5. Слить обратно (_merge_group).
    6. Проверить сохранение объёма (_volumes_consistent).

Версия: BENDBEQ_BEND_SIMULATOR_V10_10
"""

from __future__ import annotations

from typing import List, Optional, Tuple

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

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


MODULE_VERSION = "BENDBEQ_BEND_SIMULATOR_V10_10"
if _FC_OK:
    try:
        App.Console.PrintMessage(
            "[bend_simulator] Загружена версия: {}\n".format(
                MODULE_VERSION))
    except Exception:
        pass


# =====================================================================
# Диагностика
# =====================================================================

DIR_DIAG = True
_DIR_PREFIX = "[BEND-DIR]"
_ASM_PREFIX = "[ASSEMBLY]"
_MERGE_PREFIX = "[MERGE]"
_FILT_PREFIX = "[FILTER]"
_COMP_PREFIX = "[COMPOUND]"


def _dird(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_DIR_PREFIX, msg))
    except Exception:
        pass


def _asmd(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_ASM_PREFIX, msg))
    except Exception:
        pass


def _merged(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_MERGE_PREFIX, msg))
    except Exception:
        pass


def _filtd(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_FILT_PREFIX, msg))
    except Exception:
        pass


def _compd(msg: str) -> None:
    if not DIR_DIAG or not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_COMP_PREFIX, msg))
    except Exception:
        pass


def _fmt_v(v, nd: int = 3) -> str:
    try:
        return "({:.{}f},{:.{}f},{:.{}f})".format(
            float(v.x), nd, float(v.y), nd, float(v.z), nd)
    except Exception:
        return "(?,?,?)"


# =====================================================================
# Исключение
# =====================================================================

class SliceFailure(Exception):
    """Не удалось разрезать/собрать деталь.

    Attributes:
        bend_id:  идентификатор гиба.
        details:  dict с диагностикой (attempts, ref_volume, ...).
    """

    def __init__(self, message, bend_id=None, details=None):
        super().__init__(message)
        self.bend_id = bend_id
        self.details = details or {}


# =====================================================================
# Базовые утилиты
# =====================================================================

def _normalize(v, default=None):
    """Единичный вектор из v. При v ≈ 0 — default (или +Z)."""
    if default is None:
        default = Vector(0, 0, 1) if _FC_OK else (0, 0, 1)
    if not _FC_OK:
        return default
    vec = Vector(v)
    if vec.Length < 1e-9:
        d = Vector(default)
        return d.normalize() if d.Length > 1e-9 else Vector(0, 0, 1)
    return vec.normalize()


def _make_bend_plane(center, axis, up_hint, size: float = 2000.0):
    """Плоскость гиба: нормаль — ортогонализованная up_hint к оси.

    Returns:
        (plane: Part.Face, normal: Vector)
    """
    ax = _normalize(axis, Vector(1, 0, 0))
    up = _normalize(up_hint, Vector(0, 0, 1))

    up_ortho = up - ax * up.dot(ax)
    if up_ortho.Length < 1e-9:
        fallback = (Vector(1, 0, 0)
                    if abs(ax.x) < 0.9 else Vector(0, 1, 0))
        up_ortho = ax.cross(fallback)

    normal = _normalize(up_ortho)
    x_dir = ax
    y_dir = _normalize(normal.cross(x_dir))

    half = size / 2.0
    origin = Vector(center) - x_dir * half - y_dir * half
    plane = Part.makePlane(size, size, origin, normal, x_dir)
    return plane, normal


# =====================================================================
# Допуски объёма
# =====================================================================

def _auto_volume_tol(reference_volume, bbox_diag=None):
    """Адаптивные (rel, abs) допуски объёма."""
    try:
        ref = float(reference_volume)
    except (TypeError, ValueError):
        ref = 0.0

    if bbox_diag is None or bbox_diag <= 1e-9:
        return 0.01, 1.0

    char_vol = float(bbox_diag) ** 3
    if char_vol <= 0.0:
        return 0.01, 1.0

    rel = (
        max(0.001, min(0.02, 0.01 * (ref / char_vol) ** 0.5))
        if ref > 0 else 0.01
    )
    abs_tol = max(1e-6, 1e-4 * char_vol)
    return rel, abs_tol


def _volumes_consistent(solids,
                        reference_volume,
                        rel_tol: float = 0.01,
                        abs_tol: float = 1.0) -> bool:
    """Сумма объёмов solids ≈ reference_volume (в допуске)."""
    if reference_volume <= 1e-9:
        return True
    total = sum(
        getattr(s, "Volume", 0.0)
        for s in solids if s is not None and not s.isNull()
    )
    diff = abs(total - reference_volume)
    tol = max(reference_volume * rel_tol, abs_tol)
    return diff <= tol


def _drop_tiny_fragments(solids,
                         reference_volume,
                         min_rel: float = 0.005,
                         bend_id: Optional[str] = None
                         ) -> Tuple[List, List]:
    """Убрать solids с объёмом < min_rel * reference_volume."""
    if not solids:
        return [], []

    if reference_volume <= 1e-9:
        return list(solids), []

    min_vol = float(reference_volume) * float(min_rel)
    kept = []
    dropped = []
    for s in solids:
        v = float(getattr(s, "Volume", 0.0) or 0.0)
        if v >= min_vol:
            kept.append(s)
        else:
            dropped.append(s)

    if dropped:
        vols = ", ".join("{:.2f}".format(s.Volume) for s in dropped)
        _filtd("[{}] dropped {} tiny fragments "
               "(< {:.2f} mm³ each): vols=[{}]".format(
                   bend_id, len(dropped), min_vol, vols))
    return kept, dropped


# =====================================================================
# Сборка / fuse / compound
# =====================================================================

def _make_compound(solids: List, bend_id: Optional[str] = None,
                   tag: str = ""):
    """Обернуть solids в compound."""
    try:
        shapes = [s for s in solids if s is not None and not s.isNull()]
        if not shapes:
            return None
        comp = Part.makeCompound(shapes)
        if comp is not None and not comp.isNull():
            _compd("[{}]{} compound: {} solids".format(
                bend_id, tag, len(shapes)))
            return comp
    except Exception as e:
        _compd("[{}]{} compound failed: {}".format(bend_id, tag, e))
    return None


def _merge_group(solids: List, bend_id: Optional[str] = None,
                 tag: str = ""):
    """Слить solids в один шейп.

    Порядок: fuse + removeSplitter -> sewShape -> compound.
    """
    if not solids:
        return None
    if len(solids) == 1:
        return solids[0]

    n_before = len(solids)
    vol_before = sum(
        float(getattr(s, "Volume", 0.0) or 0.0) for s in solids)

    result = solids[0]
    for s in solids[1:]:
        try:
            fused = result.fuse(s)
            if fused is not None and not fused.isNull():
                result = fused
        except Exception as e:
            _merged("[{}]{} fuse step failed: {}".format(bend_id, tag, e))

    try:
        result = result.removeSplitter()
    except Exception as e:
        _merged("[{}]{} removeSplitter failed: {}".format(
            bend_id, tag, e))

    n_after = len(result.Solids) if hasattr(result, "Solids") else 0
    vol_after = float(getattr(result, "Volume", 0.0) or 0.0)

    if n_after <= 1:
        _merged("[{}]{} merged: {} -> 1 "
                "(vol {:.2f} -> {:.2f})".format(
                    bend_id, tag, n_before, vol_before, vol_after))
        return result

    try:
        sewn = result.sewShape(0.05)
        if sewn is not None and not sewn.isNull():
            sewn_solids = list(sewn.Solids)
            if len(sewn_solids) == 1:
                _merged("[{}]{} sewed {} -> 1 "
                        "(vol {:.2f})".format(
                            bend_id, tag, n_after, vol_after))
                return sewn_solids[0]
    except Exception as e:
        _merged("[{}]{} sewShape failed: {}".format(bend_id, tag, e))

    comp = _make_compound(solids, bend_id=bend_id, tag=tag)
    if comp is not None:
        _merged("[{}]{} fuse failed ({} -> {}), "
                "wrapped in compound".format(
                    bend_id, tag, n_before, n_after))
        return comp

    _merged("[{}]{} partial merge: {} -> {} "
            "(vol {:.2f} -> {:.2f})".format(
                bend_id, tag, n_before, n_after, vol_before, vol_after))
    return result


# =====================================================================
# Полупространство / slice
# =====================================================================

def _make_half_space_solid(plane_point, normal, size: float,
                           box_center=None):
    """Построить полупространство (Solid) со стороны +normal."""
    n = _normalize(normal)
    pp = Vector(plane_point)

    if hasattr(Part, "makeHalfSpace"):
        try:
            hs = Part.makeHalfSpace(n, pp)
            if hs is not None and not hs.isNull():
                return hs
        except Exception as e:
            _dird("makeHalfSpace failed: {}".format(e))

    helper = (Vector(1, 0, 0) if abs(n.x) < 0.9 else Vector(0, 1, 0))
    x_dir = _normalize(n.cross(helper))
    y_dir = _normalize(n.cross(x_dir))

    center = Vector(box_center) if box_center is not None else pp
    offset = (center - pp).dot(n)
    center_on_plane = center - n * offset

    half = size / 2.0
    p0 = center_on_plane - x_dir * half - y_dir * half
    p1 = p0 + x_dir * size
    p2 = p1 + y_dir * size
    p3 = p0 + y_dir * size

    wire = Part.makePolygon([p0, p1, p2, p3, p0])
    face = Part.Face(wire)
    return face.extrude(n * size)


def _slice_by_plane(shape, plane, plane_normal, plane_point,
                    bend_id: Optional[str] = None) -> Tuple[List, str]:
    """Разрезать shape плоскостью. Три попытки.

    Returns:
        (solids, method_name)
    Raises:
        SliceFailure — если ни одна не дала ≥ 2 solids с сохранением объёма.
    """
    if shape is None or shape.isNull():
        raise SliceFailure(
            "Передана пустая геометрия детали.", bend_id=bend_id)

    ref_volume = shape.Volume
    attempts = []
    bbox = shape.BoundBox
    bbox_diag = bbox.DiagonalLength
    size = max(bbox_diag * 3.0, 3000.0)
    n = _normalize(plane_normal)
    pp = Vector(plane_point)
    box_center = bbox.Center

    rel_tol, abs_tol = _auto_volume_tol(ref_volume, bbox_diag)

    # --- Попытка 1: half_space_common ---
    try:
        box_pos = _make_half_space_solid(pp, n, size, box_center=box_center)
        box_neg = _make_half_space_solid(pp, n * -1.0, size,
                                          box_center=box_center)
        part_pos = shape.common(box_pos)
        part_neg = shape.common(box_neg)

        raw = []
        if part_pos and not part_pos.isNull():
            raw.extend(part_pos.Solids)
        if part_neg and not part_neg.isNull():
            raw.extend(part_neg.Solids)

        solids_all = [s for s in raw if s is not None and not s.isNull()]
        solids, _dropped = _drop_tiny_fragments(
            solids_all, ref_volume, bend_id=bend_id)

        if len(solids) >= 2 and _volumes_consistent(
                solids, ref_volume, rel_tol, abs_tol):
            return solids, "half_space_common"

        attempts.append(
            ("half_space_common", len(solids),
             sum(s.Volume for s in solids)))
    except Exception as e:
        attempts.append(("half_space_common", "exception", str(e)))

    # --- Попытка 2: generalFuse ---
    try:
        pieces, _ = shape.generalFuse([plane], 1e-6)
        raw = pieces.Solids if pieces and not pieces.isNull() else []
        solids_all = [s for s in raw if s is not None and not s.isNull()]
        solids, _ = _drop_tiny_fragments(
            solids_all, ref_volume, bend_id=bend_id)
        if len(solids) >= 2 and _volumes_consistent(
                solids, ref_volume, rel_tol, abs_tol):
            return solids, "generalFuse"
        attempts.append(
            ("generalFuse", len(solids),
             sum(s.Volume for s in solids)))
    except Exception as e:
        attempts.append(("generalFuse", "exception", str(e)))

    # --- Попытка 3: shape.split ---
    try:
        pieces = shape.split(plane)
        raw = []
        if pieces is not None:
            if hasattr(pieces, "Solids"):
                raw.extend(pieces.Solids)
            elif isinstance(pieces, (list, tuple)):
                for p in pieces:
                    if p and not p.isNull() and hasattr(p, "Solids"):
                        raw.extend(p.Solids)
        solids_all = [s for s in raw if s is not None and not s.isNull()]
        solids, _ = _drop_tiny_fragments(
            solids_all, ref_volume, bend_id=bend_id)
        if len(solids) >= 2 and _volumes_consistent(
                solids, ref_volume, rel_tol, abs_tol):
            return solids, "shape.split"
        attempts.append(
            ("shape.split", len(solids),
             sum(s.Volume for s in solids)))
    except Exception as e:
        attempts.append(("shape.split", "exception", str(e)))

    raise SliceFailure(
        "Не удалось разбить деталь плоскостью гиба ни одним из методов "
        "(ожидаемый объём {:.4f} мм³).".format(ref_volume),
        bend_id=bend_id,
        details={
            "attempts": attempts,
            "reference_volume": ref_volume,
            "bbox_diag": bbox_diag,
            "rel_tol": rel_tol,
            "abs_tol": abs_tol,
        },
    )


# =====================================================================
# Геометрия относительно плоскости
# =====================================================================

def _distance_to_plane(solid, plane_point, plane_normal) -> float:
    """Минимальное расстояние от BBox corners до плоскости.

    При пересечении плоскости — 0.0.
    """
    try:
        n = _normalize(plane_normal)
        p0 = Vector(plane_point)
        bb = solid.BoundBox
        corners = [
            Vector(bb.XMin, bb.YMin, bb.ZMin),
            Vector(bb.XMax, bb.YMax, bb.ZMax),
            Vector(bb.XMin, bb.YMin, bb.ZMax),
            Vector(bb.XMax, bb.YMax, bb.ZMin),
            Vector(bb.XMin, bb.YMax, bb.ZMin),
            Vector(bb.XMax, bb.YMin, bb.ZMax),
            Vector(bb.XMin, bb.YMax, bb.ZMax),
            Vector(bb.XMax, bb.YMin, bb.ZMin),
        ]
        dists = [(c - p0).dot(n) for c in corners]
        d_min = min(dists)
        d_max = max(dists)
        if d_min <= 0 <= d_max:
            return 0.0
        return min(abs(d_min), abs(d_max))
    except Exception:
        return float("inf")


def _side_touches_plane(solid, plane_point, plane_normal,
                        band=None, sheet_thickness=None) -> bool:
    """Примыкает ли solid к плоскости гиба в пределах band."""
    if solid is None or solid.isNull():
        return False

    n = _normalize(plane_normal)
    p0 = Vector(plane_point)

    if band is None:
        try:
            bb = solid.BoundBox
            min_dim = min(bb.XLength, bb.YLength, bb.ZLength)
            if sheet_thickness is not None and sheet_thickness > 1e-6:
                band = max(float(sheet_thickness), 0.5)
            else:
                band = max(0.5, min(1.0, min_dim * 0.5))
        except Exception:
            band = 0.5

    d = _distance_to_plane(solid, p0, n)
    if d <= band:
        return True

    try:
        plane_face = Part.makePlane(
            2.0 * band + 100.0, 2.0 * band + 100.0,
            p0 - n * band, n,
        )
        if plane_face is not None and not plane_face.isNull():
            dist_info = solid.distToShape(plane_face)
            if dist_info and len(dist_info) > 0:
                min_dist = dist_info[0]
                if min_dist is not None and min_dist <= band * 2.0:
                    return True
    except Exception:
        pass

    return False


# =====================================================================
# Классификация: fixed / moving
# =====================================================================

def _classify(solids, plane_normal, plane_point,
              preferred_side=None, bend_id: Optional[str] = None,
              diag_tag: str = "", sheet_thickness=None):
    """Разделить solids на (fixed, moving).

    Критерий: сторона с меньшим объёмом считается moving. Если объёмы
    равны — preferred_side определяет направление.
    """
    n = _normalize(plane_normal)
    p0 = Vector(plane_point)

    ref_vol = sum(
        float(getattr(s, "Volume", 0.0) or 0.0) for s in solids)
    solids, _ = _drop_tiny_fragments(
        solids, ref_vol, min_rel=0.001, bend_id=bend_id)

    pos_raw: List = []
    neg_raw: List = []
    vol_pos = 0.0
    vol_neg = 0.0

    for s in solids:
        try:
            c = s.CenterOfMass
        except Exception:
            bb = s.BoundBox
            c = Vector(
                (bb.XMin + bb.XMax) * 0.5,
                (bb.YMin + bb.YMax) * 0.5,
                (bb.ZMin + bb.ZMax) * 0.5,
            )
        d = (c - p0).dot(n)
        v = float(getattr(s, "Volume", 0.0) or 0.0)
        if d > 1e-6:
            pos_raw.append(s)
            vol_pos += v
        else:
            neg_raw.append(s)
            vol_neg += v

        _dird("{} solid: vol={:.4f} dist_to_plane={:+.4f} "
              "-> {}".format(
                  diag_tag, v, d, "POS" if d > 1e-6 else "NEG"))

    if not pos_raw or not neg_raw:
        _dird("{} classify: pos={} neg={} — пустая группа".format(
            diag_tag, len(pos_raw), len(neg_raw)))
        raise SliceFailure(
            "Классификация гиба дала пустую группу (pos или neg).",
            bend_id=bend_id,
            details={
                "pos_count": len(pos_raw),
                "neg_count": len(neg_raw),
                "pos_vol": vol_pos,
                "neg_vol": vol_neg,
            },
        )

    _dird("{} raw volumes: vol_pos={:.4f} ({} solids) "
          "vol_neg={:.4f} ({} solids)".format(
              diag_tag, vol_pos, len(pos_raw),
              vol_neg, len(neg_raw)))

    pos_merged = _merge_group(pos_raw, bend_id=bend_id, tag="[pos]")
    neg_merged = _merge_group(neg_raw, bend_id=bend_id, tag="[neg]")

    vol_pos_m = float(getattr(pos_merged, "Volume", 0.0) or 0.0)
    vol_neg_m = float(getattr(neg_merged, "Volume", 0.0) or 0.0)

    _dird("{} merged: pos vol={:.2f}, neg vol={:.2f}".format(
        diag_tag, vol_pos_m, vol_neg_m))

    if vol_pos_m < vol_neg_m:
        positive_is_moving = True
        why = "vol_pos < vol_neg"
    elif vol_neg_m < vol_pos_m:
        positive_is_moving = False
        why = "vol_neg < vol_pos"
    else:
        positive_is_moving = True
        why = "volumes equal -> default True"
        if preferred_side is not None and preferred_side.Length > 1e-9:
            dot = preferred_side.dot(n)
            if dot < 0:
                positive_is_moving = False
                why = "volumes equal -> preferred_side.dot(n)<0"
            else:
                why = "volumes equal -> preferred_side.dot(n)>=0"

    _dird("{} positive_is_moving={} ({})".format(
        diag_tag, positive_is_moving, why))

    moving_solid = pos_merged if positive_is_moving else neg_merged
    fixed_solid = neg_merged if positive_is_moving else pos_merged

    if not _side_touches_plane(
            moving_solid, p0, n, sheet_thickness=sheet_thickness):
        same_side = pos_raw if positive_is_moving else neg_raw
        best = None
        best_d = float("inf")
        for s in same_side:
            d = _distance_to_plane(s, p0, n)
            if d < best_d:
                best_d = d
                best = s
        if best is not None and best_d < 5.0:
            _dird("{} moving_solid doesn't touch plane "
                  "(d={:.3f}), using closest solid instead".format(
                      diag_tag, best_d))
            if positive_is_moving:
                moving_solid = best
                fixed_solid = neg_merged
            else:
                moving_solid = best
                fixed_solid = pos_merged
        else:
            try:
                bb = moving_solid.BoundBox
                info = (
                    "vol={:.2f} bb=({:.1f},{:.1f},{:.1f})-"
                    "({:.1f},{:.1f},{:.1f}) dist_to_plane={:.3f}"
                ).format(
                    moving_solid.Volume,
                    bb.XMin, bb.YMin, bb.ZMin,
                    bb.XMax, bb.YMax, bb.ZMax,
                    _distance_to_plane(moving_solid, p0, n))
            except Exception:
                info = "vol={:.2f}".format(
                    getattr(moving_solid, "Volume", 0))
            _dird("{} touching_moving=False; moving: {}".format(
                diag_tag, info))
            raise SliceFailure(
                "Ни один солид из подвижной группы не примыкает "
                "к линии гиба.",
                bend_id=bend_id,
                details={"moving_solid": info},
            )

    return [fixed_solid], [moving_solid], positive_is_moving


# =====================================================================
# Обрезка центра / диагностика
# =====================================================================

def _clamp_center_inside_bbox(center, bbox, eps: float = 0.1,
                              border_tol: float = 1e-4):
    """Сдвинуть центр внутрь BBox, если он на границе."""
    c = Vector(center)
    moved = False

    if abs(c.x - bbox.XMin) < border_tol:
        c.x = bbox.XMin + eps
        moved = True
    elif abs(c.x - bbox.XMax) < border_tol:
        c.x = bbox.XMax - eps
        moved = True

    if abs(c.y - bbox.YMin) < border_tol:
        c.y = bbox.YMin + eps
        moved = True
    elif abs(c.y - bbox.YMax) < border_tol:
        c.y = bbox.YMax - eps
        moved = True

    if abs(c.z - bbox.ZMin) < border_tol:
        c.z = bbox.ZMin + eps
        moved = True
    elif abs(c.z - bbox.ZMax) < border_tol:
        c.z = bbox.ZMax - eps
        moved = True

    return c, moved


def _log_assembly(result, bend_id: Optional[str]) -> None:
    """Логировать сборку: число solids, volume, isValid, BBox каждого."""
    try:
        n_solids = len(result.Solids)
        vol = float(result.Volume)
        is_valid = bool(result.isValid())
        _asmd("[{}] solids={}, volume={:.4f}, isValid={}".format(
            bend_id, n_solids, vol, is_valid))
        if n_solids > 1:
            for i, s in enumerate(result.Solids):
                try:
                    bb = s.BoundBox
                    _asmd("[{}]   solid[{}]: vol={:.4f} "
                          "center={} size=({:.2f},{:.2f},{:.2f})".format(
                              bend_id, i, s.Volume, _fmt_v(bb.Center),
                              bb.XLength, bb.YLength, bb.ZLength))
                except Exception as e:
                    _asmd("[{}]   solid[{}]: error {}".format(
                        bend_id, i, e))
    except Exception as e:
        _asmd("[{}] log_assembly failed: {}".format(bend_id, e))


# =====================================================================
# Толщина листа
# =====================================================================

def _guess_sheet_thickness(part_shape) -> Optional[float]:
    """Толщина листа из SEQUENCE_CONFIG["sheet_thickness_mm"].

    Fallback: min BBox Pad'а — работает только на плоской детали.
    """
    try:
        cfg_t = float(SEQUENCE_CONFIG.get("sheet_thickness_mm", 0.0))
        if 0.1 < cfg_t < 50.0:
            return cfg_t
    except (TypeError, ValueError):
        pass

    if part_shape is None or part_shape.isNull():
        return None
    try:
        bb = part_shape.BoundBox
        dims = sorted([bb.XLength, bb.YLength, bb.ZLength])
        t = float(dims[0])
        if 0.1 < t < 50.0:
            return t
    except Exception:
        pass
    return None


# =====================================================================
# Публичный API
# =====================================================================

def simulate_bend_occ(part_shape, bend_spec, kinematics=None,
                      angle_factor: float = 1.0):
    """Симулировать один гиб в OCC. Non-destructive.

    Args:
        part_shape:    Part.Shape до гиба.
        bend_spec:     объект с .id, .center, .axis, .angle, .direction,
                       .up_hint, .normal.
        kinematics:    BendKinematics | None — источник preferred_side.
        angle_factor:  1.0 — полный угол; -1.0 — обратный (для reverse
                       валидации); 0.5 — момент гиба.

    Returns:
        (new_shape, transform_info).

    Raises:
        SliceFailure — при неудаче разрезания/сборки или потере объёма.
    """
    if not _FC_OK:
        raise SliceFailure(
            "simulate_bend_occ требует FreeCAD.Part",
            bend_id=getattr(bend_spec, "id", None))

    if part_shape is None or part_shape.isNull():
        return None, None

    bend_id = getattr(bend_spec, "id", None) or "?"
    center_in = Vector(getattr(bend_spec, "center"))
    axis = _normalize(getattr(bend_spec, "axis"))
    angle_full = float(getattr(bend_spec, "angle", 90.0))
    angle = angle_full * float(angle_factor)
    up_hint = _normalize(
        getattr(bend_spec, "up_hint", Vector(0, 0, 1)))
    direction = str(getattr(bend_spec, "direction", "up")).lower()
    sketch_normal = _normalize(
        getattr(bend_spec, "normal", Vector(0, 0, 1)))
    kin_tag = "kin=YES" if kinematics is not None else "kin=None"

    _dird("=== BEND-DIR DUMP BEGIN ===")
    _dird("step={} {} factor={} angle={:.3f} dir={}".format(
        bend_id, kin_tag, angle_factor, angle, direction))
    _dird("center={} axis={} up_hint={} sk_n={}".format(
        _fmt_v(center_in), _fmt_v(axis),
        _fmt_v(up_hint), _fmt_v(sketch_normal)))

    bb_shape = part_shape.BoundBox
    center, _moved = _clamp_center_inside_bbox(center_in, bb_shape)
    if _moved:
        _dird("center clamped: {} -> {}".format(
            _fmt_v(center_in), _fmt_v(center)))

    size = max(part_shape.BoundBox.DiagonalLength * 2.5, 2000.0)
    plane, plane_normal = _make_bend_plane(center, axis, up_hint, size)
    _dird("plane_normal={} size={:.1f}".format(
        _fmt_v(plane_normal), size))

    t_guess = _guess_sheet_thickness(part_shape)
    if t_guess is not None:
        _dird("sheet_thickness guess={:.3f}".format(t_guess))

    # --- Разрезка ---
    work = part_shape.copy()
    solids, method = _slice_by_plane(
        work, plane, plane_normal, center, bend_id=bend_id)
    vols = ", ".join("{:.2f}".format(s.Volume) for s in solids)
    _dird("slice method={} solids={} vols=[{}]".format(
        method, len(solids), vols))

    # --- Preferred side ---
    preferred = None
    preferred_src = "none"
    if kinematics is not None and hasattr(kinematics,
                                           "get_moving_direction"):
        try:
            preferred = kinematics.get_moving_direction(bend_spec)
            preferred_src = "kinematics"
        except Exception:
            preferred = None

    if preferred is None or preferred.Length < 1e-9:
        try:
            cm = work.CenterOfMass
        except Exception:
            cm = work.BoundBox.Center
        preferred = cm - center
        preferred = preferred - axis * preferred.dot(axis)
        if preferred.Length < 1e-9:
            preferred = plane_normal
            preferred_src = "fallback plane_normal"
        else:
            preferred.normalize()
            preferred_src = "center_of_mass heuristic"

    _dird("preferred_side={} (from {})".format(
        _fmt_v(preferred), preferred_src))

    # --- Классификация ---
    fixed, moving, positive_is_moving = _classify(
        solids, plane_normal, center, preferred,
        bend_id=bend_id, diag_tag="[{}]".format(bend_id),
        sheet_thickness=t_guess,
    )

    # --- Знак поворота ---
    moving_normal = (
        plane_normal if positive_is_moving else plane_normal * -1.0)
    disp_dir = axis.cross(moving_normal)
    up_dir = (
        sketch_normal if direction == "up" else sketch_normal * -1.0)
    disp_dot = disp_dir.dot(up_dir)
    if disp_dot > 0:
        signed_angle = angle
        sign_why = "disp_dir.dot(up_dir)={:+.4f} > 0".format(disp_dot)
    else:
        signed_angle = -angle
        sign_why = "disp_dir.dot(up_dir)={:+.4f} <= 0".format(disp_dot)

    _dird("moving_normal={} disp_dir={} up_dir={}".format(
        _fmt_v(moving_normal), _fmt_v(disp_dir), _fmt_v(up_dir)))
    _dird("signed_angle={:+.3f}° ({}) positive_is_moving={}".format(
        signed_angle, sign_why, positive_is_moving))

    # --- Поворот moving ---
    rotated = []
    for s in moving:
        rs = s.copy()
        rs.translate(-center)
        rs.rotate(Vector(0, 0, 0), axis, signed_angle)
        rs.translate(center)
        rotated.append(rs)

    # --- Сборка ---
    all_parts = fixed + rotated
    result = _merge_group(all_parts, bend_id=bend_id, tag="[final]")

    if result is None or result.isNull():
        raise SliceFailure(
            "Сборка согнутой детали вернула пустой шейп.",
            bend_id=bend_id,
        )

    try:
        result = result.removeSplitter()
    except Exception as e:
        _dird("removeSplitter failed: {}".format(e))

    _log_assembly(result, bend_id)

    # --- Проверка объёма ---
    ref_vol = (float(part_shape.Volume)
               if part_shape.Volume > 1e-9 else 1.0)
    got_vol = float(getattr(result, "Volume", 0.0) or 0.0)
    loss_rel = abs(got_vol - ref_vol) / ref_vol

    if float(angle_factor) < 0.0:
        max_loss = float(SEQUENCE_CONFIG.get(
            "volume_loss_max_rel_reverse", 0.05))
        mode_tag = "reverse"
    else:
        max_loss = float(SEQUENCE_CONFIG.get(
            "volume_loss_max_rel", 0.02))
        mode_tag = "forward"

    _dird("post-fuse vol={:.4f} (ref={:.4f}) loss={:.4f}% "
          "({}, max={:.2f}%)".format(
              got_vol, ref_vol, loss_rel * 100,
              mode_tag, max_loss * 100))

    if loss_rel > max_loss:
        raise SliceFailure(
            "Объём после гиба отличается на {:.3f} % "
            "({:.2f} vs {:.2f} мм³, порог {:.1f} %, режим {}).".format(
                loss_rel * 100, got_vol, ref_vol,
                max_loss * 100, mode_tag),
            bend_id=bend_id,
            details={
                "loss_rel": loss_rel,
                "got": got_vol,
                "ref": ref_vol,
                "max_rel": max_loss,
                "mode": mode_tag,
            },
        )

    transform_info = {
        "plane_point": Vector(center),
        "plane_normal": _normalize(plane_normal),
        "positive_is_moving": positive_is_moving,
        "rotate_center": Vector(center),
        "rotate_axis": _normalize(axis),
        "rotate_angle": float(signed_angle),
        "force_ids": set(),
    }

    _dird("result vol={:.4f} (ref={:.4f}) rotate_angle={:+.3f}".format(
        result.Volume, part_shape.Volume, signed_angle))
    _dird("=== BEND-DIR DUMP END ===")

    return result, transform_info


# =====================================================================
# Утилиты для transform_info
# =====================================================================

def point_on_moving_side(point, transform_info) -> bool:
    """Лежит ли point на стороне moving относительно плоскости гиба."""
    n = _normalize(transform_info["plane_normal"])
    p0 = Vector(transform_info["plane_point"])
    d = (Vector(point) - p0).dot(n)
    return (
        (d > 1e-6)
        if transform_info.get("positive_is_moving", True)
        else (d <= 1e-6)
    )


def transform_bend_spec_inplace(bend_spec, transform_info) -> bool:
    """Сдвинуть bend_spec в новую систему координат после гиба.

    Если bend_spec.center на moving-стороне — повернуть его
    (center, axis, up_hint, normal) вокруг оси гиба на rotate_angle.

    force_ids из transform_info["force_ids"] заставляет трансформировать
    независимо от положения центра (для corner flanges).

    Returns:
        True, если трансформация применена.
    """
    if bend_spec is None or transform_info is None:
        return False
    if not _FC_OK:
        return False

    bend_id = getattr(bend_spec, "id", "?")
    center = Vector(getattr(bend_spec, "center"))

    force_ids = transform_info.get("force_ids") or set()
    forced = bend_id in force_ids

    on_moving = forced or point_on_moving_side(center, transform_info)
    _dird("transform_bend_spec_inplace [{}]: on_moving_side={}{}".format(
        bend_id, on_moving, " (forced)" if forced else ""))
    if not on_moving:
        return False

    rc = Vector(transform_info["rotate_center"])
    ra = _normalize(transform_info["rotate_axis"], Vector(1, 0, 0))
    angle = float(transform_info["rotate_angle"])

    rot = App.Rotation(ra, angle)

    bend_spec.center = rc + rot.multVec(center - rc)
    bend_spec.axis = _normalize(
        rot.multVec(Vector(getattr(bend_spec, "axis", Vector(1, 0, 0)))))
    bend_spec.up_hint = _normalize(
        rot.multVec(Vector(getattr(bend_spec, "up_hint", Vector(0, 0, 1)))))

    if hasattr(bend_spec, "normal") and bend_spec.normal is not None:
        bend_spec.normal = _normalize(
            rot.multVec(Vector(bend_spec.normal)))

    _dird("transform_bend_spec_inplace [{}]: transformed "
          "center={} axis={}".format(
              bend_id, _fmt_v(bend_spec.center), _fmt_v(bend_spec.axis)))

    return True