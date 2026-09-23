# -*- coding: utf-8 -*-
"""
core/bend_matcher.py
Матчинг PhysicalBend ↔ цилиндрические грани по радиусу, оси, соседям.

Публичный API:
    extract_cylinders(body_shape, radius_mm) -> List[dict]
    match_physical_bend(pb, inner_cyls, outer_cyls, R_in, thk) -> dict | None
    match_all(bends, body_shape, R_in, R_out, thk) -> (inner, outer)
    match_edge_to_cylinder(parent_shape, edge, radius_mm) -> dict | None

Headless-safe: без FreeCAD.Part функции возвращают [] / None.

Версия: BENDBEQ_BEND_MATCHER_V1
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

try:
    import FreeCAD as App
    import Part
    PART_OK = True
except ImportError:
    App = None
    Part = None
    PART_OK = False

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


AXIS_PARALLEL_DOT_MIN = 0.99
AXIS_OFFSET_TOLERANCE_FACTOR = 2.0
AXIS_OFFSET_TOLERANCE_MIN_MM = 1.0
RADIUS_MATCH_EPS = 1e-3


def _vec3(v):
    if v is None:
        return None
    try:
        return (float(v.x), float(v.y), float(v.z))
    except (AttributeError, TypeError):
        pass
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except (TypeError, IndexError, ValueError):
        return None


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _length(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _normalize(a):
    L = _length(a)
    if L < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / L, a[1] / L, a[2] / L)


def _perp_distance(point, line_point, line_dir_norm):
    v = _sub(point, line_point)
    along = _dot(v, line_dir_norm)
    perp = (v[0] - line_dir_norm[0] * along,
            v[1] - line_dir_norm[1] * along,
            v[2] - line_dir_norm[2] * along)
    return _length(perp)


def _axis_offset_tolerance(radius_mm: float, thickness_mm: float) -> float:
    theoretical = math.sqrt(radius_mm * radius_mm
                            + (radius_mm + thickness_mm) ** 2)
    return max(AXIS_OFFSET_TOLERANCE_MIN_MM,
               AXIS_OFFSET_TOLERANCE_FACTOR * theoretical)


def extract_cylinders(body_shape,
                       radius_mm: float) -> List[Dict[str, Any]]:
    if not PART_OK or body_shape is None:
        return []

    result = []
    faces = getattr(body_shape, "Faces", None) or []
    for i, face in enumerate(faces):
        surf = getattr(face, "Surface", None)
        if surf is None:
            continue
        if surf.__class__.__name__ != "Cylinder":
            continue
        try:
            r = float(surf.Radius)
        except (TypeError, ValueError):
            continue
        if abs(r - radius_mm) > RADIUS_MATCH_EPS:
            continue

        axis_dir = _vec3(getattr(surf, "Axis", None))
        axis_center = _vec3(getattr(surf, "Center", None))
        if axis_dir is None or axis_center is None:
            continue
        if _length(axis_dir) < 1e-9:
            continue
        axis_dir = _normalize(axis_dir)

        result.append({
            "face_index": i,
            "radius_mm": r,
            "axis_direction": axis_dir,
            "axis_center": axis_center,
            "adjacent_planars": _adjacent_planars(body_shape, face, i),
        })
    return result


def _plane_normal_axis(surf):
    try:
        ax = getattr(surf, "Axis", None)
        if ax is None:
            return None
        v = _vec3(ax)
        if v is None or _length(v) < 1e-9:
            return None
        return _normalize(v)
    except Exception:
        return None


def _adjacent_planars(body_shape, cyl_face,
                       cyl_index: int) -> List[Dict[str, Any]]:
    if not PART_OK or body_shape is None or cyl_face is None:
        return []

    result = []
    seen = set()
    faces = getattr(body_shape, "Faces", None) or []
    cyl_edges = getattr(cyl_face, "Edges", None) or []

    for edge in cyl_edges:
        for j, other in enumerate(faces):
            if j == cyl_index or j in seen:
                continue
            other_surf = getattr(other, "Surface", None)
            if other_surf is None:
                continue
            if other_surf.__class__.__name__ != "Plane":
                continue
            shared = False
            for oe in getattr(other, "Edges", None) or []:
                try:
                    if oe.isSame(edge):
                        shared = True
                        break
                except Exception:
                    continue
            if not shared:
                continue
            seen.add(j)

            normal = _plane_normal_axis(other_surf)
            if normal is None:
                continue
            com = _vec3(getattr(other, "CenterOfMass", None))
            if com is None:
                continue
            try:
                area = float(getattr(other, "Area", 0.0))
            except (TypeError, ValueError):
                area = 0.0

            result.append({
                "face_index": j,
                "kind": "Plane",
                "area_mm2": area,
                "center_of_mass": com,
                "normal": normal,
                "orientation": str(getattr(other, "Orientation", "Unknown")),
            })
    return result


def _find_best_cylinder(cylinders, edge_dir_norm, edge_mid, tolerance_mm):
    rejected = []
    candidates = []
    for cyl in cylinders:
        dot = abs(_dot(cyl["axis_direction"], edge_dir_norm))
        if dot < AXIS_PARALLEL_DOT_MIN:
            rejected.append({
                "face_index": cyl["face_index"],
                "axis_offset_mm": None,
                "reason": "axis_not_parallel",
            })
            continue
        offset = _perp_distance(
            cyl["axis_center"], edge_mid, edge_dir_norm)
        if offset > tolerance_mm:
            rejected.append({
                "face_index": cyl["face_index"],
                "axis_offset_mm": offset,
                "reason": "axis_offset_too_large",
            })
            continue
        candidates.append((offset, cyl))

    if not candidates:
        return None, rejected
    candidates.sort(key=lambda t: t[0])
    return candidates[0][1], rejected


def match_physical_bend(pb, inner_cylinders, outer_cylinders,
                         radius_inner_mm: float,
                         thickness_mm: float) -> Optional[Dict[str, Any]]:
    if pb is None:
        return None
    eg = getattr(pb, "edge_geometry", None)
    if eg is None:
        return None

    edge_dir = _vec3(getattr(eg, "direction", None))
    edge_mid = _vec3(getattr(eg, "midpoint", None))
    if edge_dir is None or edge_mid is None:
        return None
    edge_dir = _normalize(edge_dir)

    tol = _axis_offset_tolerance(radius_inner_mm, thickness_mm)

    inner, rej_in = _find_best_cylinder(
        inner_cylinders, edge_dir, edge_mid, tol)
    outer, rej_out = _find_best_cylinder(
        outer_cylinders, edge_dir, edge_mid, tol)
    rejected = rej_in + rej_out

    result: Dict[str, Any] = {
        "inner_face_index": None,
        "outer_face_index": None,
        "confidence": None,
        "rejected": rejected,
    }

    if inner is None:
        if outer is not None:
            result["outer_face_index"] = outer["face_index"]
        return result

    result["inner_face_index"] = inner["face_index"]
    if outer is not None:
        result["outer_face_index"] = outer["face_index"]

    dot_abs = min(1.0, abs(_dot(inner["axis_direction"], edge_dir)))
    axis_angle_error = math.acos(dot_abs)
    axis_offset = _perp_distance(
        inner["axis_center"], edge_mid, edge_dir)

    result["confidence"] = {
        "axis_angle_error_rad": axis_angle_error,
        "axis_offset_mm": axis_offset,
        "radius_match": (abs(inner["radius_mm"] - radius_inner_mm)
                          < RADIUS_MATCH_EPS),
        "shared_midpoint": False,
    }
    return result


def match_all(bends, body_shape,
              radius_inner_mm, radius_outer_mm, thickness_mm
              ) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    inner = extract_cylinders(body_shape, radius_inner_mm)
    outer = extract_cylinders(body_shape, radius_outer_mm)
    for pb in bends or []:
        info = match_physical_bend(
            pb, inner, outer, radius_inner_mm, thickness_mm)
        if info is not None:
            try:
                setattr(pb, "_matching_info", info)
            except Exception:
                pass
    return inner, outer


def match_edge_to_cylinder(parent_shape, edge,
                            expected_radius: float
                            ) -> Optional[Dict[str, Any]]:
    if not PART_OK or parent_shape is None or edge is None:
        return None

    try:
        verts = getattr(edge, "Vertexes", None) or []
        if len(verts) >= 2:
            p1 = _vec3(verts[0].Point)
            p2 = _vec3(verts[-1].Point)
        else:
            p1 = _vec3(getattr(edge, "StartPoint", None))
            p2 = _vec3(getattr(edge, "EndPoint", None))
    except Exception:
        return None

    if p1 is None or p2 is None:
        return None
    mid = ((p1[0] + p2[0]) * 0.5,
           (p1[1] + p2[1]) * 0.5,
           (p1[2] + p2[2]) * 0.5)

    best = None
    best_offset = float("inf")
    for f in parent_shape.Faces or []:
        surf = getattr(f, "Surface", None)
        if surf is None or surf.__class__.__name__ != "Cylinder":
            continue
        try:
            r = float(surf.Radius)
        except (TypeError, ValueError):
            continue
        if abs(r - expected_radius) > RADIUS_MATCH_EPS:
            continue
        axis_center = _vec3(getattr(surf, "Center", None))
        axis_dir = _vec3(getattr(surf, "Axis", None))
        if axis_center is None or axis_dir is None:
            continue
        axis_dir = _normalize(axis_dir)
        offset = _perp_distance(axis_center, mid, axis_dir)
        if offset < best_offset:
            best_offset = offset
            best = {
                "radius_match": True,
                "radius_actual": r,
                "radius_expected": float(expected_radius),
                "axis_offset_mm": offset,
                "source": "bend_matcher.match_edge_to_cylinder",
            }

    if best is None:
        return {
            "radius_match": False,
            "radius_expected": float(expected_radius),
            "source": "bend_matcher.match_edge_to_cylinder",
        }
    return best
