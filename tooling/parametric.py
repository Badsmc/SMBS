# -*- coding: utf-8 -*-
"""Построение Part.Shape по параметрам спецификации.

Ориентация:
    X — толщина
    Y — ДЛИНА (вдоль линии гиба), extrude
    Z — ВЫСОТА (носик внизу, хвостовик вверху)
"""
from __future__ import annotations
import math
from typing import List

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector
    _FC_OK = True
except ImportError:
    _FC_OK = False


DEFAULT_LENGTH = 500.0


def _extrude_xz(pts: List["Vector"], length: float, name: str):
    """Замкнуть контур в плоскости XZ, extrude по +Y."""
    if len(pts) >= 2 and (pts[0] - pts[-1]).Length > 1e-6:
        pts = list(pts) + [pts[0]]
    wire = Part.makePolygon(pts)
    face = Part.Face(wire)
    solid = face.extrude(Vector(0, length, 0))
    return solid


def build_punch(spec, length: float = DEFAULT_LENGTH):
    """Профиль пуансона. Носик снизу (Z=0), хвостовик сверху."""
    if not _FC_OK:
        return None

    h = float(spec.height)
    bw = float(spec.body_width)
    tw = float(spec.tip_width)
    th = float(spec.tip_height)
    prof = str(getattr(spec, "profile", "brick")).lower()

    half_bw = bw / 2.0
    half_tw = tw / 2.0
    th = max(0.1, min(th, h - 0.5))

    if prof == "gooseneck":
        # Односторонний relief
        side = str(getattr(spec, "gooseneck_side", "right")).lower()
        sx = 1.0 if side == "right" else -1.0
        pts = [
            Vector(-sx * half_tw, 0, 0),
            Vector( sx * half_tw, 0, 0),
            Vector( sx * half_bw, 0, th),
            Vector( sx * half_bw, 0, h),
            Vector(-sx * half_bw, 0, h),
            Vector(-sx * half_bw, 0, th),
        ]
    else:
        # brick: прямоугольник тела + трапеция носика
        pts = [
            Vector(-half_tw, 0, 0),
            Vector( half_tw, 0, 0),
            Vector( half_bw, 0, th),
            Vector( half_bw, 0, h),
            Vector(-half_bw, 0, h),
            Vector(-half_bw, 0, th),
        ]

    solid = _extrude_xz(pts, length, spec.id)
    return solid


def build_die(spec, length: float = DEFAULT_LENGTH):
    """V-матрица. Ручьём вверх (Z=height)."""
    if not _FC_OK:
        return None

    W = float(spec.width)
    H = float(spec.height)
    Vw = float(spec.opening)
    alpha = float(spec.angle)

    half_alpha = alpha / 2.0
    half_v = Vw / 2.0

    if half_alpha < 0.5:
        depth = min(Vw, 0.5 * H)
    else:
        depth = half_v / math.tan(math.radians(half_alpha))
    depth = min(depth, 0.9 * H)

    half_v_bot = half_v - depth * math.tan(math.radians(half_alpha))
    half_v_bot = max(half_v_bot, 0.3)

    pts = [
        Vector(-W / 2, 0, 0),
        Vector( W / 2, 0, 0),
        Vector( W / 2, 0, H),
        Vector( half_v, 0, H),
        Vector( half_v_bot, 0, H - depth),
        Vector(-half_v_bot, 0, H - depth),
        Vector(-half_v, 0, H),
        Vector(-W / 2, 0, H),
    ]
    return _extrude_xz(pts, length, spec.id)