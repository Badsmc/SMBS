# -*- coding: utf-8 -*-
"""
core/panel_solid.py
Тонкие Solid-панели в мировых координатах.

Единый источник правды для:
  - collision_engine (AABB-фильтр и narrow-phase common);
  - sequence_state / backward search (is_free_fn);
  - simulation_viewer (moment mode, kinematic preview).

Модуль НЕ импортирует FreeCADGui. Работает и в headless.

Терминология:
  p2d       — PanelNode2D из core/panel_graph_2d.py
              (имеет exterior_2d: [(x, y), ...] и cell_z: float)
  T_p       — 4×4 SE(3) матрица (list[list[float]]) из FrozensetSequenceState
  thickness — толщина листа в мм

Версия: BENDBEQ_PANEL_SOLID_V1
"""

from __future__ import annotations
from typing import Iterable, Optional, Sequence, Tuple

# --- FreeCAD Part: soft import (headless-safe) ---
try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector, Matrix
    PART_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    Vector = None
    Matrix = None
    PART_OK = False

U4 = Sequence[Sequence[float]]


# =====================================================================
# Внутренние помощники
# =====================================================================

def _matrix_from_u4(T_p: U4) -> "Matrix":
    """Собрать FreeCAD.Matrix из U4 (row-major 4×4)."""
    m = T_p
    return Matrix(
        float(m[0][0]), float(m[0][1]), float(m[0][2]), float(m[0][3]),
        float(m[1][0]), float(m[1][1]), float(m[1][2]), float(m[1][3]),
        float(m[2][0]), float(m[2][1]), float(m[2][2]), float(m[2][3]),
        0.0,            0.0,            0.0,            1.0,
    )


def _identity_u4() -> list:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _is_identity_u4(T_p: U4, eps: float = 1e-12) -> bool:
    for i in range(4):
        for j in range(4):
            tgt = 1.0 if i == j else 0.0
            if abs(float(T_p[i][j]) - tgt) > eps:
                return False
    return True


def _clean_polygon_2d(exterior: Iterable[Tuple[float, float]]):
    """Снять дубликаты вершин и замкнуть, если нужно."""
    pts = [(float(x), float(y)) for (x, y) in exterior]
    if len(pts) >= 2 and (
            abs(pts[-1][0] - pts[0][0]) < 1e-9
            and abs(pts[-1][1] - pts[0][1]) < 1e-9):
        pts = pts[:-1]
    # дедуп соседних одинаковых
    out = []
    for p in pts:
        if not out or (abs(p[0] - out[-1][0]) > 1e-9
                       or abs(p[1] - out[-1][1]) > 1e-9):
            out.append(p)
    return out


def _apply_matrix(solid: "Part.Shape", T_p: U4) -> Optional["Part.Shape"]:
    """Применить T_p к копии Solid'а. Не мутирует исходный."""
    if solid is None:
        return None
    mat = _matrix_from_u4(T_p)
    try:
        copy = solid.copy()
        copy.transformShape(mat)
        return copy
    except Exception:
        try:
            return solid.transformGeometry(mat)
        except Exception:
            return None


# =====================================================================
# Публичный API
# =====================================================================

def panel_solid_world(p2d,
                      T_p: Optional[U4] = None,
                      thickness: float = 1.5) -> Optional["Part.Shape"]:
    """Тонкий Part.Solid панели из PanelNode2D в мировых координатах.

    Args:
        p2d:       PanelNode2D с exterior_2d и cell_z
        T_p:       4×4 SE(3) матрица из FrozensetSequenceState; None ≡ I
        thickness: толщина листа в мм

    Returns:
        Part.Shape (Solid) или None при ошибке / невалидной геометрии.

    Схема:
        1. Полигон exterior_2d поднимается в Z = cell_z.
        2. Extrude на +thickness вдоль Z.
        3. Применяется T_p (если задан и не I).
    """
    if not PART_OK:
        return None
    if p2d is None:
        return None

    ext = getattr(p2d, "exterior_2d", None)
    if not ext:
        return None

    pts = _clean_polygon_2d(ext)
    if len(pts) < 3:
        return None

    cell_z = float(getattr(p2d, "cell_z", 0.0))
    thk = float(thickness)
    if thk <= 0.0:
        return None

    try:
        vpts = [Vector(x, y, cell_z) for (x, y) in pts]
        vpts.append(vpts[0])
        wire = Part.makePolygon(vpts)
        face = Part.Face(wire)
        solid = face.extrude(Vector(0.0, 0.0, thk))
    except Exception:
        return None

    if T_p is None or _is_identity_u4(T_p):
        return solid

    return _apply_matrix(solid, T_p)


def panel_solid_from_face(face,
                          extrude_vec,
                          T_p: Optional[U4] = None) -> Optional["Part.Shape"]:
    """Тонкий Part.Solid из произвольной Part.Face с явным extrude.

    Используется на 3D-пути (SM): берём грань панели и вытягиваем на
    толщину внутрь материала.

    Args:
        face:        Part.Face (репрезентативная грань панели в Body)
        extrude_vec: FreeCAD.Vector — направление и длина экструзии
                     (обычно -outward_normal * thickness)
        T_p:         опциональная матрица (если нужно сразу трансформировать)

    Returns:
        Part.Shape или None.
    """
    if not PART_OK:
        return None
    if face is None or extrude_vec is None:
        return None
    try:
        solid = face.extrude(extrude_vec)
    except Exception:
        return None

    if solid is None or solid.isNull():
        return None

    # face.extrude может вернуть Shell / Compound — попробуем починить
    st = getattr(solid, "ShapeType", "")
    if st != "Solid":
        try:
            if st == "Shell":
                solid = Part.makeSolid(solid)
            elif st == "Compound":
                inner = list(solid.Solids)
                if inner:
                    solid = inner[0]
        except Exception:
            pass

    if T_p is None or _is_identity_u4(T_p):
        return solid

    return _apply_matrix(solid, T_p)


def transform_solid(solid: "Part.Shape",
                    T_p: U4) -> Optional["Part.Shape"]:
    """Применить U4 к уже построенному Solid'у (копия).

    Отдельная функция нужна, чтобы collision checker мог кэшировать
    «плоские» Solid'ы панелей и потом быстро применять к ним разные T_p.
    """
    if not PART_OK or solid is None:
        return None
    if T_p is None or _is_identity_u4(T_p):
        return solid.copy() if hasattr(solid, "copy") else solid
    return _apply_matrix(solid, T_p)


# =====================================================================
# Диагностика (для probe / аудита)
# =====================================================================

def describe_solid(solid: "Part.Shape") -> dict:
    """Сводка по Solid'у — безопасно, без падений."""
    if not PART_OK or solid is None:
        return {"ok": False}
    try:
        if solid.isNull():
            return {"ok": False, "null": True}
    except Exception:
        return {"ok": False, "error": "isNull_failed"}

    info = {"ok": True}
    try:
        info["shape_type"] = str(getattr(solid, "ShapeType", "?"))
    except Exception:
        info["shape_type"] = "?"
    try:
        info["volume"] = float(solid.Volume)
    except Exception:
        info["volume"] = None
    try:
        info["area"] = float(solid.Area)
    except Exception:
        info["area"] = None
    try:
        info["solids"] = len(solid.Solids)
        info["faces"] = len(solid.Faces)
        info["edges"] = len(solid.Edges)
    except Exception:
        pass
    try:
        bb = solid.BoundBox
        info["bbox"] = {
            "min": [bb.XMin, bb.YMin, bb.ZMin],
            "max": [bb.XMax, bb.YMax, bb.ZMax],
        }
    except Exception:
        info["bbox"] = None
    return info