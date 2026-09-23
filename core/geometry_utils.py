# -*- coding: utf-8 -*-
"""
core/geometry_utils.py
Утилиты получения геометрии в мировых координатах.

Используется адаптерами оснастки (ToolAdapter, PressGeometryAdapter),
чтобы получать форму с учётом Placement объекта, а не локальные
координаты. Без этого коллизии считались бы в неправильной системе.

Перенесено из freecad_sheetmetal_sequence (core/geometry_utils.py).
Логика без изменений.

Версия: BENDBEQ_GEOMETRY_UTILS_V1
"""

from __future__ import annotations

from typing import Optional

try:
    import FreeCAD as App
    import Part
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    _FC_OK = False


def shape_in_world(obj) -> Optional["Part.Shape"]:
    """Копия obj.Shape с применённым getGlobalPlacement().

    Возвращает None, если:
        * obj is None или у него нет .Shape;
        * Shape пустой/Null;
        * FreeCAD не доступен.

    Примечания:
        * Если у объекта нет getGlobalPlacement(), используется
          локальный Placement.
        * Если и Placement недоступен — возвращается копия шейпа
          без трансформации.
        * Любая ошибка трансформации логируется через PrintWarning,
          но не роняет вызывающий код (возвращается нетрансформированная
          копия).
    """
    if not _FC_OK:
        return None
    if obj is None or not hasattr(obj, "Shape"):
        return None
    try:
        if obj.Shape is None or obj.Shape.isNull():
            return None
    except Exception:
        return None

    shape = obj.Shape.copy()
    try:
        if hasattr(obj, "getGlobalPlacement"):
            pl = obj.getGlobalPlacement()
        elif hasattr(obj, "Placement"):
            pl = obj.Placement
        else:
            pl = None
        if pl is not None:
            shape.transformGeometry(pl.toMatrix())
    except Exception as e:
        if App is not None:
            try:
                App.Console.PrintWarning(
                    "[shape_in_world] {}: {}\n".format(
                        getattr(obj, "Name", "?"), e))
            except Exception:
                pass
    return shape


def bbox_of(obj) -> Optional[tuple]:
    """BBox объекта в мировых координатах: (XMin, XMax, YMin, YMax, ZMin, ZMax).

    Удобно для AABB-предфильтра в collision_engine.
    """
    shape = shape_in_world(obj)
    if shape is None:
        return None
    try:
        bb = shape.BoundBox
        return (bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax)
    except Exception:
        return None