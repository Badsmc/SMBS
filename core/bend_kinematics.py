# -*- coding: utf-8 -*-
"""
core/bend_kinematics.py
Независимый движок кинематики гибки (SM).

Задача: определить, какая сторона листа двигается при гибе
(punch сдвигает край, backgauge держит деталь), и вернуть вектор
этого движения в системе Body. Используется как hint в
SequenceValidator для классификации moving/fixed.

Публичный API:
    BendKinematics(operator_side, prefer_moving)
    BendKinematics.from_config()
    .is_configured() -> bool
    .get_moving_direction(bend_spec) -> Vector | None
    .get_rotation_sign(bend_spec) -> ±1 | None

Headless-friendly: если FreeCAD недоступен, operator_side хранится
как tuple, а get_moving_direction возвращает None (без геометрии).
from_config() работает через config, get_rotation_sign — чистый Python.

Перенесено из freecad_sheetmetal_sequence (core/bend_kinematics.py).
Изменения BendSeq:
  * headless-safe fallback на tuple;
  * docstring и версия.

Версия: BENDBEQ_BEND_KINEMATICS_V2
"""

from __future__ import annotations

import math
from typing import Optional, Tuple, Union

try:
    import FreeCAD as App
    from FreeCAD import Vector
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Vector = None
    _FC_OK = False


# Тип направления: Vector при наличии FreeCAD, иначе tuple
DirLike = Union["Vector", Tuple[float, float, float]]


def _as_dir(value) -> Optional[DirLike]:
    """Нормализовать входное направление.

    - None -> None
    - Vector -> Vector
    - tuple/list (3 числа) -> Vector или tuple (если FreeCAD нет)
    """
    if value is None:
        return None
    if _FC_OK and isinstance(value, Vector):
        return Vector(value)
    # Пробуем tuple/list из 3 чисел
    try:
        x, y, z = float(value[0]), float(value[1]), float(value[2])
        if _FC_OK:
            return Vector(x, y, z)
        return (x, y, z)
    except (TypeError, ValueError, IndexError, KeyError):
        # Может быть объект с .x/.y/.z
        try:
            x, y, z = float(value.x), float(value.y), float(value.z)
            if _FC_OK:
                return Vector(x, y, z)
            return (x, y, z)
        except (AttributeError, TypeError, ValueError):
            return None


def _to_tuple(v: DirLike) -> Tuple[float, float, float]:
    """Привести Vector или tuple к трём числам."""
    if isinstance(v, tuple):
        return v
    return (float(v.x), float(v.y), float(v.z))


class BendKinematics:
    """Кинематический hint для классификации moving/fixed.

    Attributes:
        operator_side: Vector | tuple | None. Направление «от оператора
                       к детали» в системе Body. None -> hint не
                       используется.
        prefer_moving: "operator" | "away" — какая сторона двигается.
    """

    def __init__(self, operator_side=None, prefer_moving: str = "operator"):
        self.operator_side = _as_dir(operator_side)
        self.prefer_moving = str(prefer_moving).lower()

    @classmethod
    def from_config(cls) -> "BendKinematics":
        """Собрать из SEQUENCE_CONFIG."""
        try:
            from config import SEQUENCE_CONFIG
        except ImportError:
            SEQUENCE_CONFIG = {}

        op = SEQUENCE_CONFIG.get("kinematics_operator_side", None)
        prefer = SEQUENCE_CONFIG.get(
            "kinematics_prefer_moving", "operator")
        return cls(operator_side=op, prefer_moving=prefer)

    def is_configured(self) -> bool:
        return self.operator_side is not None

    def get_moving_direction(self, bend_spec):
        """Проекция operator_side на плоскость, перпендикулярную оси гиба.

        Параметры:
            bend_spec — объект с .axis (Vector или tuple).

        Возвращает:
            Vector — единичная проекция, направленная от оси к
            движущейся стороне. None, если:
              * operator_side не задан;
              * FreeCAD.Vector недоступен (headless);
              * проекция вырождена (operator_side ∥ axis).
        """
        if self.operator_side is None:
            return None
        if not _FC_OK:
            # Без FreeCAD не можем считать cross/projection.
            return None

        axis = getattr(bend_spec, "axis", None)
        if axis is None:
            return None

        try:
            axis_v = Vector(axis) if not isinstance(axis, Vector) else Vector(axis)
        except Exception:
            return None
        if axis_v.Length < 1e-9:
            return None
        axis_v.normalize()

        op = self.operator_side
        if isinstance(op, tuple):
            op = Vector(op[0], op[1], op[2])
        direction = Vector(op)
        direction = direction - axis_v * direction.dot(axis_v)

        if direction.Length < 1e-9:
            return None

        direction.normalize()

        if self.prefer_moving == "away":
            direction = direction * -1.0

        return direction

    def get_rotation_sign(self, bend_spec) -> Optional[int]:
        """±1 из bend_spec.direction ("up" -> +1, "down" -> -1)."""
        direction = getattr(bend_spec, "direction", None)
        if direction is None:
            return None
        return 1 if str(direction).lower() == "up" else -1

    def __repr__(self):
        if self.operator_side is None:
            return "BendKinematics(unconfigured)"
        x, y, z = _to_tuple(self.operator_side)
        return ("BendKinematics(operator_side=({:.3f},{:.3f},{:.3f}), "
                "prefer={})").format(x, y, z, self.prefer_moving)