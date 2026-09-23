# -*- coding: utf-8 -*-
"""Спецификации инструмента — чистые данные, без FreeCAD."""

from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional


@dataclass
class PunchSpec:
    """Определение пуансона.

    geometry_mode:
        "parametric"  — строится по числам ниже
        "imported"    — сохранён Shape из FreeCAD (baked), geometry_file
        "file"        — ссылка на .step/.FCStd
    """
    id: str
    name: str
    geometry_mode: str = "parametric"

    # Рабочие параметры
    angle: float = 88.0            # угол при вершине
    nose_radius: float = 0.8
    height: float = 100.0          # от носика до хвостовика
    tip_width: float = 10.0        # ширина плоской площадки носика
    body_width: float = 60.0       # ширина тела
    tip_height: float = 6.0        # высота перехода носик→тело
    profile: str = "brick"         # "brick" | "gooseneck"
    gooseneck_side: str = "right"

    # Геометрия (только для geometry_mode != "parametric")
    geometry_file: Optional[str] = None   # относительно library root

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "PunchSpec":
        return cls(**{k: v for k, v in d.items()
                      if k in cls.__dataclass_fields__})


@dataclass
class DieSpec:
    id: str
    name: str
    geometry_mode: str = "parametric"

    opening: float = 12.0          # ширина V-ручья (Ve)
    angle: float = 88.0            # угол между стенками V
    radius: float = 0.5
    height: float = 80.0
    width: float = 70.0

    geometry_file: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "DieSpec":
        return cls(**{k: v for k, v in d.items()
                      if k in cls.__dataclass_fields__})


@dataclass
class ToolSetSpec:
    """Пресет: пара пуансон + матрица под именем."""
    id: str
    name: str
    punch_id: str
    die_id: str
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ToolSetSpec":
        return cls(**{k: v for k, v in d.items()
                      if k in cls.__dataclass_fields__})