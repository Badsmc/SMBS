# -*- coding: utf-8 -*-
"""
core/bend_feature.py
Модель гиба: BendSpec, BendFeature, BendLine.

BendLine хранит:
  - геометрию линии гиба в развёртке (flat_start, flat_end, axis_flat);
  - связь с Body (body_center, body_mid, body_direction);
  - target_com — CoM delta-части для разрешения знака в KinematicEngine.

BendFeature группирует линии одной SMBendWall-фичи. Multi-edge фичи
(например, Bend004 у U-канала) имеют len(lines) > 1. Каждая линия
активируется отдельным шагом (feat_id, line_idx) — 10 шагов вместо 6.

target_com для multi-edge: tc = (bm.x, bm.y, delta_cz), где delta_cz —
CoM BBox delta-части по Z. Формула v12.4 freecad_adapter.

Перенесено из freecad_sheetmetal_sequence (core/bend_feature.py v1.6).
Изменения BendSeq:
  * docstring и версия;
  * type hints (Optional, List);
  * __repr__ сохранён.

Версия: BENDBEQ_BEND_FEATURE_V1
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

try:
    import FreeCAD as App
    from FreeCAD import Vector
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Vector = None
    _FC_OK = False


# =====================================================================
# BendLine
# =====================================================================

@dataclass
class BendLine:
    """Одна плоская линия гиба.

    Attributes:
        index:          позиция внутри feat.lines (0..N-1).
        flat_start:     начало линии в плоской развёртке.
        flat_end:       конец линии в плоской развёртке.
        axis_flat:      нормализованное направление линии (в развёртке).
        length_mm:      длина линии.
        sub_element:    "Edge37" — ссылка на ребро parent-фичи.
        segment_id:     "Bend004_1" для multi-edge, "Bend001" для single.
        body_center:    центр линии в системе Body.
        body_direction: "left"/"right"/"up"/"down" — куда смещён центр
                        относительно CoM parent-фичи.
        body_mid:       то же, что body_center (алиас для читаемости).
        target_com:     CoM delta-части этой линии в Body (для резолвера
                        знаков KinematicEngine).
    """
    index: int
    flat_start: Vector
    flat_end: Vector
    axis_flat: Vector
    length_mm: float
    sub_element: str
    segment_id: str
    body_center: Optional[Vector] = None
    body_direction: Optional[str] = None
    body_mid: Optional[Vector] = None
    target_com: Optional[Vector] = None

    def center(self) -> Vector:
        """Центр линии (в развёртке)."""
        return (self.flat_start + self.flat_end) * 0.5


# =====================================================================
# BendFeature
# =====================================================================

@dataclass
class BendFeature:
    """Одна SheetMetal-операция (PartDesign::FeaturePython с SMBendWall).

    Может породить несколько BendLine (по одной на ребро parent-фичи).

    Attributes:
        id:                 "Bend004"
        angle_deg:          внутренний угол (90 = перпендикуляр).
        direction:          "up" | "down".
        radius_mm:          внутренний радиус гиба.
        invert:             флаг инверсии из SMBendWall.
        parent_feature:     "Bend002" | "Pad" | "".
        lines:              List[BendLine].
        flange_lengths_mm:  длины полок из LengthList.
        flip_sign:          True, если angle < 89.9° (эвристика).
        sign_override:      0 | +1 | -1 — принудительный знак.
        parent_panel_id:    заполняется PanelGraph._assign_features_via_side.
        moving_panel_ids:   заполняется PanelGraph (для диагностики).
    """
    id: str
    angle_deg: float
    direction: str
    radius_mm: float
    invert: bool
    parent_feature: str
    lines: List[BendLine] = field(default_factory=list)
    flange_lengths_mm: List[float] = field(default_factory=list)
    flip_sign: bool = False
    sign_override: int = 0

    parent_panel_id: Optional[str] = None
    moving_panel_ids: List[str] = field(default_factory=list)

    @property
    def is_multi_edge(self) -> bool:
        return len(self.lines) > 1

    def __repr__(self):
        tag = ("multi({})".format(len(self.lines))
               if self.is_multi_edge else "single")
        flip = " FLIP" if self.flip_sign else ""
        so = " SO={:+d}".format(self.sign_override) if self.sign_override else ""
        return ("BendFeature({}, {:.1f}° {}, {}, parent={}{}{})").format(
            self.id, self.angle_deg, self.direction, tag,
            self.parent_feature, flip, so)


# =====================================================================
# Утилиты
# =====================================================================

def group_bend_specs_by_feature(bends) -> dict:
    """Сгруппировать список BendSpec по метаполю 'feature'.

    Вход: list[BendSpec] (из adapters.freecad_adapter).
    Выход: {"Bend004": [spec, spec], "Bend001": [spec], ...}
    """
    result: dict = {}
    for b in bends:
        meta = getattr(b, "metadata", {}) or {}
        feat = str(meta.get("feature") or "").strip()
        if not feat:
            continue
        result.setdefault(feat, []).append(b)
    return result


def describe_features(features: List[BendFeature]) -> str:
    """Человекочитаемая сводка BendFeature для логов/UI."""
    lines = ["BendFeatures: {}".format(len(features))]
    for f in features:
        lines.append("  {}".format(f))
        for ln in f.lines:
            bd = ""
            if ln.body_direction:
                bd = " body_dir={}".format(ln.body_direction)
            tc = ""
            if ln.target_com is not None:
                tc = " tc=({:.2f},{:.2f},{:.2f})".format(
                    ln.target_com.x, ln.target_com.y, ln.target_com.z)
            lines.append("    [{}] len={:.2f}{}{}".format(
                ln.segment_id, ln.length_mm, bd, tc))
    return "\n".join(lines)