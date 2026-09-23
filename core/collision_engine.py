# -*- coding: utf-8 -*-
"""
core/collision_engine.py
Двухуровневый движок коллизий OpenCASCADE (AABB -> common).

Особенности:
  * Двухуровневая проверка: сначала BBox (дёшево), потом common()
    (дорого, но точно).
  * bbox_margin расширяет AABB с каждой стороны (см. _bbox_intersects):
    эффективный зазор между боксами = 2 * margin.
  * has_collision = True при объёмном пересечении > volume_epsilon.
  * has_contact = True при min_distance <= contact_tolerance,
    но без объёмного пересечения.
  * check_failed = True при ошибке OCC — вызывающий код должен
    трактовать как «не смог проверить», а не как «нет коллизии».

BendSeq-изменения относительно SM v2.1:
  * DEFAULT_VOLUME_EPSILON = 0.1 (BS-порог для narrow-phase).
    SM-порог 0.001 остаётся как legacy volume_epsilon.
  * pair() читает из config collision_volume_epsilon, если параметр
    volume_epsilon=None. Fallback на volume_epsilon.
  * type hints, docstrings.
  * Headless-safe: без FreeCAD.pair возвращает пустой CollisionResult.

Версия: BENDBEQ_COLLISION_ENGINE_V1
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

try:
    import FreeCAD as App
    import Part
    _FC_OK = True
except ImportError:                                    # pragma: no cover
    App = None
    Part = None
    _FC_OK = False

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


# =====================================================================
# Пороги по умолчанию
# =====================================================================

# BS-порог: 0.1 mm³ = 1 mm² x 0.1 mm глубины, или
# 1.5 mm x 1.5 mm x 0.045 mm. Тонкие нахлёсты на рёбрах (< 0.015 mm³)
# не считаются коллизией, реальное пробитие панели на толщину листа
# (≈ 20 mm³) считается.
DEFAULT_VOLUME_EPSILON = 0.1
DEFAULT_BBOX_MARGIN = 0.1
DEFAULT_CONTACT_TOL = 0.05


# =====================================================================
# Результат
# =====================================================================

@dataclass
class CollisionResult:
    """Результат проверки пары форм.

    Attributes:
        has_collision:     True при объёмном пересечении > ε.
        rejected_by_bbox:  True, если AABB далеко — common() не вызывался.
        volume:            объём пересечения (мм³).
        check_failed:      True при ошибке OCC.
        details:           список строковых пояснений.
        has_contact:       True при min_dist <= contact_tolerance.
        minimum_distance:  минимальное расстояние (только при has_contact).
    """
    has_collision: bool = False
    rejected_by_bbox: bool = False
    volume: float = 0.0
    check_failed: bool = False
    details: List[str] = field(default_factory=list)
    has_contact: bool = False
    minimum_distance: Optional[float] = None


# =====================================================================
# CollisionEngine
# =====================================================================

class CollisionEngine:

    # ------------------------------------------------------------------
    # AABB
    # ------------------------------------------------------------------

    @staticmethod
    def _bbox_intersects(a, b, margin: float = DEFAULT_BBOX_MARGIN) -> bool:
        """Пересечение AABB с расширением margin.

        margin применяется С КАЖДОЙ стороны, эффективный зазор = 2*margin.
        """
        return not (
            a.XMax + margin < b.XMin - margin
            or b.XMax + margin < a.XMin - margin
            or a.YMax + margin < b.YMin - margin
            or b.YMax + margin < a.YMin - margin
            or a.ZMax + margin < b.ZMin - margin
            or b.ZMax + margin < a.ZMin - margin
        )

    # ------------------------------------------------------------------
    # Пороги из config
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_volume_epsilon(value: Optional[float]) -> float:
        """Приоритет: параметр -> collision_volume_epsilon -> volume_epsilon."""
        if value is not None:
            return float(value)
        v = SEQUENCE_CONFIG.get("collision_volume_epsilon")
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                pass
        v = SEQUENCE_CONFIG.get("volume_epsilon")
        if v is not None:
            try:
                return float(v)
            except (TypeError, ValueError):
                pass
        return DEFAULT_VOLUME_EPSILON

    @staticmethod
    def _resolve_bbox_margin(value: Optional[float]) -> float:
        if value is not None:
            return float(value)
        try:
            return float(SEQUENCE_CONFIG.get("bbox_margin",
                                             DEFAULT_BBOX_MARGIN))
        except (TypeError, ValueError):
            return DEFAULT_BBOX_MARGIN

    @staticmethod
    def _resolve_contact_tolerance() -> float:
        try:
            return float(SEQUENCE_CONFIG.get("contact_tolerance",
                                             DEFAULT_CONTACT_TOL))
        except (TypeError, ValueError):
            return DEFAULT_CONTACT_TOL

    # ------------------------------------------------------------------
    # Парная проверка
    # ------------------------------------------------------------------

    

    @classmethod
    def pair(cls,
             shape_a,
             shape_b,
             volume_epsilon: Optional[float] = None,
             bbox_margin: Optional[float] = None,
             logger=None) -> CollisionResult:
        """Проверка коллизии между двумя формами.

        Args:
            shape_a, shape_b: Part.Shape.
            volume_epsilon:   порог объёма (мм³); None -> из config.
            bbox_margin:      расширение AABB (мм); None -> из config.
            logger:           callable(msg) для диагностики.

        Returns:
            CollisionResult.
        """
        v_eps = cls._resolve_volume_epsilon(volume_epsilon)
        margin = cls._resolve_bbox_margin(bbox_margin)
        contact_tol = cls._resolve_contact_tolerance()

        result = CollisionResult()

        # Headless — не можем ничего проверить
        if not _FC_OK:
            result.check_failed = True
            msg = "CollisionEngine.pair: FreeCAD.Part недоступен"
            result.details.append(msg)
            if logger:
                logger(msg)
            return result

        if not shape_a or not shape_b:
            result.check_failed = True
            result.details.append("pair: shape is None или falsy")
            return result
        try:
            if shape_a.isNull() or shape_b.isNull():
                result.check_failed = True
                result.details.append("pair: shape is null")
                return result
        except Exception as e:
            result.check_failed = True
            result.details.append(
                "pair: shape.isNull() exception: {}".format(e))
            return result

        # --- Уровень 1: AABB ---
        try:
            if not cls._bbox_intersects(shape_a.BoundBox, shape_b.BoundBox,
                                        margin):
                result.rejected_by_bbox = True
                return result
        except Exception as e:
            if logger:
                logger("AABB error: {}".format(e))

        # --- Уровень 2: common() ---
        try:
            inter = shape_a.common(shape_b)
            vol = 0.0
            if inter and not inter.isNull():
                vol = float(getattr(inter, "Volume", 0.0) or 0.0)
            result.volume = vol

            if vol > v_eps:
                result.has_collision = True
                result.details.append("V={:.4f} mm³".format(vol))

            # --- Contact (если объёмного пересечения нет) ---
            if vol <= v_eps:
                try:
                    dist_info = shape_a.distToShape(shape_b)
                    if dist_info and len(dist_info) > 0:
                        min_dist = dist_info[0]
                        if (min_dist is not None
                                and min_dist <= contact_tol):
                            result.has_contact = True
                            result.minimum_distance = min_dist
                            result.details.append(
                                "contact distance={:.4f} mm".format(
                                    min_dist))
                except Exception:
                    # distToShape может упасть на вырожденной геометрии —
                    # это не должно ломать основную проверку коллизии.
                    pass

        except Exception as e:
            result.check_failed = True
            msg = "common() error: {}".format(e)
            result.details.append(msg)
            if logger:
                logger(msg)

        return result

    # ------------------------------------------------------------------
    # Self-collision по сегментам
    # ------------------------------------------------------------------

    @classmethod
    def check_self_collision_segments(cls,
                                      segments,
                                      volume_epsilon: Optional[float] = None,
                                      logger=None) -> bool:
        """Проверка самопересечения листа по сегментам.

        Сегменты — список Part.Shape, разбитых по линиям гиба.
        Соседние сегменты (i, i+1) и замкнутые (0, n-1) пропускаются:
        они соединены через линию гиба и всегда граничат.

        Returns:
            True, если найдено объёмное пересечение между несоседними
            сегментами.
        """
        if not _FC_OK:
            return False

        n = len(segments) if segments else 0
        if n < 3:
            return False

        v_eps = cls._resolve_volume_epsilon(volume_epsilon)

        for i in range(n):
            for j in range(i + 2, n):
                if i == 0 and j == n - 1:
                    continue
                res = cls.pair(
                    segments[i], segments[j],
                    volume_epsilon=v_eps, logger=logger)
                if res.has_collision:
                    return True
        return False

    # ------------------------------------------------------------------
    # Диагностика
    # ------------------------------------------------------------------

    @staticmethod
    def describe(res: CollisionResult) -> str:
        """Человекочитаемая сводка CollisionResult."""
        tags = []
        if res.has_collision:
            tags.append("COLLISION")
        if res.has_contact:
            tags.append("CONTACT")
        if res.rejected_by_bbox:
            tags.append("bbox-far")
        if res.check_failed:
            tags.append("CHECK-FAILED")
        if not tags:
            tags.append("free")
        return ("CollisionResult[{}] volume={:.4f} "
                "min_dist={} details={}").format(
            ",".join(tags), res.volume,
            res.minimum_distance, res.details)