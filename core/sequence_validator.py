# -*- coding: utf-8 -*-
"""
core/sequence_validator.py
Инкрементальный валидатор шагов гибки.

Версия: BENDBEQ_SEQ_VALIDATOR_V7_8

Возможности:
  * validate_step          — один шаг (подвод + гиб + проверки).
  * validate_full_sequence — прямой проход (flat -> Body).
  * validate_full_sequence_reverse — обратный проход (Body -> flat),
    строит states[] через simulate_bend_occ(angle_factor=-1.0), затем
    переигрывает вперёд с preset_shape_after.

V7.8 (BendSeq):
  * tool_h в стартовой высоте подвода (реальный габарит инструмента).
  * die_offset = -(thickness + die_clearance): верх матрицы под листом.
  * Диагностика bbox v_punch / v_die для отладки реальной оснастки.
  * Guard: v_punch/v_die == None/Null -> CHECK_FAILED (не молчим).
  * Восстановлен вызов v_die = self.die_adapter.make_virtual_shape(...).
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Any, List, Optional

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

from core.collision_engine import CollisionEngine
from core.bend_simulator import (
    simulate_bend_occ, SliceFailure, transform_bend_spec_inplace,
)

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


MODULE_VERSION = "BENDBEQ_SEQ_VALIDATOR_V7_8"
if _FC_OK:
    try:
        App.Console.PrintMessage(
            "[sequence_validator] Загружена версия: {}\n".format(
                MODULE_VERSION))
    except Exception:
        pass


# =====================================================================
# Результат шага
# =====================================================================

@dataclass
class StepValidationResult:
    """Результат валидации одного шага.

    Attributes:
        bend_id:         идентификатор гиба.
        step_number:     1-based номер шага.
        ok:              True, если шаг прошёл все проверки.
        shape_after:     Part.Shape после гиба (если ok).
        collision_type:  "PUNCH_BEFORE" | "PUNCH_COLLISION" |
                         "DIE_COLLISION" | "PRESS_COLLISION" |
                         "SELF_COLLISION" | "GEOMETRY_INVALID" |
                         "BACKGAUGE_COLLISION" | "CHECK_FAILED" |
                         "SLICE_FAILURE" | "SIMULATION_FAILED" |
                         "BEND_TRANSFORM_FAILED" |
                         "BLOCKED_BY_PREVIOUS_STEP" | None.
        reason_human:    человекочитаемое объяснение.
        details:         диагностика (в т.ч. _transform_info).
        volume:          объём пересечения (если применимо).
        is_contact:      True при касании (min_distance <= tol), не коллизии.
        min_distance:    минимальное расстояние (только при is_contact).
        bend_spec:       оригинальный BendSpec (до трансформации).
        working_bend_spec: копия, с которой работает валидатор.
    """
    bend_id: str
    step_number: int
    ok: bool
    shape_after: Optional[Any]
    collision_type: Optional[str]
    reason_human: str
    details: dict = field(default_factory=dict)
    volume: float = 0.0
    is_contact: bool = False
    min_distance: Optional[float] = None
    bend_spec: Optional[Any] = None
    working_bend_spec: Optional[Any] = None


# =====================================================================
# Вспомогательные функции
# =====================================================================

def _clone_bend_spec(bend):
    """Копия BendSpec с независимыми Vector'ами и metadata."""
    BendSpecCls = type(bend)
    return BendSpecCls(
        id=bend.id,
        name=bend.name,
        center=Vector(bend.center),
        axis=Vector(bend.axis),
        angle=float(bend.angle),
        length=float(bend.length),
        direction=getattr(bend, "direction", "up"),
        normal=Vector(getattr(bend, "normal", Vector(0, 0, 1))),
        up_hint=Vector(getattr(bend, "up_hint", Vector(0, 0, 1))),
        source=getattr(bend, "source", None),
        metadata=copy.deepcopy(getattr(bend, "metadata", {}) or {}),
    )


def _sheet_normal_of(bend) -> "Vector":
    """Единичная нормаль листа из bend.normal."""
    try:
        n = Vector(getattr(bend, "normal", Vector(0, 0, 1)))
    except Exception:
        n = Vector(0, 0, 1)
    if n.Length < 1e-9:
        return Vector(0, 0, 1)
    return n.normalize()


def _sheet_thickness(flat_shape) -> float:
    """Толщина листа = min BBox dimension плоской развёртки."""
    try:
        if flat_shape is None or flat_shape.isNull():
            return 1.0
    except Exception:
        return 1.0

    try:
        bb = flat_shape.BoundBox
    except Exception:
        return 1.0

    candidates = []
    for v in (bb.XLength, bb.YLength, bb.ZLength):
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        if v > 1e-6:
            candidates.append(v)

    if not candidates:
        return 1.0

    t = min(candidates)
    if t > 100.0:
        return 1.0
    return t


def _bbox_str(shape) -> str:
    """Короткая строка BBox для логов."""
    try:
        bb = shape.BoundBox
        return ("size=({:.1f},{:.1f},{:.1f}) "
                "min=({:.1f},{:.1f},{:.1f}) "
                "max=({:.1f},{:.1f},{:.1f})").format(
            bb.XLength, bb.YLength, bb.ZLength,
            bb.XMin, bb.YMin, bb.ZMin,
            bb.XMax, bb.YMax, bb.ZMax)
    except Exception:
        return "size=?"


def _shape_bad(shape) -> bool:
    """None или Null или невалидный shape."""
    if shape is None:
        return True
    try:
        return bool(shape.isNull())
    except Exception:
        return True


# =====================================================================
# SequenceValidator
# =====================================================================

class SequenceValidator:
    """Инкрементальный валидатор последовательности гибки."""

    def __init__(self,
                 punch_adapter=None,
                 die_adapter=None,
                 press_shapes=None,
                 backgauge_adapter=None,
                 config=None,
                 logger=None,
                 unfold_obj=None):
        self.punch_adapter = punch_adapter
        self.die_adapter = die_adapter
        self.press_shapes = press_shapes or []
        self.backgauge_adapter = backgauge_adapter
        self.config = config or dict(SEQUENCE_CONFIG)
        self.logger = logger
        self.unfold_obj = unfold_obj
        self._flat_ref = None

    def _log(self, msg: str) -> None:
        if self.logger:
            try:
                self.logger(msg)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Один шаг
    # ------------------------------------------------------------------

    def validate_step(self,
                      current_shape,
                      bend,
                      step_idx: int,
                      already_done: Optional[list] = None,
                      kinematics=None,
                      preset_shape_after=None) -> StepValidationResult:
        res = self._validate_step_impl(
            current_shape, bend, step_idx,
            already_done=already_done,
            kinematics=kinematics,
            preset_shape_after=preset_shape_after,
        )

        bend_id = getattr(bend, "id", str(step_idx))
        if not res.ok:
            self._log(
                "[validator] Шаг {} ({}) ПРОВАЛ: type={} — {}".format(
                    step_idx + 1, bend_id, res.collision_type,
                    res.reason_human))
            if res.details:
                self._log("[validator]   details={}".format(res.details))
        else:
            extra = ""
            if getattr(res, "is_contact", False):
                md = getattr(res, "min_distance", None)
                extra = (", contact min_d={:.4f}".format(md)
                         if md is not None else ", contact")
            self._log("[validator] Шаг {} ({}) OK{}".format(
                step_idx + 1, bend_id, extra))

        return res

    def _validate_step_impl(self,
                            current_shape,
                            bend,
                            step_idx: int,
                            already_done: Optional[list] = None,
                            kinematics=None,
                            preset_shape_after=None
                            ) -> StepValidationResult:
        if not _FC_OK:
            return StepValidationResult(
                bend_id=getattr(bend, "id", str(step_idx)),
                step_number=step_idx + 1,
                ok=False,
                shape_after=None,
                collision_type="CHECK_FAILED",
                reason_human="FreeCAD недоступен (headless)",
            )

        already_done = already_done or []
        bend_id = getattr(bend, "id", str(step_idx))

        res = StepValidationResult(
            bend_id=bend_id,
            step_number=step_idx + 1,
            ok=True,
            shape_after=None,
            collision_type=None,
            reason_human="ОК. Коллизий не обнаружено.",
        )

        if _shape_bad(current_shape):
            res.ok = False
            res.collision_type = "BLOCKED_BY_PREVIOUS_STEP"
            res.reason_human = "Отсутствует входная геометрия заготовки."
            return res

        vol_eps = float(self.config.get("volume_epsilon", 0.001))
        approach_dist = float(self.config.get("approach_distance", 50.0))
        samples = max(1, int(self.config.get("approach_samples", 5)))

        center = getattr(bend, "center", None)
        axis = getattr(bend, "axis", None)
        length = float(getattr(bend, "length", 100.0))

        sheet_normal = _sheet_normal_of(bend)
        tool_up_hint = sheet_normal

        flat_ref = getattr(self, "_flat_ref", None)
        if flat_ref is not None:
            thickness = _sheet_thickness(flat_ref)
            t_src = "flat_ref"
        else:
            thickness = _sheet_thickness(current_shape)
            t_src = "current"

        self._log("[validator] Шаг {} ({}): thickness={:.3f} мм "
                  "(source={})".format(
                      step_idx + 1, bend_id, thickness, t_src))

        # --- 1. Подвод пуансона (до гиба) ---
        if (self.config.get("check_punch", True)
                and self.punch_adapter and center is not None):
            try:
                tool_h = 0.0
                if bool(self.config.get("tool_approach_from_height", True)):
                    try:
                        tool_h = float(
                            self.punch_adapter.working_height())
                    except Exception:
                        tool_h = 0.0

                start_h = approach_dist + thickness + tool_h * 0.5
                end_h = thickness + 0.01
                step_h = (start_h - end_h) / float(samples)

                self._log(
                    "[validator]   approach: tool_h={:.1f} "
                    "start_h={:.1f} end_h={:.2f} samples={}".format(
                        tool_h, start_h, end_h, samples))

                for i in range(samples + 1):
                    h = start_h - i * step_h
                    if h < end_h:
                        h = end_h
                    offset_vec = sheet_normal * h
                    v_punch = self.punch_adapter.make_virtual_shape(
                        center, axis, up_hint=tool_up_hint,
                        length=length, offset=offset_vec,
                    )
                    if _shape_bad(v_punch):
                        res.ok = False
                        res.collision_type = "CHECK_FAILED"
                        res.reason_human = (
                            "Не удалось построить виртуальный пуансон "
                            "(shape=None/Null). Проверка невозможна."
                        )
                        return res
                    if i == 0:
                        try:
                            self._log(
                                "[validator]   v_punch@h={:.1f} {}".format(
                                    h, _bbox_str(v_punch)))
                        except Exception:
                            pass

                    c_res = CollisionEngine.pair(
                        current_shape, v_punch,
                        volume_epsilon=vol_eps, logger=self._log,
                    )
                    if c_res.check_failed:
                        res.ok = False
                        res.collision_type = "CHECK_FAILED"
                        res.reason_human = (
                            "Ошибка при проверке подвода пуансона (OCC)."
                        )
                        return res
                    if c_res.has_collision:
                        res.ok = False
                        res.collision_type = "PUNCH_BEFORE"
                        res.volume = c_res.volume
                        res.reason_human = (
                            "Пуансон не проходит: столкновение с фланцем "
                            "предыдущего гиба на высоте {:.1f} мм "
                            "от линии, V={:.3f} мм³.".format(
                                h, c_res.volume)
                        )
                        return res
            except Exception as exc:
                self._log("Ошибка Punch Insertion: {}".format(exc))

        # --- 2. Симуляция гиба ---
        if preset_shape_after is not None:
            bent_shape = preset_shape_after
            transform_info = {}
        else:
            try:
                bent_shape, transform_info = simulate_bend_occ(
                    current_shape, bend, kinematics=kinematics)
            except SliceFailure as exc:
                res.ok = False
                res.collision_type = "SLICE_FAILURE"
                res.reason_human = "Не удалось построить гиб: {}.".format(exc)
                res.details = {
                    "slice_failure": getattr(exc, "details", {})
                }
                return res
            except Exception as exc:
                res.ok = False
                res.collision_type = "SIMULATION_FAILED"
                res.reason_human = (
                    "Непредвиденная ошибка при 3D-симуляции гиба: "
                    "{}".format(exc)
                )
                res.details["exception"] = repr(exc)
                return res

        res.shape_after = bent_shape
        res.details["_transform_info"] = transform_info

        # --- 3. Столкновение пуансона (после гиба) ---
        if (self.config.get("check_punch", True)
                and self.punch_adapter and center is not None):
            try:
                clearance = float(self.config.get(
                    "punch_clearance_offset", 2.0))
                offset_vec = sheet_normal * (thickness + clearance)
                v_punch = self.punch_adapter.make_virtual_shape(
                    center, axis, up_hint=tool_up_hint,
                    length=length, offset=offset_vec,
                )
                if not _shape_bad(v_punch):
                    c_res = CollisionEngine.pair(
                        current_shape, v_punch,
                        volume_epsilon=vol_eps, logger=self._log,
                    )
                    if c_res.check_failed:
                        res.ok = False
                        res.collision_type = "CHECK_FAILED"
                        res.reason_human = (
                            "Ошибка при проверке коллизии с пуансоном."
                        )
                        return res
                    if c_res.has_collision:
                        res.ok = False
                        res.collision_type = "PUNCH_COLLISION"
                        res.volume = c_res.volume
                        res.reason_human = (
                            "Пуансон врезается в уже согнутый фланец "
                            "от предыдущего шага "
                            "(V={:.3f} мм³).".format(c_res.volume)
                        )
                        return res
                    if c_res.has_contact:
                        res.is_contact = True
                        res.min_distance = c_res.minimum_distance
            except Exception as exc:
                self._log("Ошибка Punch Collision: {}".format(exc))

        # --- 4. Матрица ---
        if (self.config.get("check_die", True)
                and self.die_adapter and center is not None):
            try:
                die_clearance = float(self.config.get(
                    "die_clearance_offset", 0.5))
                # Верх матрицы (ref = max-up) должен быть НИЖЕ нижней
                # плоскости листа на die_clearance. bend_center.z — верх
                # листа, поэтому сдвиг = -(thickness + die_clearance).
                die_offset = sheet_normal * (-(thickness + die_clearance))

                # ВАЖНО: v_die создаётся ЗДЕСЬ (был потерян при патче).
                v_die = self.die_adapter.make_virtual_shape(
                    center, axis, up_hint=tool_up_hint,
                    length=length, offset=die_offset,
                )
                if _shape_bad(v_die):
                    res.ok = False
                    res.collision_type = "CHECK_FAILED"
                    res.reason_human = (
                        "Не удалось построить виртуальную матрицу "
                        "(shape=None/Null). Проверка невозможна."
                    )
                    return res
                try:
                    self._log(
                        "[validator]   v_die {}".format(
                            _bbox_str(v_die)))
                except Exception:
                    pass

                c_res = CollisionEngine.pair(
                    bent_shape, v_die,
                    volume_epsilon=vol_eps, logger=self._log,
                )
                if c_res.check_failed:
                    res.ok = False
                    res.collision_type = "CHECK_FAILED"
                    res.reason_human = (
                        "Ошибка при проверке коллизии с матрицей."
                    )
                    return res
                if c_res.has_collision:
                    res.ok = False
                    res.collision_type = "DIE_COLLISION"
                    res.volume = c_res.volume
                    res.reason_human = (
                        "Отогнутый фланец врезается в матрицу "
                        "(V={:.3f} мм³).".format(c_res.volume)
                    )
                    return res
                if c_res.has_contact and not res.is_contact:
                    res.is_contact = True
                    res.min_distance = c_res.minimum_distance
            except Exception as exc:
                self._log("Ошибка Die Collision: {}".format(exc))

        # --- 5. Станина пресса ---
        if (self.config.get("check_press_frame", True)
                and self.press_shapes):
            for press_shape in self.press_shapes:
                if _shape_bad(press_shape):
                    continue
                c_res = CollisionEngine.pair(
                    bent_shape, press_shape,
                    volume_epsilon=vol_eps, logger=self._log,
                )
                if c_res.check_failed:
                    res.ok = False
                    res.collision_type = "CHECK_FAILED"
                    res.reason_human = (
                        "Ошибка при проверке коллизии со станиной."
                    )
                    return res
                if c_res.has_collision:
                    res.ok = False
                    res.collision_type = "PRESS_COLLISION"
                    res.volume = c_res.volume
                    res.reason_human = (
                        "Отогнутый фланец упирается в станину пресса "
                        "(V={:.3f} мм³).".format(c_res.volume)
                    )
                    return res
                if c_res.has_contact and not res.is_contact:
                    res.is_contact = True
                    res.min_distance = c_res.minimum_distance

        # --- 6. Самопересечение / isValid ---
        if self.config.get("check_self_collision", True):
            if kinematics and hasattr(kinematics, "get_segment_solids"):
                try:
                    segments = kinematics.get_segment_solids(bent_shape)
                    if CollisionEngine.check_self_collision_segments(
                            segments, volume_epsilon=vol_eps,
                            logger=self._log):
                        res.ok = False
                        res.collision_type = "SELF_COLLISION"
                        res.reason_human = (
                            "Заготовка пересекает сама себя."
                        )
                        return res
                except Exception as exc:
                    self._log(
                        "Ошибка Self-Collision (segments): {}".format(exc))
            else:
                try:
                    if not bent_shape.isValid():
                        res.ok = False
                        res.collision_type = "GEOMETRY_INVALID"
                        res.reason_human = (
                            "Топология детали невалидна после гиба "
                            "(OCC isValid=False)."
                        )
                        return res
                except Exception:
                    pass

        # --- 7. Задний упор ---
        if (self.config.get("check_backgauge", True)
                and self.backgauge_adapter):
            try:
                gauge_clearance = float(self.config.get(
                    "backgauge_clearance", 1.0))
                gauge_offset = sheet_normal * gauge_clearance
                v_gauge = self.backgauge_adapter.make_virtual_shape(
                    center, axis, offset=gauge_offset)
                if not _shape_bad(v_gauge):
                    c_res = CollisionEngine.pair(
                        bent_shape, v_gauge,
                        volume_epsilon=vol_eps, logger=self._log,
                    )
                    if c_res.check_failed:
                        res.ok = False
                        res.collision_type = "CHECK_FAILED"
                        res.reason_human = (
                            "Ошибка при проверке заднего упора."
                        )
                        return res
                    if c_res.has_collision:
                        res.ok = False
                        res.collision_type = "BACKGAUGE_COLLISION"
                        res.volume = c_res.volume
                        res.reason_human = (
                            "Деталь врезается в задний упор "
                            "(V={:.3f} мм³).".format(c_res.volume)
                        )
                        return res
                    if c_res.has_contact and not res.is_contact:
                        res.is_contact = True
                        res.min_distance = c_res.minimum_distance
            except Exception as exc:
                self._log("Ошибка Backgauge: {}".format(exc))

        return res

    # ------------------------------------------------------------------
    # Полная последовательность: forward
    # ------------------------------------------------------------------

    def validate_full_sequence(self,
                               flat_shape,
                               ordered_bends: list,
                               kinematics=None) -> List[StepValidationResult]:
        if ordered_bends is None:
            ordered_bends = []

        self._flat_ref = flat_shape

        self._log(
            "[validator] Старт ПРЯМОЙ валидации: гибов={}, ids={}".format(
                len(ordered_bends),
                [getattr(b, "id", "?") for b in ordered_bends]))

        t = _sheet_thickness(flat_shape)
        self._log(
            "[validator] Толщина листа (min BBox flat) = {:.3f} мм".format(t))

        if not _FC_OK:
            return [StepValidationResult(
                bend_id=getattr(b, "id", str(i)),
                step_number=i + 1,
                ok=False,
                shape_after=None,
                collision_type="CHECK_FAILED",
                reason_human="FreeCAD недоступен (headless)",
            ) for i, b in enumerate(ordered_bends)]

        working_bends = [_clone_bend_spec(b) for b in ordered_bends]
        results: List[StepValidationResult] = []
        current_shape = (
            flat_shape.copy()
            if flat_shape and not flat_shape.isNull() else None
        )

        already_done = []
        blocked = False

        for idx, working_bend in enumerate(working_bends):
            original_bend = ordered_bends[idx]
            bend_id = getattr(original_bend, "id", str(idx))

            if blocked:
                res = StepValidationResult(
                    bend_id=bend_id,
                    step_number=idx + 1,
                    ok=False,
                    shape_after=None,
                    collision_type="BLOCKED_BY_PREVIOUS_STEP",
                    reason_human=(
                        "Предыдущий шаг завершился с ошибкой, "
                        "дальнейшая проверка невозможна."
                    ),
                )
                res.bend_spec = original_bend
                res.working_bend_spec = working_bend
                results.append(res)
                self._log(
                    "[validator] Шаг {} ({}) пропущен: "
                    "BLOCKED_BY_PREVIOUS_STEP".format(idx + 1, bend_id))
                continue

            step_res = self.validate_step(
                current_shape, working_bend, idx,
                already_done, kinematics=kinematics,
            )

            step_res.bend_spec = original_bend
            step_res.working_bend_spec = working_bend
            results.append(step_res)

            if step_res.ok and step_res.shape_after is not None:
                current_shape = step_res.shape_after
                transform_info = step_res.details.get("_transform_info")

                if transform_info is not None:
                    cur_meta = getattr(working_bend, "metadata", {}) or {}
                    cur_feature = str(cur_meta.get("feature") or "")
                    cur_side = str(cur_meta.get("body_direction") or "")

                    force_ids = set()
                    for future_bend in working_bends[idx + 1:]:
                        f_meta = getattr(future_bend, "metadata", {}) or {}
                        if (str(f_meta.get("parent_feature") or "")
                                != cur_feature):
                            continue
                        f_side = str(f_meta.get("body_direction") or "")
                        if cur_side and f_side == cur_side:
                            force_ids.add(future_bend.id)
                    transform_info["force_ids"] = force_ids

                    if force_ids:
                        self._log(
                            "[validator] force_ids для [{}] "
                            "(feature={}, side={}): {}".format(
                                bend_id, cur_feature, cur_side,
                                sorted(force_ids)))

                    remaining = working_bends[idx + 1:]
                    for future_bend in remaining:
                        try:
                            transform_bend_spec_inplace(
                                future_bend, transform_info)
                        except Exception as exc:
                            step_res.ok = False
                            step_res.collision_type = (
                                "BEND_TRANSFORM_FAILED")
                            step_res.reason_human = (
                                "Не удалось преобразовать координаты "
                                "будущих гибов: {}".format(exc)
                            )
                            blocked = True
                            break

                if step_res.ok:
                    already_done.append(working_bend)
                else:
                    blocked = True
            else:
                blocked = True
                already_done.append(working_bend)

        return results

    # ------------------------------------------------------------------
    # Полная последовательность: reverse
    # ------------------------------------------------------------------

    def validate_full_sequence_reverse(self,
                                       body_shape,
                                       ordered_bends: list,
                                       kinematics=None,
                                       flat_ref=None
                                       ) -> Optional[List[StepValidationResult]]:
        if ordered_bends is None:
            ordered_bends = []
        if not ordered_bends:
            return []
        if _shape_bad(body_shape):
            return []
        if not _FC_OK:
            return None

        if flat_ref is not None:
            self._flat_ref = flat_ref
        else:
            self._log(
                "[validator] ВНИМАНИЕ: обратный проход без flat_ref — "
                "толщина будет считаться от Body (неточно).")
            self._flat_ref = body_shape

        n = len(ordered_bends)
        working_bends = [_clone_bend_spec(b) for b in ordered_bends]

        self._log(
            "[validator] Старт ОБРАТНОЙ валидации: гибов={}, ids={}".format(
                n, [getattr(b, "id", "?") for b in working_bends]))
        t = _sheet_thickness(self._flat_ref)
        self._log(
            "[validator] Толщина листа (min BBox flat_ref) = "
            "{:.3f} мм".format(t))

        states: List[Any] = [None] * (n + 1)
        states[n] = body_shape.copy()

        current = body_shape.copy()
        reverse_failed_at = None

        for k in range(n - 1, -1, -1):
            bend = working_bends[k]
            bend_id = getattr(bend, "id", str(k))

            try:
                unbent, _ = simulate_bend_occ(
                    current, bend,
                    kinematics=kinematics,
                    angle_factor=-1.0,
                )
            except SliceFailure as exc:
                self._log(
                    "[validator] Обратный отгиб [{}] "
                    "ПРОВАЛ (SliceFailure): {}".format(bend_id, exc))
                unbent = None
                if reverse_failed_at is None:
                    reverse_failed_at = k
            except Exception as exc:
                self._log(
                    "[validator] Обратный отгиб [{}] "
                    "exception: {}".format(bend_id, exc))
                unbent = None
                if reverse_failed_at is None:
                    reverse_failed_at = k

            if _shape_bad(unbent):
                states[k] = current.copy() if current else None
            else:
                states[k] = unbent.copy()
                current = unbent

            if states[k] is not None:
                try:
                    bb = states[k].BoundBox
                    self._log(
                        "[validator] state[{}] после отгиба {}: "
                        "bbox=({:.1f},{:.1f},{:.1f}), vol={:.1f}".format(
                            k, bend_id,
                            bb.XLength, bb.YLength, bb.ZLength,
                            states[k].Volume))
                except Exception:
                    pass

        flat_ok = False
        if states[0] is not None:
            try:
                bb0 = states[0].BoundBox
                dims = sorted([bb0.XLength, bb0.YLength, bb0.ZLength])
                self._log(
                    "[validator] Финальная 'плоская' форма: "
                    "{:.1f} x {:.1f} x {:.1f}".format(
                        bb0.XLength, bb0.YLength, bb0.ZLength))
                if dims[0] <= max(5.0 * t, 5.0):
                    flat_ok = True
                else:
                    self._log(
                        "[validator] Обратный проход не сошёлся "
                        "к плоскому: мин. габарит {:.2f} > "
                        "{:.2f}, первый провал на шаге {}".format(
                            dims[0], max(5.0 * t, 5.0),
                            reverse_failed_at))
            except Exception:
                pass

        if not flat_ok:
            self._log(
                "[validator] Возвращаем None — вызывающий код должен "
                "использовать validate_full_sequence (прямой проход).")
            return None

        results: List[StepValidationResult] = []
        already_done = []
        blocked = False

        for idx, bend in enumerate(working_bends):
            original_bend = ordered_bends[idx]
            bend_id = getattr(original_bend, "id", str(idx))

            shape_before = states[idx]
            shape_after = states[idx + 1]

            if blocked:
                res = StepValidationResult(
                    bend_id=bend_id,
                    step_number=idx + 1,
                    ok=False,
                    shape_after=None,
                    collision_type="BLOCKED_BY_PREVIOUS_STEP",
                    reason_human=(
                        "Предыдущий шаг завершился с ошибкой, "
                        "дальнейшая проверка невозможна."),
                )
                res.bend_spec = original_bend
                res.working_bend_spec = bend
                results.append(res)
                self._log(
                    "[validator] Шаг {} ({}) пропущен: "
                    "BLOCKED_BY_PREVIOUS_STEP".format(idx + 1, bend_id))
                continue

            step_res = self.validate_step(
                shape_before, bend, idx, already_done,
                kinematics=kinematics,
                preset_shape_after=shape_after,
            )
            step_res.bend_spec = original_bend
            step_res.working_bend_spec = bend
            results.append(step_res)

            if step_res.ok and step_res.shape_after is not None:
                already_done.append(bend)
            else:
                blocked = True

        return results