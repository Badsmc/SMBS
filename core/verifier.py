# -*- coding: utf-8 -*-
"""
core/verifier.py
Сравнение симулированной детали с реальным Body.

Публичный API:
    verify_against_reference(sim_shape, ref_shape, logger=None)
        -> VerificationReport

Метрики:
    * volume_sim / volume_ref / volume_delta_rel
    * area_sim / area_ref / area_delta_rel
    * bbox_sim / bbox_ref / bbox_max_delta (по отсортированным габаритам)
    * inertia_sim / inertia_ref / inertia_max_delta_rel (главные моменты)

Допуски читаются из SEQUENCE_CONFIG на каждом вызове (не на import),
чтобы изменения в config применялись без перезагрузки модуля.

Headless-safe: без FreeCAD импорт не падает, verify вернёт ok=False
с note «no_freecad».

Перенесено из freecad_sheetmetal_sequence (core/verifier.py v1.2).
BendSeq-изменения: typed, docstring.

Версия: BENDBEQ_VERIFIER_V1
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

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
# Дефолты допусков
# =====================================================================

_DEFAULT_TOL_VOLUME_REL = 0.025
_DEFAULT_TOL_AREA_REL = 0.010
_DEFAULT_TOL_BBOX_MM = 2.0
_DEFAULT_TOL_INERTIA_REL = 0.02


def _get_tol(key: str, default: float) -> float:
    try:
        return float(SEQUENCE_CONFIG.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _tolerances() -> Tuple[float, float, float, float]:
    """Читается при каждом вызове — не на import."""
    return (
        _get_tol("verify_tol_volume_rel", _DEFAULT_TOL_VOLUME_REL),
        _get_tol("verify_tol_area_rel", _DEFAULT_TOL_AREA_REL),
        _get_tol("verify_tol_bbox_mm", _DEFAULT_TOL_BBOX_MM),
        _get_tol("verify_tol_inertia_rel", _DEFAULT_TOL_INERTIA_REL),
    )


# Совместимость со старым кодом, который читал TOL_* на import.
TOL_VOLUME_REL = _get_tol("verify_tol_volume_rel",
                           _DEFAULT_TOL_VOLUME_REL)
TOL_AREA_REL = _get_tol("verify_tol_area_rel", _DEFAULT_TOL_AREA_REL)
TOL_BBOX_MM = _get_tol("verify_tol_bbox_mm", _DEFAULT_TOL_BBOX_MM)
TOL_INERTIA_REL = _get_tol("verify_tol_inertia_rel",
                            _DEFAULT_TOL_INERTIA_REL)


# =====================================================================
# Отчёт
# =====================================================================

@dataclass
class VerificationReport:
    ok: bool = False
    volume_sim: float = 0.0
    volume_ref: float = 0.0
    volume_delta_rel: float = 0.0
    area_sim: float = 0.0
    area_ref: float = 0.0
    area_delta_rel: float = 0.0
    bbox_sim: tuple = (0.0, 0.0, 0.0)
    bbox_ref: tuple = (0.0, 0.0, 0.0)
    bbox_max_delta: float = 0.0
    inertia_sim: tuple = (0.0, 0.0, 0.0)
    inertia_ref: tuple = (0.0, 0.0, 0.0)
    inertia_max_delta_rel: float = 0.0
    tol_volume_rel: float = _DEFAULT_TOL_VOLUME_REL
    tol_area_rel: float = _DEFAULT_TOL_AREA_REL
    tol_bbox_mm: float = _DEFAULT_TOL_BBOX_MM
    tol_inertia_rel: float = _DEFAULT_TOL_INERTIA_REL
    notes: List[str] = field(default_factory=list)

    def short(self) -> str:
        status = "OK" if self.ok else "MISMATCH"
        return (
            "[verify] {} | "
            "V={:.2f}/{:.2f} (d={:.2f}% <= {:.2f}%) | "
            "A={:.2f}/{:.2f} (d={:.2f}% <= {:.2f}%) | "
            "bboxD={:.2f}mm <= {:.2f} | "
            "ID={:.2f}% <= {:.2f}%"
        ).format(
            status,
            self.volume_sim, self.volume_ref,
            self.volume_delta_rel * 100, self.tol_volume_rel * 100,
            self.area_sim, self.area_ref,
            self.area_delta_rel * 100, self.tol_area_rel * 100,
            self.bbox_max_delta, self.tol_bbox_mm,
            self.inertia_max_delta_rel * 100, self.tol_inertia_rel * 100,
        )

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "volume_sim": self.volume_sim,
            "volume_ref": self.volume_ref,
            "volume_delta_rel": self.volume_delta_rel,
            "area_sim": self.area_sim,
            "area_ref": self.area_ref,
            "area_delta_rel": self.area_delta_rel,
            "bbox_sim": list(self.bbox_sim),
            "bbox_ref": list(self.bbox_ref),
            "bbox_max_delta": self.bbox_max_delta,
            "inertia_sim": list(self.inertia_sim),
            "inertia_ref": list(self.inertia_ref),
            "inertia_max_delta_rel": self.inertia_max_delta_rel,
            "tol_volume_rel": self.tol_volume_rel,
            "tol_area_rel": self.tol_area_rel,
            "tol_bbox_mm": self.tol_bbox_mm,
            "tol_inertia_rel": self.tol_inertia_rel,
            "notes": list(self.notes),
            "short": self.short(),
        }


# =====================================================================
# Вспомогательные
# =====================================================================

def _sorted_bbox_dims(shape) -> Tuple[float, float, float]:
    bb = shape.BoundBox
    dims = sorted([float(bb.XLength), float(bb.YLength), float(bb.ZLength)])
    return (dims[0], dims[1], dims[2])


def _principal_moments(shape) -> Tuple[float, float, float]:
    """Главные моменты инерции (отсортированы по возрастанию).

    Пытается использовать numpy.linalg.eigvalsh, при отсутствии numpy
    возвращает диагональные элементы MatrixOfInertia.
    """
    try:
        m = shape.MatrixOfInertia
        try:
            import numpy as np
            arr = np.array([
                [m.A11, m.A12, m.A13],
                [m.A21, m.A22, m.A23],
                [m.A31, m.A32, m.A33],
            ])
            w = np.linalg.eigvalsh(arr)
            return tuple(sorted([float(x) for x in w]))
        except ImportError:
            return tuple(sorted([float(m.A11), float(m.A22), float(m.A33)]))
    except Exception:
        return (0.0, 0.0, 0.0)


# =====================================================================
# Публичный API
# =====================================================================

def verify_against_reference(sim_shape,
                             ref_shape,
                             logger=None) -> VerificationReport:
    """Сравнить симулированную деталь с реальной.

    Args:
        sim_shape: Part.Shape после симуляции (все гибы).
        ref_shape: Part.Shape Body (реальная деталь).
        logger:    callable(msg).

    Returns:
        VerificationReport.
    """
    _log = logger if callable(logger) else (lambda m: None)
    report = VerificationReport()

    tol_v, tol_a, tol_b, tol_i = _tolerances()
    report.tol_volume_rel = tol_v
    report.tol_area_rel = tol_a
    report.tol_bbox_mm = tol_b
    report.tol_inertia_rel = tol_i

    if not _FC_OK:
        report.notes.append("FreeCAD.Part недоступен (headless)")
        return report

    if sim_shape is None or sim_shape.isNull():
        report.notes.append("sim_shape пустой")
        return report
    if ref_shape is None or ref_shape.isNull():
        report.notes.append("ref_shape пустой")
        return report

    # --- Volume ---
    try:
        report.volume_sim = float(sim_shape.Volume)
        report.volume_ref = float(ref_shape.Volume)
        if report.volume_ref > 1e-9:
            report.volume_delta_rel = (
                abs(report.volume_sim - report.volume_ref)
                / report.volume_ref)
    except Exception as e:
        report.notes.append("volume error: {}".format(e))

    # --- Area ---
    try:
        report.area_sim = float(sim_shape.Area)
        report.area_ref = float(ref_shape.Area)
        if report.area_ref > 1e-9:
            report.area_delta_rel = (
                abs(report.area_sim - report.area_ref)
                / report.area_ref)
    except Exception as e:
        report.notes.append("area error: {}".format(e))

    # --- BBox (sorted dims) ---
    try:
        ds = _sorted_bbox_dims(sim_shape)
        dr = _sorted_bbox_dims(ref_shape)
        report.bbox_sim = ds
        report.bbox_ref = dr
        report.bbox_max_delta = max(
            abs(a - b) for a, b in zip(ds, dr))
    except Exception as e:
        report.notes.append("bbox error: {}".format(e))

    # --- Inertia (principal moments) ---
    try:
        Is = _principal_moments(sim_shape)
        Ir = _principal_moments(ref_shape)
        report.inertia_sim = Is
        report.inertia_ref = Ir
        deltas = []
        for a, b in zip(Is, Ir):
            denom = max(abs(b), 1e-9)
            deltas.append(abs(a - b) / denom)
        report.inertia_max_delta_rel = max(deltas) if deltas else 0.0
    except Exception as e:
        report.notes.append("inertia error: {}".format(e))

    # --- Вердикт ---
    ok_v = report.volume_delta_rel <= tol_v
    ok_a = report.area_delta_rel <= tol_a
    ok_b = report.bbox_max_delta <= tol_b
    ok_i = report.inertia_max_delta_rel <= tol_i
    report.ok = ok_v and ok_a and ok_b and ok_i

    if not ok_v:
        report.notes.append(
            "Объём расходится на {:.2f}% (допуск {:.2f}%)".format(
                report.volume_delta_rel * 100, tol_v * 100))
    if not ok_a:
        report.notes.append(
            "Площадь расходится на {:.2f}% (допуск {:.2f}%)".format(
                report.area_delta_rel * 100, tol_a * 100))
    if not ok_b:
        report.notes.append(
            "Габариты расходятся до {:.2f} мм (допуск {:.2f} мм)".format(
                report.bbox_max_delta, tol_b))
    if not ok_i:
        report.notes.append(
            "Моменты инерции расходятся до {:.2f}% (допуск {:.2f}%)".format(
                report.inertia_max_delta_rel * 100, tol_i * 100))

    _log(report.short())
    for n in report.notes:
        _log("[verify]   {}".format(n))

    return report