# -*- coding: utf-8 -*-
"""
export/report_writer.py
JSON и TXT отчёты по последовательности гибки.

Публичный API:
    write_json_report(state, path, doc_name, tooling_info=None) -> path
    write_txt_report(state, path, doc_name) -> path

bendability = "yes" | "no" | "partial" | "no_data".

Версия: BENDBEQ_REPORT_WRITER_V1
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any, Dict, List, Optional

MODULE_VERSION = "BENDBEQ_REPORT_WRITER_V1"


def _vec_to_list(v):
    if v is None:
        return None
    try:
        return [float(v.x), float(v.y), float(v.z)]
    except (AttributeError, TypeError):
        pass
    try:
        return [float(v[0]), float(v[1]), float(v[2])]
    except (TypeError, IndexError, ValueError):
        return None


def _summarize_shape(shape) -> Dict[str, Any]:
    if shape is None:
        return {"present": False}
    try:
        if shape.isNull():
            return {"present": True, "is_null": True}
    except Exception:
        return {"present": True, "is_null": "unknown"}

    info: Dict[str, Any] = {
        "present": True, "is_null": False,
        "volume_mm3": None, "surface_area_mm2": None,
        "edge_count": None, "face_count": None, "solid_count": None,
        "bbox": None, "is_valid": None,
    }
    try:
        info["volume_mm3"] = float(shape.Volume)
    except Exception:
        pass
    try:
        info["surface_area_mm2"] = float(shape.Area)
    except Exception:
        pass
    try:
        info["edge_count"] = len(shape.Edges)
        info["face_count"] = len(shape.Faces)
        info["solid_count"] = len(shape.Solids)
    except Exception:
        pass
    try:
        bb = shape.BoundBox
        info["bbox"] = {
            "x": [float(bb.XMin), float(bb.XMax)],
            "y": [float(bb.YMin), float(bb.YMax)],
            "z": [float(bb.ZMin), float(bb.ZMax)],
            "diagonal_mm": float(bb.DiagonalLength),
        }
    except Exception:
        pass
    try:
        info["is_valid"] = bool(shape.isValid())
    except Exception:
        pass
    return info


def _safe_serialize(obj, _depth: int = 0, _seen=None):
    if _depth > 12:
        return "<max-depth>"
    if _seen is None:
        _seen = set()
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj

    oid = id(obj)
    if oid in _seen:
        return "<cycle>"

    if not isinstance(obj, (list, tuple, set, frozenset, dict)):
        vec = _vec_to_list(obj)
        if vec is not None:
            return vec

    if isinstance(obj, BaseException):
        return {"type": type(obj).__name__, "message": str(obj)}

    if isinstance(obj, (list, tuple, set, frozenset)):
        _seen.add(oid)
        out = [_safe_serialize(x, _depth + 1, _seen) for x in obj]
        _seen.discard(oid)
        return out

    if isinstance(obj, dict):
        _seen.add(oid)
        out = {str(k): _safe_serialize(v, _depth + 1, _seen)
                for k, v in obj.items()}
        _seen.discard(oid)
        return out

    try:
        if hasattr(obj, "ShapeType"):
            return _summarize_shape(obj)
    except Exception:
        pass

    if hasattr(obj, "id") and hasattr(obj, "center"):
        return _serialize_bend_spec(obj)

    try:
        r = repr(obj)
        return r if len(r) <= 500 else r[:500] + "…"
    except Exception:
        return "<unserializable>"


def _serialize_bend_spec(bend) -> Optional[Dict[str, Any]]:
    if bend is None:
        return None
    return {
        "id": getattr(bend, "id", None),
        "name": getattr(bend, "name", None),
        "angle_deg": float(getattr(bend, "angle", 0.0) or 0.0),
        "length_mm": float(getattr(bend, "length", 0.0) or 0.0),
        "direction": getattr(bend, "direction", "up"),
        "center": _vec_to_list(getattr(bend, "center", None)),
        "axis": _vec_to_list(getattr(bend, "axis", None)),
        "normal": _vec_to_list(getattr(bend, "normal", None)),
        "up_hint": _vec_to_list(getattr(bend, "up_hint", None)),
        "metadata": _safe_serialize(
            getattr(bend, "metadata", {}) or {}),
    }


def _serialize_step_result(res) -> Dict[str, Any]:
    details = getattr(res, "details", {}) or {}
    min_d = getattr(res, "min_distance", None)
    return {
        "step": getattr(res, "step_number", None),
        "bend_id": getattr(res, "bend_id", None),
        "ok": bool(getattr(res, "ok", False)),
        "collision_type": getattr(res, "collision_type", None),
        "reason": getattr(res, "reason_human", ""),
        "volume_mm3": float(getattr(res, "volume", 0.0) or 0.0),
        "is_contact": bool(getattr(res, "is_contact", False)),
        "min_distance_mm": float(min_d) if min_d is not None else None,
        "details": _safe_serialize(details),
        "shape_after": _summarize_shape(
            getattr(res, "shape_after", None)),
        "bend_spec_source": _serialize_bend_spec(
            getattr(res, "bend_spec", None)),
        "bend_spec_working": _serialize_bend_spec(
            getattr(res, "working_bend_spec", None)),
    }


def _ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        try:
            os.makedirs(parent, exist_ok=True)
        except Exception:
            pass


def _build_report_dict(state, doc_name: str,
                        tooling_info=None) -> Dict[str, Any]:
    step_results = list(getattr(state, "step_results", []) or [])
    all_bends = list(getattr(state, "all_bends", []) or [])
    ordered_ids = list(getattr(state, "ordered_ids", []) or [])

    ok_count = sum(1 for r in step_results if getattr(r, "ok", False))
    fail_count = len(step_results) - ok_count
    contacts = sum(1 for r in step_results
                    if getattr(r, "is_contact", False))

    collision_counts: Dict[str, int] = {}
    for r in step_results:
        ct = getattr(r, "collision_type", None)
        if ct:
            collision_counts[ct] = collision_counts.get(ct, 0) + 1

    try:
        is_complete = bool(state.is_complete())
    except Exception:
        is_complete = None

    final_shape = (step_results[-1].shape_after if step_results
                    else getattr(state, "flat_shape", None))

    if not step_results:
        bendability = "no_data"
        physically_bendable = False
        bendability_note = ("Нет данных валидации: "
                             "последовательность не проверялась.")
    elif not is_complete:
        bendability = "partial"
        physically_bendable = False
        bendability_note = "Последовательность неполная — не все гибы."
    elif ok_count != len(step_results):
        bendability = "no"
        physically_bendable = False
        bad = [getattr(r, "bend_id", "?")
                for r in step_results if not getattr(r, "ok", False)]
        bendability_note = (
            "Коллизии на шагах: {}. Нельзя согнуть в этой "
            "последовательности.".format(", ".join(str(b) for b in bad)))
    else:
        bendability = "yes"
        physically_bendable = True
        bendability_note = (
            "Все шаги без коллизий. Проверено: подвод пуансона, "
            "столкновения с пуансоном/матрицей/станиной/упором, "
            "самопересечение, валидность топологии.")
        if contacts:
            bendability_note += (" ВНИМАНИЕ: есть касания инструмента "
                                  "с деталью — проверьте зазоры.")

    summary = {
        "total_steps": len(step_results),
        "ok_steps": ok_count,
        "failed_steps": fail_count,
        "steps_with_contact": contacts,
        "collision_type_counts": collision_counts,
        "is_complete": is_complete,
        "is_partial": (not is_complete) and len(step_results) > 0,
        "all_valid": (is_complete is True and len(step_results) > 0
                       and ok_count == len(step_results)),
        "all_valid_relaxed": (len(step_results) > 0
                               and ok_count == len(step_results)),
        "physically_bendable": physically_bendable,
        "bendability": bendability,
        "bendability_note": bendability_note,
        "final_shape": _summarize_shape(final_shape),
    }

    return {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "module_version": MODULE_VERSION,
        "part_name": doc_name,
        "summary": summary,
        "tooling": _safe_serialize(tooling_info or {}),
        "ordered_bend_ids": ordered_ids,
        "all_bends_source": [_serialize_bend_spec(b) for b in all_bends],
        "flat_shape": _summarize_shape(
            getattr(state, "flat_shape", None)),
        "sequence": [_serialize_step_result(r) for r in step_results],
    }


def write_json_report(state, output_path, doc_name="SheetMetalPart",
                       tooling_info=None) -> str:
    report = _build_report_dict(state, doc_name, tooling_info)
    _ensure_parent_dir(output_path)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    return output_path


def write_txt_report(state, output_path,
                      doc_name="SheetMetalPart") -> str:
    report = _build_report_dict(state, doc_name)
    summary = report["summary"]
    lines = []

    lines.append("=" * 72)
    lines.append(" ОТЧЁТ ПО ПОСЛЕДОВАТЕЛЬНОСТИ ГИБКИ — {}".format(doc_name))
    lines.append("=" * 72)
    lines.append("Сгенерировано:    {}".format(report["generated_at"]))
    lines.append("Версия модуля:    {}".format(report["module_version"]))
    lines.append("Всего шагов:      {}".format(summary["total_steps"]))
    lines.append("Успешных:         {}".format(summary["ok_steps"]))
    lines.append("Провалившихся:    {}".format(summary["failed_steps"]))
    lines.append("С касаниями:      {}".format(
        summary["steps_with_contact"]))
    lines.append("Полная?:          {}".format(summary["is_complete"]))
    lines.append("Валидна?:         {}".format(summary["all_valid"]))
    lines.append("-" * 72)

    bendability = summary.get("bendability", "no_data")
    lines.append("")
    if bendability == "yes":
        lines.append("=" * 72)
        lines.append(" ИТОГ: МОЖНО СОГНУТЬ НА СТАНКЕ")
        lines.append("=" * 72)
        lines.append(" Все гибы добавлены, все шаги прошли без коллизий.")
    elif bendability == "no":
        lines.append("=" * 72)
        lines.append(" ИТОГ: НЕЛЬЗЯ СОГНУТЬ В ЭТОЙ ПОСЛЕДОВАТЕЛЬНОСТИ")
        lines.append("=" * 72)
        lines.append(" Есть коллизии на отдельных шагах. См. ниже.")
    elif bendability == "partial":
        lines.append("=" * 72)
        lines.append(" ИТОГ: НЕПОЛНАЯ ПОСЛЕДОВАТЕЛЬНОСТЬ")
        lines.append("=" * 72)
        lines.append(" Добавлены не все гибы.")
    else:
        lines.append("=" * 72)
        lines.append(" ИТОГ: НЕТ ДАННЫХ ВАЛИДАЦИИ")
        lines.append("=" * 72)
        lines.append(" Последовательность не проверялась.")
    lines.append(" {}".format(summary.get("bendability_note", "")))
    lines.append("=" * 72)
    lines.append("")

    for step in report["sequence"]:
        status = "OK  " if step["ok"] else "FAIL"
        contact_tag = " contact" if step.get("is_contact") else ""
        lines.append("[{:>3}] {} {:<12} type={:<26} V={:.4f}{}".format(
            step["step"], status, str(step["bend_id"]),
            step["collision_type"] or "—", step["volume_mm3"],
            contact_tag))
        lines.append("       {}".format(step["reason"]))

    lines.append("=" * 72)
    _ensure_parent_dir(output_path)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    return output_path
