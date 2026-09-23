# -*- coding: utf-8 -*-
"""
core/unfold_extractor.py
Извлечение 11 панелей и 10 hinge-линий из SheetMetal API (getUnfold).

Публичный API:
    extract_unfold_data(body, face_name, ...) -> (panels, hinges, unfolded)
    extract_unfold_data_ex(body, face_name, ...) -> dict с diagnostics

Версия: BENDBEQ_UNFOLD_EXTRACTOR_V1
"""

from __future__ import annotations

from typing import List, Optional, Tuple, Dict, Any

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector
    PART_OK = True
except ImportError:
    App = None
    Part = None
    Vector = None
    PART_OK = False

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


_SM_UNFOLDER_IMPORTED = False
_getUnfold = None
_BendAllowanceCalculator = None
_SplitAPI = None


def _try_import_sm():
    global _SM_UNFOLDER_IMPORTED, _getUnfold, _BendAllowanceCalculator, _SplitAPI
    if _SM_UNFOLDER_IMPORTED:
        return True
    try:
        from SheetMetalNewUnfolder import (getUnfold,
                                            BendAllowanceCalculator)
        from BOPTools import SplitAPI
        _getUnfold = getUnfold
        _BendAllowanceCalculator = BendAllowanceCalculator
        _SplitAPI = SplitAPI
        _SM_UNFOLDER_IMPORTED = True
        return True
    except ImportError:
        return False


DEFAULT_K_FACTOR = 0.5
DEFAULT_MIN_AREA_MM2 = 50.0
CUTTER_Z_LO = -1.0
CUTTER_Z_HI = 3.0
CUTTER_EXTEND_MM = 1000.0


def _min_panel_volume(thickness_mm, min_area_mm2=None):
    if min_area_mm2 is None:
        min_area_mm2 = float(SEQUENCE_CONFIG.get(
            "unfold_min_panel_area_mm2", DEFAULT_MIN_AREA_MM2))
    if thickness_mm is None or thickness_mm <= 1e-6:
        return 100.0
    return float(min_area_mm2) * float(thickness_mm)


def _thickness_of_body(body):
    if body is None:
        return None
    try:
        group = list(getattr(body, "Group", []) or [])
    except Exception:
        return None
    for obj in group:
        try:
            if not obj.isDerivedFrom("PartDesign::Pad"):
                continue
            if not hasattr(obj, "Shape") or obj.Shape.isNull():
                continue
            bb = obj.Shape.BoundBox
            dims = sorted([bb.XLength, bb.YLength, bb.ZLength])
            t = float(dims[0])
            if 0.1 < t < 50.0:
                return t
        except Exception:
            continue
    return None


def _make_cutters(hinges):
    cutters = []
    for e in hinges:
        try:
            p1 = e.valueAt(e.FirstParameter)
            p2 = e.valueAt(e.LastParameter)
        except Exception:
            continue
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        L = (dx * dx + dy * dy) ** 0.5
        if L < 1e-6:
            continue
        ux, uy = dx / L, dy / L
        ex1 = p1.x - ux * CUTTER_EXTEND_MM
        ey1 = p1.y - uy * CUTTER_EXTEND_MM
        ex2 = p2.x + ux * CUTTER_EXTEND_MM
        ey2 = p2.y + uy * CUTTER_EXTEND_MM
        try:
            wire = Part.makePolygon([
                Vector(ex1, ey1, CUTTER_Z_LO),
                Vector(ex2, ey2, CUTTER_Z_LO),
                Vector(ex2, ey2, CUTTER_Z_HI),
                Vector(ex1, ey1, CUTTER_Z_HI),
                Vector(ex1, ey1, CUTTER_Z_LO),
            ])
            cutters.append(Part.Face(wire))
        except Exception:
            continue
    return cutters


def _filter_panels(solids, min_volume):
    kept = []
    dropped = []
    all_valid = []
    for s in solids:
        if s is None or s.isNull():
            continue
        all_valid.append(s)
        try:
            v = float(s.Volume)
        except Exception:
            v = 0.0
        if v >= min_volume:
            kept.append(s)
        else:
            dropped.append(s)
    return kept, dropped, all_valid


def extract_unfold_data(body, face_name, k_factor=DEFAULT_K_FACTOR,
                         thickness_mm=None, min_area_mm2=None):
    d = extract_unfold_data_ex(body, face_name, k_factor=k_factor,
                                thickness_mm=thickness_mm,
                                min_area_mm2=min_area_mm2)
    return d["panels"], d["hinges"], d["unfolded"]


def extract_unfold_data_ex(body, face_name, k_factor=DEFAULT_K_FACTOR,
                            thickness_mm=None, min_area_mm2=None):
    diag = {
        "reason": "init",
        "sm_api": False,
        "k_factor": float(k_factor),
        "thickness_mm": None,
        "min_volume_mm3": None,
        "min_area_mm2": (float(min_area_mm2)
                          if min_area_mm2 is not None
                          else float(SEQUENCE_CONFIG.get(
                              "unfold_min_panel_area_mm2",
                              DEFAULT_MIN_AREA_MM2))),
        "n_solids_raw": 0,
        "n_panels_kept": 0,
        "n_panels_dropped": 0,
        "dropped_volumes": [],
        "slice_method": "none",
        "errors": [],
    }
    result = {"ok": False, "panels": [], "hinges": [],
              "unfolded": None, "diagnostics": diag}

    if not PART_OK:
        diag["reason"] = "no_freecad_part"
        diag["errors"].append("FreeCAD.Part недоступен")
        return result
    if body is None:
        diag["reason"] = "no_body"
        diag["errors"].append("body is None")
        return result
    if not face_name:
        diag["reason"] = "no_face_name"
        diag["errors"].append("face_name пустой")
        return result
    if not _try_import_sm():
        diag["reason"] = "no_sm_api"
        diag["errors"].append(
            "SheetMetalNewUnfolder / BOPTools.SplitAPI недоступны")
        return result
    diag["sm_api"] = True

    if thickness_mm is None:
        thickness_mm = _thickness_of_body(body)
    diag["thickness_mm"] = float(thickness_mm) if thickness_mm else None

    min_vol = _min_panel_volume(thickness_mm, min_area_mm2)
    diag["min_volume_mm3"] = float(min_vol)

    try:
        bac = _BendAllowanceCalculator.from_single_value(
            float(k_factor), "ansi")
        sel_face, unfolded, bend_compound, root_normal, bend_infodata = (
            _getUnfold(bac, body, face_name))
    except Exception as e:
        diag["reason"] = "getUnfold_failed"
        diag["errors"].append("getUnfold: {}: {}".format(type(e).__name__, e))
        return result

    if unfolded is None or unfolded.isNull():
        diag["reason"] = "unfolded_null"
        diag["errors"].append("unfolded пуст")
        return result

    try:
        hinges = list(bend_compound.Edges or [])
    except Exception:
        hinges = []

    if not hinges:
        diag["reason"] = "no_hinges_partial"
        diag["errors"].append("bend_compound без Edges")
        result["unfolded"] = unfolded
        try:
            all_solids = list(unfolded.Solids or [])
        except Exception:
            all_solids = []
        kept, dropped, all_valid = _filter_panels(all_solids, min_vol)
        diag["n_solids_raw"] = len(all_valid)
        diag["n_panels_kept"] = len(kept)
        diag["n_panels_dropped"] = len(dropped)
        diag["dropped_volumes"] = [
            round(float(getattr(s, "Volume", 0.0) or 0.0), 4)
            for s in dropped]
        result["panels"] = kept or ([unfolded] if not kept else [])
        result["ok"] = bool(result["panels"])
        return result

    result["unfolded"] = unfolded
    result["hinges"] = hinges

    cutters = _make_cutters(hinges)
    if not cutters:
        diag["reason"] = "no_cutters"
        diag["errors"].append("не удалось построить режущие плоскости")
        try:
            all_solids = list(unfolded.Solids or [])
        except Exception:
            all_solids = []
        kept, dropped, all_valid = _filter_panels(all_solids, min_vol)
        diag["n_solids_raw"] = len(all_valid)
        diag["n_panels_kept"] = len(kept)
        diag["n_panels_dropped"] = len(dropped)
        result["panels"] = kept or ([unfolded] if not kept else [])
        result["ok"] = bool(result["panels"])
        return result

    all_solids = []
    method = "none"

    try:
        slice_result = _SplitAPI.slice(unfolded, cutters, "Split")
        if slice_result is not None and not slice_result.isNull():
            all_solids = list(slice_result.Solids or [])
            if all_solids:
                method = "splitapi"
    except Exception as e:
        diag["errors"].append(
            "SplitAPI.slice: {}: {}".format(type(e).__name__, e))

    if not all_solids or len(all_solids) < 2:
        try:
            cutter_compound = Part.makeCompound(cutters)
            split_result = unfolded.split(cutter_compound)
            cands = []
            if hasattr(split_result, "Solids"):
                cands = list(split_result.Solids or [])
            elif isinstance(split_result, (list, tuple)):
                for part in split_result:
                    if part is None or part.isNull():
                        continue
                    if hasattr(part, "Solids"):
                        cands.extend(list(part.Solids or []))
            if cands:
                all_solids = cands
                method = "shape_split"
        except Exception as e:
            diag["errors"].append(
                "unfolded.split: {}: {}".format(type(e).__name__, e))

    if not all_solids:
        try:
            all_solids = list(unfolded.Solids or [])
        except Exception:
            all_solids = []
        if all_solids:
            method = "no_split"
            diag["errors"].append("SplitAPI и split не сработали, "
                                    "возвращаю unfolded как одну панель")

    kept, dropped, all_valid = _filter_panels(all_solids, min_vol)

    diag["n_solids_raw"] = len(all_valid)
    diag["n_panels_kept"] = len(kept)
    diag["n_panels_dropped"] = len(dropped)
    diag["dropped_volumes"] = [
        round(float(getattr(s, "Volume", 0.0) or 0.0), 4) for s in dropped]
    diag["slice_method"] = method

    if not kept and all_valid:
        diag["errors"].append(
            "все {} solids ниже min_volume={:.2f} — "
            "возвращаю unfolded целиком".format(len(all_valid), min_vol))
        kept = [unfolded]

    result["panels"] = kept
    result["ok"] = bool(kept) and len(kept) >= 1
    diag["reason"] = "ok" if result["ok"] else "no_panels"
    return result


def describe_extraction(d):
    diag = d.get("diagnostics", {})
    lines = [
        "Unfold extraction:",
        "  ok:               {}".format(d.get("ok")),
        "  reason:           {}".format(diag.get("reason")),
        "  sm_api:           {}".format(diag.get("sm_api")),
        "  slice_method:     {}".format(diag.get("slice_method")),
        "  k_factor:         {}".format(diag.get("k_factor")),
        "  thickness_mm:     {}".format(diag.get("thickness_mm")),
        "  min_area_mm2:     {}".format(diag.get("min_area_mm2")),
        "  min_volume_mm3:   {}".format(diag.get("min_volume_mm3")),
        "  n_solids_raw:     {}".format(diag.get("n_solids_raw")),
        "  n_panels_kept:    {}".format(diag.get("n_panels_kept")),
        "  n_panels_dropped: {}".format(diag.get("n_panels_dropped")),
        "  n_hinges:         {}".format(len(d.get("hinges") or [])),
    ]
    dropped = diag.get("dropped_volumes") or []
    if dropped:
        lines.append("  dropped_volumes:  {}".format(dropped))
    errors = diag.get("errors") or []
    if errors:
        lines.append("  errors:")
        for e in errors:
            lines.append("    - {}".format(e))
    return "\n".join(lines)
