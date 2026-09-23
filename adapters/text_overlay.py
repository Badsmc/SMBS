# -*- coding: utf-8 -*-
"""
adapters/text_overlay.py
SequenceTextOverlay — номера шагов гибки на плоской развёртке.

Позиционирование задаётся извне через set_bend_positions(dict).
Внутренний fallback: bend.center + (0,0,1).

Текст:
  1. Draft.makeShapeString (с центрированием по BBox).
  2. 7-сегментные цифры (если нет шрифта).
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, Optional

try:
    import FreeCAD as App
    import Part
    _FC_OK = True
except ImportError:
    App = None
    Part = None
    _FC_OK = False

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {"char_height": 15.0, "overlay_offset_mm": 25.0}


DEFAULT_FONT_SIZE_MM = 15.0


_FONT_CANDIDATES = [
    "/System/Library/Fonts/Supplemental/Arial.ttf",
    "/System/Library/Fonts/Supplemental/Helvetica.ttf",
    "/System/Library/Fonts/Supplemental/Verdana.ttf",
    "/System/Library/Fonts/Supplemental/Courier New.ttf",
    "/System/Library/Fonts/SFNSMono.ttf",
    "/System/Library/Fonts/SFNS.ttf",
    "/Library/Fonts/Arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    r"C:\Windows\Fonts\arial.ttf",
    r"C:\Windows\Fonts\calibri.ttf",
]


def _find_default_font():
    for p in _FONT_CANDIDATES:
        try:
            if os.path.isfile(p):
                return p
        except Exception:
            continue
    return None


# =====================================================================
# 7-сегментные цифры (fallback)
# =====================================================================

_SEGMENTS = {
    0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg",
    5: "acdfg", 6: "acdefg", 7: "abc", 8: "abcdefg", 9: "abcdfg",
}


def _h_seg(x1, x2, y, t):
    return [(x1, y - t / 2), (x2, y - t / 2),
            (x2, y + t / 2), (x1, y + t / 2)]


def _v_seg(x, y1, y2, t):
    return [(x - t / 2, y1), (x + t / 2, y1),
            (x + t / 2, y2), (x - t / 2, y2)]


def _digit_polys(digit, W, H, t):
    s = _SEGMENTS.get(digit, "")
    p = []
    if "a" in s: p.append(_h_seg(0.1 * W, 0.9 * W, H, t))
    if "g" in s: p.append(_h_seg(0.1 * W, 0.9 * W, H / 2, t))
    if "d" in s: p.append(_h_seg(0.1 * W, 0.9 * W, 0, t))
    if "f" in s: p.append(_v_seg(0, H / 2, H, t))
    if "e" in s: p.append(_v_seg(0, 0, H / 2, t))
    if "b" in s: p.append(_v_seg(W, H / 2, H, t))
    if "c" in s: p.append(_v_seg(W, 0, H / 2, t))
    return p


def _make_7seg_compound(text, char_h):
    if not _FC_OK:
        return None
    W, H, t = 0.6 * char_h, char_h, 0.15 * char_h
    gap = 0.2 * W
    digits = [int(ch) for ch in str(text) if ch.isdigit()]
    if not digits:
        return None
    n = len(digits)
    x0 = -(n * W + (n - 1) * gap) / 2.0
    y0 = -H / 2.0
    faces = []
    for i, d in enumerate(digits):
        for poly in _digit_polys(d, W, H, t):
            pts = [App.Vector(x0 + i * (W + gap) + p[0],
                              y0 + p[1], 0.0) for p in poly]
            pts.append(pts[0])
            try:
                faces.append(Part.Face(Part.makePolygon(pts)))
            except Exception:
                continue
    if not faces:
        return None
    try:
        return Part.makeCompound(faces)
    except Exception:
        return None


# =====================================================================
# ShapeString
# =====================================================================

def _make_shapestring(doc, text, size_mm, font_path):
    """Draft.ShapeString, центрированный по bounding box.

    После создания сдвигаем Shape так, чтобы bb.Center = (0,0,0).
    Тогда Placement.Base точно указывает центр текста.
    """
    try:
        import Draft
    except ImportError:
        return None
    if not hasattr(Draft, "makeShapeString"):
        return None

    obj = None
    try:
        obj = Draft.makeShapeString(
            String=str(text), FontFile=font_path or "",
            Size=float(size_mm), Tracking=0)
    except Exception:
        return None
    if obj is None:
        return None

    try:
        sh = obj.Shape
        if sh is None or sh.isNull():
            raise ValueError("null")
        bb = sh.BoundBox
        if not (math.isfinite(bb.XLength) and math.isfinite(bb.YLength)):
            raise ValueError("non-finite")
        if bb.XLength < 1e-6 or bb.YLength < 1e-6:
            raise ValueError("tiny")
    except Exception:
        try:
            doc.removeObject(obj.Name)
        except Exception:
            pass
        return None

    # Центрируем форму: bb.Center -> (0,0,0)
    try:
        center = App.Vector(
            (bb.XMin + bb.XMax) * 0.5,
            (bb.YMin + bb.YMax) * 0.5,
            (bb.ZMin + bb.ZMax) * 0.5)
        new_shape = sh.copy()
        new_shape.translate(-center)
        obj.Shape = new_shape
    except Exception:
        pass

    return obj


# =====================================================================
# Логи
# =====================================================================

def _log(msg):
    if not _FC_OK:
        return
    try:
        App.Console.PrintMessage("[overlay] {}\n".format(msg))
    except Exception:
        pass


def _dbg():
    try:
        return bool(SEQUENCE_CONFIG.get("overlay_debug", False))
    except Exception:
        return False


# =====================================================================
# Overlay
# =====================================================================

class SequenceTextOverlay:

    GROUP_NAME = "Bend_Sequence_Text"

    def __init__(self, parent_unfold=None, panel_graph_2d=None,
                 font_size_mm=None, font_path=None):
        self.parent_unfold = parent_unfold
        self.panel_graph_2d = panel_graph_2d

        # Позиции, задаваемые снаружи: bend_id -> App.Vector
        self._bend_positions = {}

        cfg_size = float(SEQUENCE_CONFIG.get(
            "char_height", DEFAULT_FONT_SIZE_MM))
        self._font_size_mm = float(
            font_size_mm if font_size_mm is not None else cfg_size)
        self._font_path = font_path or _find_default_font()
        self._last_previews = None

        _log("init: font_size={:.1f} mm, font_path={}".format(
            self._font_size_mm, self._font_path or "<none>"))

    # ------------------------------------------------------------------

    def set_panel_graph_2d(self, pg2):
        self.panel_graph_2d = pg2

    def set_bend_positions(self, bend_id_to_pos):
        """bend_id -> мировая позиция (App.Vector) для номера."""
        self._bend_positions = dict(bend_id_to_pos) if bend_id_to_pos else {}
        _log("set_bend_positions: {} entries".format(
            len(self._bend_positions)))
        if _dbg():
            for k, v in list(self._bend_positions.items())[:5]:
                _log("  {} -> ({:.2f},{:.2f},{:.2f})".format(
                    k, v.x, v.y, v.z))

    def set_font_size(self, size_mm):
        new_size = max(1.0, min(200.0, float(size_mm)))
        if abs(new_size - self._font_size_mm) < 1e-9:
            return
        self._font_size_mm = new_size
        _log("set_font_size({:.1f} mm)".format(new_size))
        if self._last_previews:
            self.draw_numbers(self._last_previews)

    def get_font_size(self):
        return self._font_size_mm

    def clear(self):
        if not _FC_OK:
            return
        doc = App.ActiveDocument
        if not doc:
            return
        group = doc.getObject(self.GROUP_NAME)
        if group is None:
            return
        for child in list(getattr(group, "Group", [])):
            try:
                doc.removeObject(child.Name)
            except Exception:
                pass
        try:
            doc.removeObject(group.Name)
        except Exception:
            pass
        try:
            doc.recompute()
        except Exception:
            pass

    # ------------------------------------------------------------------

    def draw_numbers(self, step_results, bend_id_to_face_pair=None):
        if not _FC_OK:
            return 0
        doc = App.ActiveDocument
        if not doc or not step_results:
            return 0

        self._last_previews = list(step_results)

        _log("draw_numbers: previews={}, positions={}".format(
            len(step_results), len(self._bend_positions)))

        self.clear()

        group = None
        try:
            group = doc.addObject(
                "App::DocumentObjectGroup", self.GROUP_NAME)
            group.Label = "Bend Sequence (Numbers)"
        except Exception as e:
            _log("group failed: {}".format(e))

        placed = 0
        shape_ok = 0
        fallback_ok = 0

        for item in step_results:
            bend = getattr(item, "bend_spec", None)
            if not bend:
                continue
            ok = bool(getattr(item, "ok", True))
            step_num = int(getattr(item, "step_number", 1))
            bend_id = getattr(bend, "id", None)

            pos = self._compute_position(bend, bend_id)

            if _dbg():
                _log("  step {} id={} pos=({:.3f},{:.3f},{:.3f})".format(
                    step_num, bend_id,
                    float(pos.x), float(pos.y), float(pos.z)))

            name = "bs_step_{:02d}".format(step_num)
            obj = None
            if self._font_path:
                obj = _make_shapestring(
                    doc, str(step_num), self._font_size_mm,
                    self._font_path)
                if obj is not None:
                    try:
                        obj.Name = name
                    except Exception:
                        pass
                    shape_ok += 1

            if obj is None:
                comp = _make_7seg_compound(
                    str(step_num), self._font_size_mm)
                if comp is None:
                    continue
                try:
                    obj = doc.addObject("Part::Feature", name)
                    obj.Shape = comp
                    fallback_ok += 1
                except Exception:
                    continue

            try:
                obj.Placement = App.Placement(pos, App.Rotation())
            except Exception:
                pass

            try:
                if getattr(obj, "ViewObject", None):
                    obj.ViewObject.Visibility = True
                    obj.ViewObject.ShapeColor = (
                        (0.0, 0.8, 0.0) if ok else (0.9, 0.1, 0.1))
            except Exception:
                pass

            try:
                obj.Label = "step_{}_{}".format(step_num, bend_id or "")
            except Exception:
                pass

            if group is not None:
                try:
                    group.addObject(obj)
                except Exception:
                    pass
            placed += 1

        try:
            doc.recompute()
        except Exception:
            pass

        _log("draw_numbers: placed={} (shapestring={}, 7seg={})".format(
            placed, shape_ok, fallback_ok))
        return placed

    def _compute_position(self, bend, bend_id):
        # 1. Из task_panel (точная позиция)
        if self._bend_positions and bend_id in self._bend_positions:
            return self._bend_positions[bend_id]

        # 2. Fallback: bend.center + 1 по Z
        c = getattr(bend, "center", None)
        if c is None:
            return App.Vector(0, 0, 0)
        return App.Vector(float(c.x), float(c.y), float(c.z) + 1.0)