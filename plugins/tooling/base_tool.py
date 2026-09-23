# -*- coding: utf-8 -*-
"""
plugins/tooling/base_tool.py
ToolAdapter — позиционер оснастки.

V11.7 (BendSeq):
  * _detect_axes_semantic — геометрическое определение осей по форме.
    UP = ось с наибольшей вариацией площади сечения (носик/ручей vs тело).
    LENGTH = более длинная из двух оставшихся.
    Без привязки к именам объектов.
  * _stretch_shape_to_length — растяжение до длины линии гиба
    (экструзия профиля, без искажений).
  * reference_point для die берётся с extreme="min" (ручей = узкий конец).
  * Ориентация up: от узкого конца (рабочего) к широкому (нерабочему).

V11.6 (BendSeq):
  * role: "punch" | "die" | "gauge" | "auto".
  * tool_use_reference_point: рабочая точка — ЦЕНТР экстремальной грани.
  * tool_trim_mode: "none" | "bbox" | "length_only".
  * _detect_axes_from_bbox: up = СРЕДНЯЯ ось BBox.
  * _crop_shape_local — обрезка до length × cap × cap в ЛОКАЛЬНОЙ системе.
"""

from __future__ import annotations

from typing import Optional, Tuple, Union

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Vector, Matrix
    _FC_OK = True
except ImportError:
    App = None
    Part = None
    Vector = None
    Matrix = None
    _FC_OK = False

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


_TAG = "[TOOL-ADAPTER]"
DirLike = Union["Vector", Tuple[float, float, float]]


def _tlog(msg: str) -> None:
    if not _FC_OK:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_TAG, msg))
    except Exception:
        pass


def _as_dir(value) -> Optional[DirLike]:
    if value is None:
        return None
    if _FC_OK and isinstance(value, Vector):
        return Vector(value)
    try:
        x, y, z = float(value[0]), float(value[1]), float(value[2])
        if _FC_OK:
            return Vector(x, y, z)
        return (x, y, z)
    except (TypeError, ValueError, IndexError, KeyError):
        try:
            x, y, z = float(value.x), float(value.y), float(value.z)
            if _FC_OK:
                return Vector(x, y, z)
            return (x, y, z)
        except (AttributeError, TypeError, ValueError):
            return None


def _ortho_normalize(vec, ref_axis):
    v = Vector(vec)
    v = v - ref_axis * v.dot(ref_axis)
    if v.Length < 1e-9:
        return None
    return v.normalize()


def _vcomp(v, i):
    if i == 0:
        return v.x
    if i == 1:
        return v.y
    return v.z


def _detect_axes_from_bbox(shape):
    """(axis, up) по BBox.

    press-brake инструмент: length = длинная ось, height = средняя,
    thickness = короткая. axis=length, up=medium.
    """
    try:
        bb = shape.BoundBox
        sizes = [
            ("x", float(bb.XLength)),
            ("y", float(bb.YLength)),
            ("z", float(bb.ZLength)),
        ]
        sizes.sort(key=lambda kv: kv[1], reverse=True)
        longest_len = sizes[0][1]
        if longest_len <= 1e-6:
            return Vector(0, 0, 1), Vector(0, 1, 0)
        if (longest_len - sizes[2][1]) / longest_len < 0.05:
            return Vector(0, 0, 1), Vector(0, 1, 0)
        longest = sizes[0][0]
        medium = sizes[1][0]
        vec = {
            "x": Vector(1, 0, 0),
            "y": Vector(0, 1, 0),
            "z": Vector(0, 0, 1),
        }
        return vec[longest], vec[medium]
    except Exception:
        return Vector(0, 0, 1), Vector(0, 1, 0)


# =====================================================================
# Семантический детектор осей (без имён)
# =====================================================================

def _slice_area(shape, axis_vec, pos):
    """Площадь сечения плоскостью axis_vec=pos. 0 при неудаче."""
    if not _FC_OK or shape is None:
        return 0.0
    sec = None
    try:
        sec = shape.slice(Vector(axis_vec), float(pos))
    except Exception:
        sec = None
    if sec is None or sec.isNull():
        try:
            plane = Part.Plane(Vector(axis_vec) * float(pos),
                                Vector(axis_vec))
            sec = shape.section(plane)
        except Exception:
            return 0.0
    if sec is None or sec.isNull():
        return 0.0
    area = 0.0
    try:
        for w in sec.Wires:
            try:
                if not w.isClosed():
                    continue
                f = Part.Face(w)
                if f is not None and not f.isNull():
                    area += float(f.Area)
            except Exception:
                continue
    except Exception:
        pass
    if area > 1e-9:
        return area
    try:
        bb = sec.BoundBox
        return (bb.XLength * bb.YLength
                + bb.YLength * bb.ZLength
                + bb.ZLength * bb.XLength) * 0.5
    except Exception:
        return 0.0


def _axis_variation(shape, axis_vec, bb, axis_name):
    """(ratio, [a_lo, a_mid, a_hi]) вдоль оси. None при неудаче."""
    up_letter = axis_name.upper()
    dim = float(getattr(bb, "{}Length".format(up_letter), 0.0))
    if dim < 1e-6:
        return None
    lo = float(getattr(bb, "{}Min".format(up_letter), 0.0))
    hi = float(getattr(bb, "{}Max".format(up_letter), 0.0))
    p_lo = lo + dim * 0.05
    p_mid = lo + dim * 0.5
    p_hi = hi - dim * 0.05
    areas = [_slice_area(shape, axis_vec, p) for p in (p_lo, p_mid, p_hi)]
    if any(a < 1e-9 for a in areas):
        return None
    denom = max(min(areas[0], areas[2]), 1e-9)
    r = max(areas[0], areas[2]) / denom
    return r, areas


def _detect_axes_semantic(shape, role="auto"):
    """Геометрическое определение осей инструмента.

    UP = ось с наибольшей вариацией площади сечения
        (носик/ручей vs тело/основание).
    LENGTH = более длинная из двух оставшихся.

    Направление UP: от узкого конца (рабочего) к широкому (нерабочему).
    Тогда для punch working point = min по up (носик).
    Для die working point = min по up (ручей).

    Returns:
        (axis_vec, up_vec) или (None, None) если сигнал слабый.
    """
    if not _FC_OK or shape is None:
        return None, None
    try:
        bb = shape.BoundBox
    except Exception:
        return None, None

    candidates = [
        ("x", Vector(1, 0, 0)),
        ("y", Vector(0, 1, 0)),
        ("z", Vector(0, 0, 1)),
    ]

    results = []
    for name, vec in candidates:
        v = _axis_variation(shape, vec, bb, name)
        if v is None:
            continue
        dim = float(getattr(bb, "{}Length".format(name.upper()), 0.0))
        results.append({
            "name": name, "vec": vec, "dim": dim,
            "ratio": v[0], "areas": v[1],
        })

    if len(results) < 2:
        _tlog("semantic: not enough axes resolved")
        return None, None

    results.sort(key=lambda r: -r["ratio"])
    up = results[0]

    if len(results) >= 2:
        ratio_gap = up["ratio"] / max(results[1]["ratio"], 1.001)
        if ratio_gap < 1.5:
            _tlog("semantic: ratio gap weak ({:.2f}); fallback".format(
                ratio_gap))
            return None, None

    remaining = [r for r in results if r["name"] != up["name"]]
    remaining.sort(key=lambda r: -r["dim"])
    length = remaining[0]

    # Направление UP: от узкого конца (min area) к широкому (max area).
    a_lo, a_mid, a_hi = up["areas"]
    if a_lo < a_hi:
        up_dir = Vector(up["vec"])       # узкий на lo -> up от lo к hi
    else:
        up_dir = Vector(up["vec"]) * -1  # узкий на hi -> up от hi к lo

    _tlog("semantic detect: up={} (ratio={:.2f}, areas=[{:.0f},{:.0f},{:.0f}]) "
          "length={} (dim={:.1f}) role={}".format(
              up["name"], up["ratio"],
              a_lo, a_mid, a_hi,
              length["name"], length["dim"], role))

    return Vector(length["vec"]), up_dir


def _stretch_shape_to_length(shape, a_local, target_length):
    """Растянуть shape вдоль a_local до target_length.

    Берём сечение тела плоскостью, перпендикулярной a_local, через
    центр; экструдируем профиль на target_length. Работает для
    призматических инструментов (профиль постоянный по длине).
    """
    if not _FC_OK or shape is None:
        return None
    try:
        target_length = float(target_length)
        if target_length <= 1e-6:
            return None

        a = Vector(a_local)
        if a.Length < 1e-9:
            return None
        a.normalize()

        bb = shape.BoundBox
        center = Vector((bb.XMin + bb.XMax) * 0.5,
                        (bb.YMin + bb.YMax) * 0.5,
                        (bb.ZMin + bb.ZMax) * 0.5)

        section = None
        try:
            section = shape.slice(a, center.dot(a))
        except Exception as e:
            _tlog("stretch: slice failed: {}".format(e))
            section = None
        if section is None or section.isNull():
            try:
                plane = Part.Plane(center, a)
                section = shape.section(plane)
            except Exception as e:
                _tlog("stretch: section failed: {}".format(e))
                section = None
        if section is None or section.isNull():
            _tlog("stretch: section null")
            return None

        faces = []
        try:
            for w in section.Wires:
                try:
                    if not w.isClosed():
                        continue
                    f = Part.Face(w)
                    if f is not None and not f.isNull():
                        faces.append(f)
                except Exception:
                    continue
        except Exception:
            pass

        if not faces:
            _tlog("stretch: no faces")
            return None

        new_solids = []
        for f in faces:
            try:
                s = f.extrude(a * target_length)
                if s is not None and not s.isNull():
                    new_solids.append(s)
            except Exception:
                continue
        if not new_solids:
            return None

        if len(new_solids) == 1:
            stretched = new_solids[0]
        else:
            stretched = Part.makeCompound(new_solids)

        # Центрирование: собственный центр в исходном положении
        try:
            sbb = stretched.BoundBox
            s_center = Vector((sbb.XMin + sbb.XMax) * 0.5,
                              (sbb.YMin + sbb.YMax) * 0.5,
                              (sbb.ZMin + sbb.ZMax) * 0.5)
            # Сместить вдоль a: чтобы центр сечения совпал с center
            delta = a * (center.dot(a) - s_center.dot(a))
            stretched = stretched.translate(delta)
        except Exception:
            pass

        return stretched
    except Exception as e:
        _tlog("stretch failed: {}".format(e))
        return None


def _reference_point_along(shape, axis_dir, extreme="min"):
    """Центр экстремальной грани shape вдоль axis_dir."""
    if not _FC_OK or shape is None:
        return None
    try:
        ax = Vector(axis_dir)
        if ax.Length < 1e-9:
            return None
        ax.normalize()
    except Exception:
        return None

    try:
        bb = shape.BoundBox
    except Exception:
        return None

    center = Vector(
        (bb.XMin + bb.XMax) * 0.5,
        (bb.YMin + bb.YMax) * 0.5,
        (bb.ZMin + bb.ZMax) * 0.5,
    )

    comps = (abs(ax.x), abs(ax.y), abs(ax.z))
    i = 0 if comps[0] >= comps[1] and comps[0] >= comps[2] else (
        1 if comps[1] >= comps[2] else 2)
    sign = (ax.x, ax.y, ax.z)[i]

    lo, hi = ((bb.XMin, bb.XMax),
              (bb.YMin, bb.YMax),
              (bb.ZMin, bb.ZMax))[i]

    if extreme == "min":
        v = lo if sign >= 0 else hi
    else:
        v = hi if sign >= 0 else lo

    if i == 0:
        return Vector(v, center.y, center.z)
    elif i == 1:
        return Vector(center.x, v, center.z)
    else:
        return Vector(center.x, center.y, v)


def _build_rotation_matrix(a_local, u_local, c_local,
                            a_world, u_world, c_world):
    """3x3 matrix R такой что R·a_local = a_world и т.д."""
    basis_w = (a_world, u_world, c_world)
    basis_l = (a_local, u_local, c_local)
    R = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            s = 0.0
            for k in range(3):
                s += _vcomp(basis_w[k], i) * _vcomp(basis_l[k], j)
            R[i][j] = s
    return R


def _crop_shape_local(shape, a_local, u_local, c_local, length, cap):
    """Обрезать shape до параллелепипеда length × cap × cap
    в ЛОКАЛЬНЫХ осях инструмента."""
    if not _FC_OK or shape is None:
        return None

    try:
        ax, ay, az = float(a_local.x), float(a_local.y), float(a_local.z)
        ux, uy, uz = float(u_local.x), float(u_local.y), float(u_local.z)
        cx, cy, cz = float(c_local.x), float(c_local.y), float(c_local.z)
    except Exception:
        return None

    M_to_local = Matrix(
        ax, ux, cx, 0.0,
        ay, uy, cy, 0.0,
        az, uz, cz, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )

    try:
        shape_local = shape.transformGeometry(M_to_local)
    except Exception as e:
        _tlog("_crop: to_local failed: {}".format(e))
        return None
    if shape_local is None or shape_local.isNull():
        return None

    try:
        bb = shape_local.BoundBox
    except Exception:
        return None

    center = Vector(
        (bb.XMin + bb.XMax) * 0.5,
        (bb.YMin + bb.YMax) * 0.5,
        (bb.ZMin + bb.ZMax) * 0.5,
    )

    half_l = float(length) * 0.5
    half_c = float(cap) * 0.5
    origin = Vector(center.x - half_l, center.y - half_c,
                    center.z - half_c)

    try:
        box = Part.makeBox(
            float(length), float(cap), float(cap),
            origin, Vector(0.0, 0.0, 1.0))
    except Exception as e:
        _tlog("_crop: makeBox failed: {}".format(e))
        return None

    try:
        cropped_local = shape_local.common(box)
    except Exception as e:
        _tlog("_crop: common failed: {}".format(e))
        return None

    if cropped_local is None or cropped_local.isNull():
        return None

    try:
        vol = float(cropped_local.Volume)
    except Exception:
        vol = 0.0
    if vol < 1e-6:
        return None

    M_to_world = Matrix(
        ax, ay, az, 0.0,
        ux, uy, uz, 0.0,
        cx, cy, cz, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )
    try:
        cropped_world = cropped_local.transformGeometry(M_to_world)
    except Exception as e:
        _tlog("_crop: to_world failed: {}".format(e))
        return None

    return cropped_world


class ToolAdapter:
    """Позиционер оснастки V11.7."""

    def __init__(self, tool_obj, local_axis=None, local_up=None,
                 role: str = "auto"):
        self.tool_obj = tool_obj
        self.role = (role or SEQUENCE_CONFIG.get(
            "tool_default_role", "auto")).lower()

        self._source_shape = None
        if tool_obj is not None and _FC_OK:
            try:
                sh = tool_obj.Shape
                if sh is not None and not sh.isNull():
                    self._source_shape = sh
            except Exception:
                self._source_shape = None

        # --- Определение осей ---
        if tool_obj is not None and (local_axis is None or local_up is None):
            ref_shape = self._source_shape
            auto_axis = None
            auto_up = None

            # 1. Семантический детектор (по геометрии).
            if ref_shape is not None and _FC_OK:
                try:
                    auto_axis, auto_up = _detect_axes_semantic(
                        ref_shape, role=self.role)
                except Exception as e:
                    _tlog("semantic detect failed: {}".format(e))

            # 2. Fallback — BBox-детектор.
            if auto_axis is None or auto_up is None:
                if ref_shape is not None and _FC_OK:
                    try:
                        auto_axis, auto_up = _detect_axes_from_bbox(ref_shape)
                        _tlog("fallback to bbox detect")
                    except Exception:
                        auto_axis, auto_up = (
                            Vector(0, 0, 1), Vector(0, 1, 0))
                else:
                    auto_axis, auto_up = (
                        Vector(0, 0, 1), Vector(0, 1, 0))

            self.local_axis = (_as_dir(local_axis) if local_axis
                                else auto_axis)
            self.local_up = (_as_dir(local_up) if local_up else auto_up)
        else:
            self.local_axis = (_as_dir(local_axis) or
                                _as_dir((0.0, 0.0, 1.0)))
            self.local_up = (_as_dir(local_up) or
                              _as_dir((0.0, 1.0, 0.0)))

        # --- Reference point ---
        self._ref_point_local = None
        if (self._source_shape is not None
                and bool(SEQUENCE_CONFIG.get(
                    "tool_use_reference_point", True))):
            try:
                up = Vector(self.local_up)
                if up.Length > 1e-9:
                    up.normalize()
                    # up направлен ОТ рабочего конца (узкого) К нерабочему.
                    # Значит рабочий конец = min по up для обеих ролей.
                    if self.role in ("punch", "die"):
                        self._ref_point_local = _reference_point_along(
                            self._source_shape, up, extreme="min")
            except Exception as e:
                _tlog("ref_point failed: {}".format(e))

        rp = self._ref_point_local
        rp_str = ("({:.2f},{:.2f},{:.2f})".format(rp.x, rp.y, rp.z)
                  if rp is not None else "center")
        try:
            bb = self._source_shape.BoundBox if self._source_shape else None
            dims = ("({:.1f},{:.1f},{:.1f})".format(
                bb.XLength, bb.YLength, bb.ZLength)) if bb else "?"
        except Exception:
            dims = "?"
        _tlog("init {}: role={} local_axis={} local_up={} "
              "ref={} bbox_dims={}".format(
                  getattr(tool_obj, "Name", "?"), self.role,
                  self.local_axis, self.local_up, rp_str, dims))

    # ------------------------------------------------------------------

    def make_virtual_shape(self, bend_center, bend_axis, *,
                            up_hint=None, length=None, offset=None):
        """Копия tool_obj.Shape, ориентированная вдоль bend_axis,
        рабочей точкой на bend_center + offset.
        """
        if not _FC_OK or self.tool_obj is None:
            return None

        axis = Vector(bend_axis)
        if axis.Length < 1e-8:
            return None
        axis.normalize()

        a_local = Vector(self.local_axis)
        if a_local.Length < 1e-9:
            a_local = Vector(0, 0, 1)
        a_local.normalize()

        u_local = Vector(self.local_up)
        u_local = u_local - a_local * u_local.dot(a_local)
        if u_local.Length < 1e-9:
            fallback = (Vector(0, 1, 0) if abs(a_local.z) > 0.9
                        else Vector(0, 0, 1))
            u_local = fallback - a_local * fallback.dot(a_local)
        u_local.normalize()
        c_local = a_local.cross(u_local)

        # --- Источник: оригинал / обрезанный / растянутый ---
        source = self._source_shape
        cropped = False

        trim_mode = str(SEQUENCE_CONFIG.get(
            "tool_trim_mode", "none")).lower()
        try:
            cap = float(SEQUENCE_CONFIG.get(
                "tool_max_cross_section_mm", 0.0))
        except (TypeError, ValueError):
            cap = 0.0

        # bbox-crop
        if (trim_mode == "bbox" and cap > 0.0
                and length is not None and float(length) > 1e-6):
            local_len = float(length)
            crop = _crop_shape_local(
                self._source_shape, a_local, u_local, c_local,
                local_len, cap)
            if crop is not None and not crop.isNull():
                source = crop
                cropped = True
                try:
                    bb = source.BoundBox
                    _tlog("crop_local OK: new bbox=({:.1f},{:.1f},{:.1f})"
                          .format(bb.XLength, bb.YLength, bb.ZLength))
                except Exception:
                    pass
            else:
                _tlog("crop_local failed; using original")

        # растяжение (mode="none" или "length_only")
        stretch_enabled = bool(SEQUENCE_CONFIG.get(
            "tool_stretch_to_bend_length", True))
        if (stretch_enabled and trim_mode != "bbox"
                and length is not None and float(length) > 1e-6
                and source is not None):
            try:
                stretch_to = float(length)
                bb = source.BoundBox
                cur_len = (bb.XLength * abs(a_local.x)
                           + bb.YLength * abs(a_local.y)
                           + bb.ZLength * abs(a_local.z))
                if abs(cur_len - stretch_to) > 1.0:
                    stretched = _stretch_shape_to_length(
                        source, a_local, stretch_to)
                    if stretched is not None and not stretched.isNull():
                        source = stretched
                        cropped = True
                        _tlog("stretch OK: {:.1f} -> {:.1f}".format(
                            cur_len, stretch_to))
                    else:
                        _tlog("stretch failed ({} -> {}); using original"
                              .format(cur_len, stretch_to))
            except Exception as e:
                _tlog("stretch exception: {}".format(e))

        # --- Пересчёт ref после обрезки/растяжения ---
        ref = self._ref_point_local
        if cropped:
            try:
                up_vec = Vector(self.local_up)
                if up_vec.Length > 1e-9:
                    up_vec.normalize()
                    if self.role in ("punch", "die"):
                        ref2 = _reference_point_along(
                            source, up_vec, extreme="min")
                        if ref2 is not None:
                            ref = ref2
            except Exception:
                pass

        if ref is None:
            try:
                bb = source.BoundBox
                ref = Vector(
                    (bb.XMin + bb.XMax) * 0.5,
                    (bb.YMin + bb.YMax) * 0.5,
                    (bb.ZMin + bb.ZMax) * 0.5,
                )
            except Exception:
                ref = Vector(0, 0, 0)
        ref = Vector(ref)

        # --- Целевой базис в мире ---
        a_world = axis
        if up_hint is not None and Vector(up_hint).Length > 1e-9:
            uh = Vector(up_hint)
            u_world = uh - a_world * uh.dot(a_world)
        else:
            fb = (Vector(0, 1, 0) if abs(a_world.z) > 0.9
                  else Vector(0, 0, 1))
            u_world = fb - a_world * fb.dot(a_world)
        if u_world.Length < 1e-9:
            fb = (Vector(1, 0, 0) if abs(a_world.x) < 0.9
                  else Vector(0, 1, 0))
            u_world = fb - a_world * fb.dot(a_world)
        u_world.normalize()
        c_world = a_world.cross(u_world)

        _tlog("make_virtual: a_local=({:+.3f},{:+.3f},{:+.3f}) "
              "u_local=({:+.3f},{:+.3f},{:+.3f}) "
              "a_world=({:+.3f},{:+.3f},{:+.3f}) "
              "u_world=({:+.3f},{:+.3f},{:+.3f})".format(
                  a_local.x, a_local.y, a_local.z,
                  u_local.x, u_local.y, u_local.z,
                  a_world.x, a_world.y, a_world.z,
                  u_world.x, u_world.y, u_world.z))

        R = _build_rotation_matrix(a_local, u_local, c_local,
                                    a_world, u_world, c_world)

        R_ref_x = R[0][0]*ref.x + R[0][1]*ref.y + R[0][2]*ref.z
        R_ref_y = R[1][0]*ref.x + R[1][1]*ref.y + R[1][2]*ref.z
        R_ref_z = R[2][0]*ref.x + R[2][1]*ref.y + R[2][2]*ref.z

        target = Vector(bend_center)
        if offset is not None:
            try:
                target = target + Vector(offset)
            except Exception:
                pass

        tx = target.x - R_ref_x
        ty = target.y - R_ref_y
        tz = target.z - R_ref_z

        full = Matrix(
            R[0][0], R[0][1], R[0][2], tx,
            R[1][0], R[1][1], R[1][2], ty,
            R[2][0], R[2][1], R[2][2], tz,
            0.0,     0.0,     0.0,     1.0,
        )

        try:
            bb0 = source.BoundBox
            _tlog("make_virtual: pre-rot bbox=({:.1f},{:.1f},{:.1f}) "
                  "min=({:.1f},{:.1f},{:.1f}) max=({:.1f},{:.1f},{:.1f})".format(
                      bb0.XLength, bb0.YLength, bb0.ZLength,
                      bb0.XMin, bb0.YMin, bb0.ZMin,
                      bb0.XMax, bb0.YMax, bb0.ZMax))
        except Exception:
            pass

        try:
            vs = source.transformGeometry(full)
        except Exception as e:
            _tlog("transformGeometry failed: {}".format(e))
            return None
        if vs is None or vs.isNull():
            _tlog("transformGeometry returned null")
            return None

        try:
            bb1 = vs.BoundBox
            _tlog("make_virtual: post-rot bbox=({:.1f},{:.1f},{:.1f}) "
                  "min=({:.1f},{:.1f},{:.1f}) max=({:.1f},{:.1f},{:.1f})".format(
                      bb1.XLength, bb1.YLength, bb1.ZLength,
                      bb1.XMin, bb1.YMin, bb1.ZMin,
                      bb1.XMax, bb1.YMax, bb1.ZMax))
        except Exception:
            pass

        return vs

    def working_height(self) -> float:
        if self._source_shape is None:
            return 0.0
        try:
            bb = self._source_shape.BoundBox
            up = Vector(self.local_up)
            if up.Length < 1e-9:
                return 0.0
            up.normalize()
            return (bb.XLength * abs(up.x)
                    + bb.YLength * abs(up.y)
                    + bb.ZLength * abs(up.z))
        except Exception:
            return 0.0

    def __repr__(self):
        return "ToolAdapter(obj={}, role={})".format(
            getattr(self.tool_obj, "Name", None), self.role)