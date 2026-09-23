# -*- coding: utf-8 -*-
"""
ui/simulation_viewer.py
SimulationController — 3D-симуляция последовательности гибки.

V9 (BendSeq):
  * SimPunch/SimDie ориентируются по bend.normal (нормаль листа),
    НЕ по bend.up_hint (который = нормаль плоскости разреза).
  * Расширенная диагностика: bbox после каждого инструмента.

V8 (BendSeq):
  * Build-шаг: создаёт SimPart_i, SimPunch_i, SimDie_i, SimGauge_i,
    SimPress_j, SimCollision_i в отдельной группе BendSimulation.
  * Кинематический режим: если передан KinematicEngine-совместимый
    контекст (unfold_panels + bend_features), строит состояния через
    киниматику; иначе — использует shape_after из step_results.
  * Moment mode: пересчитывает активный шаг с angle_factor=0.5.
"""

from __future__ import annotations

import FreeCAD as App
import FreeCADGui as Gui

try:
    from core.bend_simulator import simulate_bend_occ
except ImportError:
    simulate_bend_occ = None

try:
    from core.bend_kinematics import BendKinematics
except ImportError:
    BendKinematics = None

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


DIR_DIAG = True
_DIR_PREFIX = "[SIM-VIEW]"


def _dird(msg: str) -> None:
    if not DIR_DIAG:
        return
    try:
        App.Console.PrintMessage("{} {}\n".format(_DIR_PREFIX, msg))
    except Exception:
        pass


def _fmt_v(v, nd: int = 3) -> str:
    try:
        return "({:.{}f},{:.{}f},{:.{}f})".format(
            float(v.x), nd, float(v.y), nd, float(v.z), nd)
    except Exception:
        return "(?,?,?)"


def _sheet_thickness(flat_shape) -> float:
    """Толщина листа = min BBox dimension."""
    if flat_shape is None or flat_shape.isNull():
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
    return 1.0 if t > 100.0 else t


def _sheet_normal(bend) -> App.Vector:
    """Нормаль ЛИСТА (не up_hint)."""
    try:
        n = App.Vector(getattr(bend, "normal", App.Vector(0, 0, 1)))
    except Exception:
        n = App.Vector(0, 0, 1)
    if n.Length < 1e-9:
        return App.Vector(0, 0, 1)
    return n.normalize()


class SimulationController:
    """Контроллер 3D-визуализации последовательности гибки."""

    GROUP_NAME = "BendSimulation"

    def __init__(self,
                 step_results,
                 flat_shape=None,
                 pad_shape=None,
                 bend_features=None,
                 unfold_panels=None,
                 unfold_hinges=None,
                 punch_adapter=None,
                 die_adapter=None,
                 gauge_adapter=None,
                 press_adapter=None,
                 logger=None,
                 kinematics=None,
                 body=None):
        self.step_results = list(step_results or [])
        self.flat_shape = flat_shape
        self.pad_shape = pad_shape
        self.bend_features = list(bend_features or [])
        self.unfold_panels = list(unfold_panels or [])
        self.unfold_hinges = list(unfold_hinges or [])
        self.body = body

        self.punch_adapter = punch_adapter
        self.die_adapter = die_adapter
        self.gauge_adapter = gauge_adapter
        self.press_adapter = press_adapter
        self._log = logger if callable(logger) else (lambda m: None)

        if kinematics is not None:
            self._kinematics = kinematics
        elif BendKinematics is not None:
            try:
                self._kinematics = BendKinematics.from_config()
            except Exception:
                self._kinematics = None
        else:
            self._kinematics = None

        self._thickness = _sheet_thickness(flat_shape)

        self._punch_clearance = float(
            SEQUENCE_CONFIG.get("punch_clearance_offset", 2.0))
        self._die_clearance = float(
            SEQUENCE_CONFIG.get("die_clearance_offset", 0.5))
        self._gauge_clearance = float(
            SEQUENCE_CONFIG.get("backgauge_clearance", 1.0))

        self._group = None
        self._part_objs = []
        self._punch_objs = []
        self._die_objs = []
        self._gauge_objs = []
        self._collision_objs = []
        self._press_objs = []

        self.current_step = 0
        self.moment_mode = False

        self._panel_graph = None
        self._kin_engine = None
        self._ordered_features = []

        _dird("init: steps={} panels={} hinges={} thickness={:.3f}".format(
            len(self.step_results), len(self.unfold_panels),
            len(self.unfold_hinges), self._thickness))

    # ------------------------------------------------------------------
    # Публичный API
    # ------------------------------------------------------------------

    def is_built(self) -> bool:
        return self._group is not None

    def build(self) -> None:
        if self.is_built():
            return

        doc = App.ActiveDocument
        if doc is None:
            self._log("[SimCtrl] нет активного документа — build отменён")
            return

        self._log("[SimCtrl] build: {} шагов".format(
            len(self.step_results)))

        self._group = doc.addObject(
            "App::DocumentObjectGroup", self.GROUP_NAME)
        self._group.Label = "Bend Simulation (3D)"

        # --- Кинематический движок ---
        try:
            from core.panel_graph import PanelGraph
            from core.kinematic_engine import KinematicEngine
            if self.unfold_panels and self.bend_features:
                self._panel_graph = PanelGraph(
                    self.unfold_panels, self.unfold_hinges,
                    self.bend_features)
                self._kin_engine = KinematicEngine(
                    self._panel_graph, self.bend_features,
                    body=self.body)
                self._ordered_features = self._kin_engine.all_step_keys()
                self._log("[SimCtrl] Kinematic: panels={} steps={}".format(
                    len(self._panel_graph.nodes),
                    len(self._ordered_features)))
            else:
                self._log("[SimCtrl] unfold_panels пусты — "
                          "используем shape_after")
        except Exception as e:
            self._log("[SimCtrl] kinematic init error: {}".format(e))
            self._panel_graph = None
            self._kin_engine = None
            self._ordered_features = []

        # --- Станина ---
        if self.press_adapter is not None:
            try:
                shapes = self.press_adapter.get_virtual_shapes() or []
            except Exception as e:
                self._log("[SimCtrl] press shapes error: {}".format(e))
                shapes = []
            for j, sh in enumerate(shapes):
                obj = self._add_feature(
                    "SimPress_{}".format(j + 1), sh, visible=True)
                if obj is not None:
                    self._press_objs.append(obj)

        missing = []

        for i, step in enumerate(self.step_results):
            bend = (getattr(step, "working_bend_spec", None)
                    or getattr(step, "bend_spec", None))

            # --- Part shape ---
            part_shape = None
            if self._kin_engine is not None and self._ordered_features:
                try:
                    active = self._ordered_features[:i + 1]
                    part_shape = self._kin_engine.build_step_compound(
                        active, moment_factor=1.0)
                except Exception as e:
                    self._log("[SimCtrl] kinematic step {} error: {}".format(
                        i + 1, e))
                    part_shape = None

            if part_shape is None:
                part_shape = getattr(step, "shape_after", None)
            if part_shape is None:
                part_shape = self._fallback_part_shape(i)

            part_obj = self._add_feature(
                "SimPart_{}".format(i + 1), part_shape)
            self._part_objs.append(part_obj)
            if part_obj is None:
                missing.append("part[{}]".format(i + 1))

            # --- Пуансон ---
            punch_obj = None
            if self.punch_adapter is not None:
                punch_obj = self._add_feature(
                    "SimPunch_{}".format(i + 1),
                    self._make_punch_shape(bend))
                if punch_obj is None:
                    missing.append("punch[{}]".format(i + 1))
            self._punch_objs.append(punch_obj)

            # --- Матрица ---
            die_obj = None
            if self.die_adapter is not None:
                die_obj = self._add_feature(
                    "SimDie_{}".format(i + 1),
                    self._make_die_shape(bend))
                if die_obj is None:
                    missing.append("die[{}]".format(i + 1))
            self._die_objs.append(die_obj)

            # --- Задний упор ---
            gauge_obj = None
            if self.gauge_adapter is not None:
                gauge_obj = self._add_feature(
                    "SimGauge_{}".format(i + 1),
                    self._make_gauge_shape(bend))
                if gauge_obj is None:
                    missing.append("gauge[{}]".format(i + 1))
            self._gauge_objs.append(gauge_obj)

            # --- Коллизия ---
            coll_shape = None
            details = getattr(step, "details", {}) or {}
            if not getattr(step, "ok", True):
                coll_shape = details.get("collision_shape")
            self._collision_objs.append(
                self._add_collision(
                    "SimCollision_{}".format(i + 1), coll_shape))

        if self.punch_adapter is None:
            self._log("[SimCtrl] punch_adapter=None → SimPunch не созданы")
        if self.die_adapter is None:
            self._log("[SimCtrl] die_adapter=None → SimDie не созданы")
        if self.gauge_adapter is None:
            self._log("[SimCtrl] gauge_adapter=None → SimGauge не созданы")
        if missing:
            self._log("[SimCtrl] не созданы объекты: {}".format(
                ", ".join(missing)))

        try:
            doc.recompute()
        except Exception:
            pass

        self._log("[SimCtrl] build завершён")

    def show_step(self, idx: int) -> None:
        if not self.is_built():
            return
        if not (0 <= idx < len(self.step_results)):
            return

        if self._kin_engine is not None and self._ordered_features:
            try:
                active = self._ordered_features[:idx + 1]
                factor = 0.5 if self.moment_mode else 1.0
                new_shape = self._kin_engine.build_step_compound(
                    active, moment_factor=factor)
                if new_shape is not None and not new_shape.isNull():
                    obj = (self._part_objs[idx]
                           if idx < len(self._part_objs) else None)
                    if obj is not None:
                        obj.Shape = new_shape
            except Exception as e:
                self._log("[SimCtrl] kinematic show_step error: {}".format(e))
        else:
            if self.moment_mode:
                self._rebuild_part_moment(idx)
            else:
                self._rebuild_part_final(idx)

        self._hide_all()

        for lst in (self._part_objs, self._punch_objs, self._die_objs,
                    self._gauge_objs, self._collision_objs):
            obj = lst[idx] if idx < len(lst) else None
            if obj is None:
                continue
            try:
                obj.ViewObject.Visibility = True
            except Exception:
                pass

        for obj in self._press_objs:
            try:
                obj.ViewObject.Visibility = True
            except Exception:
                pass

        self.current_step = idx

    def clear(self) -> None:
        doc = App.ActiveDocument
        if doc is None or self._group is None:
            return
        try:
            grp = doc.getObject(self._group.Name)
            if grp is not None:
                for child in list(getattr(grp, "Group", [])):
                    try:
                        doc.removeObject(child.Name)
                    except Exception:
                        pass
                doc.removeObject(grp.Name)
                doc.recompute()
        except Exception as e:
            self._log("[SimCtrl] cleanup error: {}".format(e))

        self._group = None
        self._part_objs = []
        self._punch_objs = []
        self._die_objs = []
        self._gauge_objs = []
        self._collision_objs = []
        self._press_objs = []

    def fit_view(self) -> None:
        try:
            view = Gui.ActiveDocument.ActiveView
            view.viewAxonometric()
            view.fitAll()
        except Exception:
            pass

    def step_status(self, idx: int):
        if not (0 <= idx < len(self.step_results)):
            return ("?", False)
        step = self.step_results[idx]
        return (getattr(step, "bend_id", "?"),
                bool(getattr(step, "ok", True)))

    def set_moment_mode(self, enabled: bool) -> None:
        self.moment_mode = bool(enabled)
        if self.is_built():
            self.show_step(self.current_step)

    # ------------------------------------------------------------------
    # Внутреннее
    # ------------------------------------------------------------------

    def _rebuild_part_final(self, idx: int) -> None:
        shape = getattr(self.step_results[idx], "shape_after", None)
        if shape is None:
            shape = self._fallback_part_shape(idx)
        self._replace_part_shape(idx, shape)

    def _rebuild_part_moment(self, idx: int) -> None:
        if simulate_bend_occ is None:
            return

        step = self.step_results[idx]
        bend = (getattr(step, "working_bend_spec", None)
                or getattr(step, "bend_spec", None))
        if bend is None:
            return

        shape_before = self._shape_before_step(idx)
        if shape_before is None:
            _dird("moment idx={}: shape_before=None".format(idx))
            return

        try:
            moment_shape, _ = simulate_bend_occ(
                shape_before, bend,
                kinematics=self._kinematics,
                angle_factor=0.5,
            )
        except Exception as e:
            self._log("[SimCtrl] moment rebuild error: {}".format(e))
            return

        if moment_shape is None or moment_shape.isNull():
            return

        self._replace_part_shape(idx, moment_shape)

    def _shape_before_step(self, idx: int):
        if idx == 0:
            return self.flat_shape
        prev = getattr(self.step_results[idx - 1], "shape_after", None)
        if prev is not None:
            return prev
        return self.flat_shape

    def _replace_part_shape(self, idx: int, shape) -> None:
        if shape is None or shape.isNull():
            return
        obj = self._part_objs[idx] if idx < len(self._part_objs) else None
        if obj is None:
            return
        try:
            obj.Shape = shape
        except Exception as e:
            self._log("[SimCtrl] replace part error: {}".format(e))

    def _fallback_part_shape(self, i: int):
        for j in range(i - 1, -1, -1):
            sh = getattr(self.step_results[j], "shape_after", None)
            if sh is not None:
                return sh
        return self.flat_shape

    def _add_feature(self, name: str, shape, visible: bool = False):
        if shape is None or shape.isNull():
            return None
        doc = App.ActiveDocument
        if doc is None:
            return None
        obj = doc.addObject("Part::Feature", name)
        obj.Shape = shape
        try:
            obj.ViewObject.Visibility = visible
        except Exception:
            pass
        self._group.addObject(obj)
        return obj

    def _add_collision(self, name: str, shape):
        if shape is None or shape.isNull():
            return None
        doc = App.ActiveDocument
        if doc is None:
            return None
        obj = doc.addObject("Part::Feature", name)
        obj.Shape = shape
        try:
            vo = obj.ViewObject
            vo.ShapeColor = (1.0, 0.0, 0.0)
            vo.Transparency = 70
            vo.Visibility = False
        except Exception:
            pass
        self._group.addObject(obj)
        return obj

    def _make_punch_shape(self, bend):
        """Пуансон. Нормаль ЛИСТА (bend.normal) идёт вверх."""
        if bend is None or self.punch_adapter is None:
            return None
        n = _sheet_normal(bend)
        offset_vec = n * (self._thickness + self._punch_clearance)
        try:
            vs = self.punch_adapter.make_virtual_shape(
                App.Vector(bend.center), App.Vector(bend.axis),
                up_hint=n,
                length=float(getattr(bend, "length", 0.0) or 0.0),
                offset=offset_vec,
            )
            if vs is not None and not vs.isNull():
                bb = vs.BoundBox
                _dird("punch {}: bbox=({:.1f},{:.1f},{:.1f}) center=({:.1f},{:.1f},{:.1f})".format(
                    getattr(bend, "id", "?"),
                    bb.XLength, bb.YLength, bb.ZLength,
                    bb.Center.x, bb.Center.y, bb.Center.z))
            return vs
        except Exception as e:
            self._log("[SimCtrl] punch error: {}".format(e))
            return None

    def _make_die_shape(self, bend):
        """Матрица. Нормаль ЛИСТА (bend.normal) идёт вверх."""
        if bend is None or self.die_adapter is None:
            return None
        n = _sheet_normal(bend)
        offset_vec = n * (-self._die_clearance)
        try:
            vs = self.die_adapter.make_virtual_shape(
                App.Vector(bend.center), App.Vector(bend.axis),
                up_hint=n,
                length=float(getattr(bend, "length", 0.0) or 0.0),
                offset=offset_vec,
            )
            if vs is not None and not vs.isNull():
                bb = vs.BoundBox
                _dird("die {}: bbox=({:.1f},{:.1f},{:.1f}) center=({:.1f},{:.1f},{:.1f})".format(
                    getattr(bend, "id", "?"),
                    bb.XLength, bb.YLength, bb.ZLength,
                    bb.Center.x, bb.Center.y, bb.Center.z))
            return vs
        except Exception as e:
            self._log("[SimCtrl] die error: {}".format(e))
            return None

    def _make_gauge_shape(self, bend):
        if bend is None or self.gauge_adapter is None:
            return None
        n = _sheet_normal(bend)
        offset_vec = n * self._gauge_clearance
        try:
            return self.gauge_adapter.make_virtual_shape(
                App.Vector(bend.center), App.Vector(bend.axis),
                up_hint=n, offset=offset_vec,
            )
        except Exception as e:
            self._log("[SimCtrl] gauge error: {}".format(e))
            return None

    def _hide_all(self) -> None:
        for lst in (self._part_objs, self._punch_objs, self._die_objs,
                    self._gauge_objs, self._collision_objs):
            for obj in lst:
                if obj is None:
                    continue
                try:
                    obj.ViewObject.Visibility = False
                except Exception:
                    pass