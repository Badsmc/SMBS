# -*- coding: utf-8 -*-
"""
ui/panel_auto.py — PanelAuto: расчёт последовательности.

V2: Tool Set вместо combo_punch/combo_die. Инструменты берутся из
библиотеки tooling (~/BendSeq/ToolLibrary). Кнопка "Manage…" открывает
вкладку Tooling (Punches / Dies / Sets).

V1: перенос логики из task_panel.py (V8.5). Использует BendSeqContext.
"""

from __future__ import annotations

import time
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide2 import QtCore, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtWidgets

from ui.context import BendSeqContext
from core.sequence_state import ManualSequenceState
from core.sequence_validator import SequenceValidator, _clone_bend_spec
from plugins.tooling.base_tool import ToolAdapter
from plugins.tooling.press_adapter import PressGeometryAdapter
from export.report_writer import write_json_report, write_txt_report
from core.bend_kinematics import BendKinematics

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {"debug_enabled": False}


MODULE_VERSION = "BENDBEQ_PANEL_AUTO_V2_TOOLSET"
try:
    App.Console.PrintMessage(
        "[panel_auto] Загружена версия: {}\n".format(MODULE_VERSION))
except Exception:
    pass


class PanelAuto(QtWidgets.QWidget):

    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setWindowTitle("BendSeq: Auto")
        self.ctx = BendSeqContext.get()

        self._validating = False
        self._unfold_face_name = None

        self._build_ui()
        self._populate_tool_sets()
        self._populate_press_gauge()

    # ----------------------------------------------------------------

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        lbl = QtWidgets.QLabel("<b>Auto — расчёт последовательности</b>")
        lbl.setStyleSheet("font-size: 13pt; padding: 4px;")
        layout.addWidget(lbl)

        # --- инструменты ---
        grid = QtWidgets.QGridLayout()

        grid.addWidget(QtWidgets.QLabel("Tool Set:"), 0, 0)
        self.combo_toolset = QtWidgets.QComboBox()
        grid.addWidget(self.combo_toolset, 0, 1)
        self.btn_manage = QtWidgets.QPushButton("Manage…")
        self.btn_manage.setMaximumWidth(90)
        self.btn_manage.clicked.connect(self._open_tooling_tab)
        grid.addWidget(self.btn_manage, 0, 2)

        grid.addWidget(QtWidgets.QLabel("Станина:"), 2, 0)
        self.combo_press = QtWidgets.QComboBox()
        grid.addWidget(self.combo_press, 2, 1)

        grid.addWidget(QtWidgets.QLabel("Задний упор:"), 3, 0)
        self.combo_gauge = QtWidgets.QComboBox()
        grid.addWidget(self.combo_gauge, 3, 1)
        layout.addLayout(grid)

        info = QtWidgets.QLabel(
            "1. Выделите плоскую грань детали в 3D-виде.\n"
            "2. Нажмите нужную кнопку Auto.")
        info.setWordWrap(True)
        info.setStyleSheet("color: #666; padding: 4px;")
        layout.addWidget(info)

        # --- кнопки Auto ---
        btns = QtWidgets.QHBoxLayout()
        self.btn_physical = QtWidgets.QPushButton("Auto (Physical)")
        self.btn_hybrid = QtWidgets.QPushButton("Auto (Hybrid)")
        self.btn_backward = QtWidgets.QPushButton("Backward [legacy]")
        self.btn_forward = QtWidgets.QPushButton("Forward [legacy]")
        self.btn_physical.clicked.connect(
            lambda: self._run_auto("physical"))
        self.btn_hybrid.clicked.connect(
            lambda: self._run_auto("hybrid"))
        self.btn_backward.clicked.connect(
            lambda: self._run_auto("backward_astar"))
        self.btn_forward.clicked.connect(
            lambda: self._run_auto("forward_greedy"))
        for b in (self.btn_physical, self.btn_hybrid,
                  self.btn_backward, self.btn_forward):
            btns.addWidget(b)
        layout.addLayout(btns)

        # --- кнопки действий ---
        btns2 = QtWidgets.QHBoxLayout()
        self.btn_clear = QtWidgets.QPushButton("Очистить")
        self.btn_numbers = QtWidgets.QPushButton("Показать номера")
        self.btn_export = QtWidgets.QPushButton("Экспорт")
        self.btn_clear.clicked.connect(self._on_clear)
        self.btn_numbers.clicked.connect(self._on_numbers)
        self.btn_export.clicked.connect(self._on_export)
        btns2.addWidget(self.btn_clear)
        btns2.addWidget(self.btn_numbers)
        btns2.addWidget(self.btn_export)
        layout.addLayout(btns2)

        # --- таблица ---
        layout.addWidget(QtWidgets.QLabel("Найденная последовательность:"))
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setColumnCount(4)
        self.tree.setHeaderLabels(["#", "Гиб", "Угол", "Score"])
        self.tree.setRootIsDecorated(False)
        self.tree.setAlternatingRowColors(True)
        layout.addWidget(self.tree, 1)

        self.status = QtWidgets.QLabel("Готово к работе.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        # --- Закрыть ---
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_close = QtWidgets.QPushButton("Закрыть")
        btn_close.setMinimumWidth(90)
        btn_close.clicked.connect(self._on_close)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

    # ----------------------------------------------------------------
    # Tooling
    # ----------------------------------------------------------------

    def _open_tooling_tab(self):
        try:
            from ui.tooling_dialog import ToolingTab
        except Exception as e:
            QtWidgets.QMessageBox.warning(
                self, "Tooling",
                "Не удалось открыть вкладку Tooling: {}".format(e))
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Tooling — Library")
        dlg.resize(1000, 560)
        lay = QtWidgets.QVBoxLayout(dlg)
        tab = ToolingTab(dlg)
        lay.addWidget(tab, 1)
        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        btn_close = QtWidgets.QPushButton("Close")
        btn_close.clicked.connect(dlg.accept)
        row.addWidget(btn_close)
        lay.addLayout(row)
        dlg.exec_()
        # после закрытия — перечитать combo
        self._populate_tool_sets()

    def _populate_tool_sets(self):
        try:
            self.combo_toolset.clear()
        except Exception:
            return
        self.combo_toolset.addItem("— нет —", None)
        try:
            from tooling import get_library
            lib = get_library()
            for s in lib.list_sets():
                label = "{}  [{} + {}]".format(s.name, s.punch_id, s.die_id)
                self.combo_toolset.addItem(label, s.id)
            self.ctx.log("[panel_auto] tool sets: {} loaded from {}".format(
                len(lib.list_sets()), lib.root))
        except Exception as e:
            self.ctx.log(
                "[panel_auto] tooling library недоступна: {}".format(e))

    def _populate_press_gauge(self):
        roles = {
            "press": (self.combo_press,
                      ("press", "frame", "st", "станина", "ram")),
            "gauge": (self.combo_gauge,
                      ("gauge", "backgauge", "upor", "упор", "stop")),
        }
        for combo, _ in roles.values():
            combo.clear()
            combo.addItem("— Отсутствует —", None)

        doc = App.ActiveDocument
        if not doc:
            return
        for obj in doc.Objects:
            if not hasattr(obj, "Shape") or obj.Shape.isNull():
                continue
            try:
                if obj.isDerivedFrom("Sketcher::SketchObject"):
                    continue
            except Exception:
                pass
            name = (obj.Name + " " + getattr(obj, "Label", "")).lower()
            for _, (combo, keys) in roles.items():
                if any(k in name for k in keys):
                    combo.addItem(obj.Label, obj)
                    break

    def _build_adapters(self):
        press_obj = self.combo_press.currentData()
        gauge_obj = self.combo_gauge.currentData()

        punch = None
        die = None

        set_id = self.combo_toolset.currentData()
        if set_id:
            try:
                from tooling import (
                    get_library, make_punch_adapter, make_die_adapter,
                )
                lib = get_library()
                s = lib.get_set(set_id)
                if s is not None:
                    p_spec = lib.get_punch(s.punch_id)
                    d_spec = lib.get_die(s.die_id)
                    if p_spec is not None:
                        punch = make_punch_adapter(
                            p_spec, lib, logger=self.ctx.log)
                    if d_spec is not None:
                        die = make_die_adapter(
                            d_spec, lib, logger=self.ctx.log)
                    self.ctx.log(
                        "[panel_auto] tool set '{}' loaded: "
                        "punch={} die={}".format(
                            s.name, s.punch_id, s.die_id))
            except Exception as e:
                self.ctx.log(
                    "[panel_auto] tool set load failed: {}".format(e))

        if punch is None:
            self.ctx.log("[panel_auto] punch=None — PUNCH-проверки пропущены")
        if die is None:
            self.ctx.log("[panel_auto] die=None — DIE-проверки пропущены")

        gauge = ToolAdapter(gauge_obj, role="gauge") if gauge_obj else None
        press = PressGeometryAdapter([press_obj] if press_obj else [])

        adapters = {"punch": punch, "die": die,
                    "press": press, "gauge": gauge}
        self.ctx.tool_adapters = adapters
        return adapters

    # ----------------------------------------------------------------
    # Auto
    # ----------------------------------------------------------------

    def _run_auto(self, strategy):
        sel = Gui.Selection.getSelectionEx()
        face_owner = None
        face_name = None
        for s in sel:
            for sub in s.SubElementNames:
                if sub.startswith("Face"):
                    face_owner = s.Object
                    face_name = sub
                    break
            if face_owner is not None:
                break
        if face_owner is None or face_name is None:
            self.status.setText("Выделите плоскую грань детали в 3D-виде.")
            self.status.setStyleSheet("color: #c08000; font-weight: bold;")
            return

        self._unfold_face_name = face_name
        self.ctx.unfold_face_name = face_name

        self.status.setText("Строю развёртку от {}.{}…".format(
            face_owner.Name, face_name))
        self.status.setStyleSheet("color: #0066cc;")

        prog = QtWidgets.QProgressDialog(
            "Построение развёртки…", None, 0, 0, self)
        prog.setWindowTitle("Auto-Sequencer")
        try:
            prog.setWindowModality(QtCore.Qt.WindowModal)
        except Exception:
            pass
        prog.setCancelButton(None)
        prog.setMinimumDuration(0)
        prog.show()
        try:
            unfold = self._create_unfold(face_owner, face_name)
        finally:
            prog.close()

        if unfold is None:
            self.status.setText("Не удалось создать развёртку.")
            self.status.setStyleSheet("color: #cc0000; font-weight: bold;")
            return

        self.ctx.unfold_obj = unfold

        try:
            from adapters.freecad_adapter import FreeCADSheetMetalAdapter
            adapter = FreeCADSheetMetalAdapter(
                unfold, logger=self.ctx.log)
            self.ctx.set_adapter(adapter)
        except Exception as exc:
            self.ctx.log("[panel_auto] adapter error: {}".format(exc))
            return

        if not self.ctx.all_bends:
            self.status.setText("Гибы не найдены.")
            self.status.setStyleSheet("color: #cc0000; font-weight: bold;")
            return

        self._do_auto(strategy)

    def _do_auto(self, strategy):
        try:
            from autosequencer import calculate_auto_sequence_ex
        except Exception as exc:
            self.ctx.log("[panel_auto] import failed: {}".format(exc))
            return

        adapters = self._build_adapters()
        press_shapes = adapters["press"].get_virtual_shapes()

        self.status.setText("Auto ({}): поиск…".format(strategy))
        self.status.setStyleSheet("color: #0066cc;")

        prog = QtWidgets.QProgressDialog(
            "Auto ({}): поиск…".format(strategy), None, 0, 0, self)
        prog.setWindowTitle("Auto-Sequencer")
        try:
            prog.setWindowModality(QtCore.Qt.WindowModal)
        except Exception:
            pass
        prog.setAutoClose(False)
        prog.setAutoReset(False)
        prog.setCancelButton(None)
        prog.setMinimumDuration(300)
        prog.show()

        prog_state = {"last": 0.0}

        def _progress(done, total, oracle_calls):
            now = time.time()
            if now - prog_state["last"] < 0.5:
                return
            prog_state["last"] = now
            try:
                prog.setLabelText(
                    "Поиск: {}/{} (оракул={})".format(
                        done, total, oracle_calls))
            except Exception:
                pass

        try:
            self._prepare_context(adapters, press_shapes)
        except Exception as e:
            self.ctx.log("[panel_auto] context error: {}".format(e))

        bends_flat = []
        try:
            for b in (self.ctx.all_bends or []):
                try:
                    nb = _clone_bend_spec(b)
                except Exception:
                    nb = b
                bends_flat.append(nb)
            self._apply_flat_coords(bends_flat)
        except Exception as e:
            self.ctx.log("[panel_auto] bends_flat failed: {}".format(e))
            bends_flat = []

        try:
            result = calculate_auto_sequence_ex(
                unfold_obj=self.ctx.unfold_obj,
                adapter=self.ctx.adapter,
                flat_shape=self.ctx.flat_shape,
                punch_adapter=adapters["punch"],
                die_adapter=adapters["die"],
                press_shapes=press_shapes,
                gauge_adapter=adapters["gauge"],
                bends_flat=bends_flat,
                time_budget_sec=120.0,
                panel_graph_2d=self.ctx.pg2,
                panel_graph=self.ctx.pg3,
                bend_info=self.ctx.bend_info,
                thickness_mm=self.ctx.thickness,
                strategy=strategy,
                max_iterations=2000,
                max_seconds=None,
                allow_partial=True,
                logger=self.ctx.log,
                cancel_check=lambda: False,
                progress_callback=_progress,
            )
        except Exception as exc:
            import traceback
            self.ctx.log("[panel_auto] exception: {}\n{}".format(
                exc, traceback.format_exc()))
            result = {"ok": False, "order_ids": [], "notes": [str(exc)]}
        finally:
            prog.close()

        self.ctx.last_auto_result = result
        seq = list(result.get("order_ids") or [])

        self.ctx.log("[panel_auto] strategy={} ok={} source={} seq={}".format(
            strategy, result.get("ok"), result.get("source"), seq))

        self.tree.clear()
        if not result.get("ok") or not seq:
            self.status.setText(
                "Последовательность не найдена.\nЗаметки: {}".format(
                    "; ".join(result.get("notes") or [])))
            self.status.setStyleSheet("color: #cc0000; font-weight: bold;")
            return

        self.ctx.state = ManualSequenceState(
            self.ctx.flat_shape, self.ctx.all_bends)
        for bid in seq:
            self.ctx.state.append_bend(bid)

        self._populate_tree(seq, result.get("score"))

        n_total = len(self.ctx.all_bends)
        n_done = len(seq)
        partial = " (частичная)" if n_done < n_total else ""
        self.status.setText(
            "Auto{}: {}/{} гибов (source={}). Валидация…".format(
                partial, n_done, n_total, result.get("source")))
        self.status.setStyleSheet("color: #008000;")

        self._do_validate()

    def _populate_tree(self, seq_ids, score):
        id_map = {b.id: b for b in self.ctx.all_bends}
        score_total = score.get("total") if score else None
        for step, bid in enumerate(seq_ids, start=1):
            bend = id_map.get(bid)
            if bend is None:
                continue
            angle = float(getattr(bend, "angle", 0.0))
            direction = str(getattr(bend, "direction", "up")).lower()
            sign = "+" if direction == "up" else "−"
            item = QtWidgets.QTreeWidgetItem(self.tree)
            item.setText(0, str(step))
            item.setText(1, bid)
            item.setText(2, "{}{:.1f}°".format(sign, angle))
            item.setText(3, "" if score_total is None
                           else "{:.2f}".format(score_total))
        for col in (0, 1, 2, 3):
            self.tree.resizeColumnToContents(col)

    # ----------------------------------------------------------------
    # Контекст и плоские координаты
    # ----------------------------------------------------------------

    def _prepare_context(self, adapters, press_shapes):
        self.ctx.pg3 = None
        self.ctx.pg2 = None
        self.ctx.bend_info = {}
        self.ctx.thickness = None

        try:
            from core.unfold_extractor import extract_unfold_data_ex
            from core.panel_graph import PanelGraph
        except Exception as e:
            self.ctx.log("[panel_auto] import failed: {}".format(e))
            return

        body = None
        if self.ctx.adapter and hasattr(self.ctx.adapter, "_find_body"):
            body = self.ctx.adapter._find_body()
        if body is None:
            return

        try:
            from core.sequence_validator import _sheet_thickness
            thickness = _sheet_thickness(self.ctx.flat_shape)
        except Exception:
            thickness = None
        if thickness is None or thickness <= 0:
            thickness = float(SEQUENCE_CONFIG.get("sheet_thickness_mm", 1.5))
        self.ctx.thickness = thickness

        bends_sketch = None
        for o in App.ActiveDocument.Objects:
            n = getattr(o, "Name", "")
            lbl = getattr(o, "Label", "")
            if "Sketch_Bends" in n or "Sketch_Bends" in lbl:
                if "Labels" in n or "Label" in lbl:
                    continue
                bends_sketch = o
                break
        self.ctx.bends_sketch = bends_sketch

        face_name = self.ctx.unfold_face_name or "Face4"
        d = extract_unfold_data_ex(body, face_name)
        panels = d.get("panels") or []
        hinges = d.get("hinges") or []
        if not panels:
            return

        features = self.ctx.adapter.build_bend_features()
        if not features:
            return

        self.ctx.pg3 = PanelGraph(panels, hinges, features)

        bend_info = {}
        for nid, node in self.ctx.pg3.nodes.items():
            if node.feature_id is None:
                continue
            parent = self.ctx.pg3.nodes.get(node.parent_id)
            if parent is None:
                continue
            feat = next((f for f in features
                         if f.id == node.feature_id), None)
            if feat is None:
                continue
            pfi = parent.resolved_face_index()
            cfi = node.resolved_face_index()
            if pfi is None or cfi is None:
                continue
            sign = -1 if feat.invert else 1
            bend_info[(pfi, cfi)] = {
                "angle_deg": float(feat.angle_deg), "sign": sign}
        self.ctx.bend_info = bend_info

        if bends_sketch is None:
            return
        try:
            from core.panel_graph_2d import build_panel_graph_2d, SHAPELY_OK
            if not SHAPELY_OK:
                return
            self.ctx.pg2 = build_panel_graph_2d(
                self.ctx.unfold_obj, bends_sketch, self.ctx.pg3,
                thickness_mm=thickness, body_shape=None)
        except Exception as e:
            self.ctx.log("[panel_auto] pg2 failed: {}".format(e))
            self.ctx.pg2 = None

    def _build_bend_id_to_face_pair(self):
        pg3 = self.ctx.pg3
        pg2 = self.ctx.pg2
        if pg3 is None or pg2 is None:
            return {}
        try:
            from core.sequence_state import (
                build_face_pair_to_step_key_mapping)
            pair_to_step = build_face_pair_to_step_key_mapping(pg3, pg2)
        except Exception:
            return {}
        if not pair_to_step:
            return {}

        from collections import OrderedDict
        feat_to_ids = OrderedDict()
        for b in (self.ctx.all_bends or []):
            meta = getattr(b, "metadata", {}) or {}
            feat = str(meta.get("feature") or "")
            if not feat:
                continue
            feat_to_ids.setdefault(feat, []).append(b.id)
        step_to_bend_id = {}
        for feat, ids in feat_to_ids.items():
            for k, bid in enumerate(ids):
                step_to_bend_id[(feat, k)] = bid
        out = {}
        for face_pair, step_key in pair_to_step.items():
            bid = step_to_bend_id.get(step_key)
            if bid is not None:
                out[bid] = face_pair
        return out

    def _apply_flat_coords(self, bends):
        pg2 = self.ctx.pg2
        if pg2 is None or not bends:
            return
        b2fp = self._build_bend_id_to_face_pair()
        if not b2fp:
            return
        thickness = float(self.ctx.thickness or 1.5)
        try:
            z_top = float(pg2.cell_z)
        except Exception:
            z_top = thickness
        z_flat = z_top - thickness * 0.5
        world_z = App.Vector(0.0, 0.0, 1.0)
        for b in bends:
            bid = getattr(b, "id", None)
            fp = b2fp.get(bid)
            if fp is None:
                continue
            axis2d = pg2.bend_by_face_pair(fp[0], fp[1])
            if axis2d is None:
                continue
            mx = (axis2d.p1[0] + axis2d.p2[0]) * 0.5
            my = (axis2d.p1[1] + axis2d.p2[1]) * 0.5
            dx, dy = axis2d.direction
            na = App.Vector(float(dx), float(dy), 0.0)
            if na.Length > 1e-9:
                na.normalize()
            nu = na.cross(world_z)
            if nu.Length > 1e-9:
                nu.normalize()
            b.center = App.Vector(mx, my, z_flat)
            b.axis = na
            b.up_hint = nu

    # ----------------------------------------------------------------
    # Валидация
    # ----------------------------------------------------------------

    def _do_validate(self):
        if self._validating:
            return
        self._validating = True
        try:
            self._run_validation()
        except Exception as exc:
            import traceback
            self.ctx.log("[panel_auto] validate error: {}\n{}".format(
                exc, traceback.format_exc()))
            self.status.setText("Ошибка валидации: {}".format(exc))
            self.status.setStyleSheet("color: #cc0000;")
        finally:
            self._validating = False

    def _run_validation(self):
        if self.ctx.state is None:
            return
        ordered = self.ctx.state.get_ordered_bends()
        if not ordered:
            return
        if self.ctx.flat_shape is None or self.ctx.flat_shape.isNull():
            return

        flat_bends = []
        for b in ordered:
            try:
                nb = _clone_bend_spec(b)
            except Exception:
                nb = b
            flat_bends.append(nb)
        try:
            self._apply_flat_coords(flat_bends)
        except Exception:
            pass

        adapters = self.ctx.tool_adapters or self._build_adapters()
        press_shapes = adapters["press"].get_virtual_shapes()

        validator = SequenceValidator(
            punch_adapter=adapters["punch"],
            die_adapter=adapters["die"],
            press_shapes=press_shapes,
            backgauge_adapter=adapters["gauge"],
            logger=self.ctx.log,
        )
        try:
            validator._flat_ref = self.ctx.flat_shape
        except Exception:
            pass

        try:
            kinematics = BendKinematics.from_config()
        except Exception:
            kinematics = None

        step_results = validator.validate_full_sequence(
            self.ctx.flat_shape, flat_bends, kinematics=kinematics)

        for res, bend in zip(step_results, flat_bends):
            if getattr(res, "bend_spec", None) is None:
                setattr(res, "bend_spec", bend)

        self.ctx.set_step_results(step_results)

        ok = sum(1 for r in step_results if r.ok)
        total = len(step_results)
        if ok == total:
            self.status.setText(
                "ФИЗИЧЕСКИ МОЖНО СОГНУТЬ. {}/{} OK.".format(ok, total))
            self.status.setStyleSheet(
                "color: #008000; font-weight: bold; font-size: 12pt;")
        else:
            bad = [getattr(r, "bend_id", "?")
                   for r in step_results if not r.ok]
            self.status.setText(
                "НЕЛЬЗЯ СОГНУТЬ ({}/{}). Провал: {}".format(
                    ok, total, ", ".join(str(b) for b in bad)))
            self.status.setStyleSheet(
                "color: #cc0000; font-weight: bold; font-size: 12pt;")

    # ----------------------------------------------------------------
    # Развёртка
    # ----------------------------------------------------------------

    def _create_unfold(self, owner, face_name):
        try:
            doc = App.ActiveDocument
            if doc is None:
                return None
            for obj in list(doc.Objects):
                n = getattr(obj, "Name", "")
                lbl = getattr(obj, "Label", "")
                if n.startswith("AutoUnfold") or lbl.startswith("Auto Unfold"):
                    if not list(getattr(obj, "InList", []) or []):
                        try:
                            doc.removeObject(obj.Name)
                        except Exception:
                            pass
            try:
                doc.recompute()
            except Exception:
                pass

            from SheetMetalUnfoldCmd import SMUnfold
            u = doc.addObject("Part::FeaturePython", "AutoUnfold")
            u.Label = "Auto Unfold"
            SMUnfold(u, owner, [face_name])
            doc.recompute()
            if not hasattr(u, "Shape") or u.Shape.isNull():
                return None
            return u
        except Exception as e:
            self.ctx.log("[panel_auto] unfold error: {}".format(e))
            return None

    # ----------------------------------------------------------------
    # Кнопки действий
    # ----------------------------------------------------------------

    def _on_clear(self):
        self.ctx.state = None
        self.ctx.step_results = []
        self.tree.clear()
        self.status.setText("Очищено.")
        self.status.setStyleSheet("")

    def _on_numbers(self):
        self.status.setText("Overlay номеров: пока не подключён.")

    def _on_export(self):
        if self.ctx.state is None:
            return
        doc = App.ActiveDocument
        doc_name = doc.Name if doc else "SheetMetalPart"
        dlg = QtWidgets.QFileDialog(self)
        dlg.setFileMode(QtWidgets.QFileDialog.AnyFile)
        dlg.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
        dlg.setNameFilter("JSON (*.json);;TXT (*.txt)")
        if not dlg.exec_():
            return
        out_path = Path(dlg.selectedFiles()[0])
        try:
            if out_path.suffix.lower() == ".json":
                write_json_report(self.ctx.state, out_path, doc_name)
            elif out_path.suffix.lower() == ".txt":
                write_txt_report(self.ctx.state, out_path, doc_name)
        except Exception as e:
            self.ctx.log("[panel_auto] export error: {}".format(e))

    def _on_close(self):
        try:
            Gui.Control.closeDialog()
        except Exception:
            pass

    @property
    def form(self):
        return self

    def getStandardButtons(self):
        try:
            return int(QtWidgets.QDialogButtonBox.Close)
        except Exception:
            return 0

    def accept(self):
        self._on_close()
        return True

    def reject(self):
        self._on_close()
        return True