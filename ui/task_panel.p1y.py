# -*- coding: utf-8 -*-
"""
ui/task_panel.py
Панель задачи: автопоиск последовательности + 3D-симуляция.

V8.6 (BendSeq):
  * Tool Set вместо отдельных combo для пуансона/матрицы.
    Инструменты загружаются из библиотеки tooling (~/BendSeq/ToolLibrary).
  * Кнопка "Manage…" открывает вкладку Tooling (Punches/Dies/Sets).
  * Станина и задний упор по-прежнему из объектов активного документа.

V8.5 (BendSeq):
  * Интеграция physical_solver: перед автопоиском спеки клонируются
    и переводятся в систему плоского листа (_apply_flat_coords);
    результат передаётся как bends_flat в calculate_auto_sequence_ex.
  * Кнопки: Physical / Hybrid / Backward [legacy] / Forward [legacy].
  * Прогресс-метка обновляется из progress_callback physical-поиска.

V8.4 (BendSeq):
  * Overlay через PanelGraph2D.BendAxis2D с переходом в Sketch-систему.
  * Управление размером текста номеров.
  * 4 стратегии поиска: forward greedy/A*, backward A*, hybrid.
  * Slide номеров вдоль линии гиба (config: overlay_slide_mm).
  * _apply_flat_coords: перед валидацией bend.center/bend.axis
    пересчитываются в координаты плоской развёртки.
"""

import os
import time
from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide2 import QtCore, QtWidgets
except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets
    except ImportError:
        from PySide import QtCore, QtGui as QtWidgets

from core.sequence_state import ManualSequenceState
from core.sequence_validator import SequenceValidator, _clone_bend_spec
from plugins.tooling.base_tool import ToolAdapter
from plugins.tooling.press_adapter import PressGeometryAdapter
from export.report_writer import write_json_report, write_txt_report
from core.bend_kinematics import BendKinematics
from ui.simulation_viewer import SimulationController

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {
        "debug_enabled": False,
        "debug_default_path": None,
        "kinematic_visualization": False,
    }

MODULE_VERSION = "TASK_PANEL_V8_6_TOOLSET"
App.Console.PrintMessage(
    "[task_panel] Загружена версия: {}\n".format(MODULE_VERSION))


class SequenceTaskPanel(QtWidgets.QWidget):

    def __init__(self, adapter=None, overlay=None):
        super().__init__()

        self.adapter = adapter

        if overlay is None:
            try:
                from adapters.text_overlay import SequenceTextOverlay
                overlay = SequenceTextOverlay(parent_unfold=None)
                App.Console.PrintMessage(
                    "[task_panel] overlay создан внутри панели\n")
            except Exception as e:
                App.Console.PrintWarning(
                    "[task_panel] не удалось создать overlay: {}\n".format(e))
                overlay = None
        self.overlay = overlay

        if self.adapter is not None:
            try:
                self.flat_shape = self.adapter.get_flat_shape()
                self.all_bends = self.adapter.get_bend_specs()
            except Exception as e:
                App.Console.PrintWarning(
                    "[task_panel] adapter error: {}\n".format(e))
                self.flat_shape = None
                self.all_bends = []
        else:
            self.flat_shape = None
            self.all_bends = []

        self.state = ManualSequenceState(self.flat_shape, self.all_bends)

        self.debug_enabled = bool(SEQUENCE_CONFIG.get("debug_enabled", False))

        self._validating = False
        self._adapters = None
        self._debug_buffer = []
        self._auto_show_3d_pending = False
        self._unfold_face_name = None
        self._last_pipeline_result = None
        self._last_auto_result = None

        self._pg3 = None
        self._pg2 = None
        self._bend_info = {}
        self._thickness = None
        self._bends_sketch = None
        self._unfold_obj = None

        self._sim_controller = None
        self._sim_built = False
        self._sim_step = 0
        self._sim_timer = QtCore.QTimer()
        self._sim_timer.setInterval(800)
        self._sim_timer.timeout.connect(self._on_sim_tick)

        self.setWindowTitle("BendSeq (Auto)")
        root = QtWidgets.QVBoxLayout(self)
        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs)

        self._build_main_tab()
        self._build_simulation_tab()
        if self.debug_enabled:
            self._build_debug_tab()

        QtCore.QTimer.singleShot(0, lambda: Gui.Selection.clearSelection())

    # ------------------------------------------------------------------
    # Вкладки
    # ------------------------------------------------------------------

    def _build_main_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        grid = QtWidgets.QGridLayout()

        grid.addWidget(QtWidgets.QLabel("Tool Set:"), 0, 0)
        self.combo_toolset = QtWidgets.QComboBox()
        grid.addWidget(self.combo_toolset, 0, 1)
        self.btn_manage_tools = QtWidgets.QPushButton("Manage…")
        self.btn_manage_tools.clicked.connect(self._open_tooling_tab)
        grid.addWidget(self.btn_manage_tools, 0, 2)

        grid.addWidget(QtWidgets.QLabel("Станина пресса:"), 2, 0)
        self.combo_press = QtWidgets.QComboBox()
        grid.addWidget(self.combo_press, 2, 1)

        grid.addWidget(QtWidgets.QLabel("Задний упор:"), 3, 0)
        self.combo_gauge = QtWidgets.QComboBox()
        grid.addWidget(self.combo_gauge, 3, 1)

        layout.addLayout(grid)
        self._populate_tool_sets()
        self._populate_press_gauge()

        info = QtWidgets.QLabel(
            "1. Выделите плоскую грань детали в 3D-виде.\n"
            "2. Выберите Tool Set и стратегию поиска.\n"
            "3. Панель построит развёртку и запустит проверку.")
        info.setWordWrap(True)
        layout.addWidget(info)

        btns = QtWidgets.QHBoxLayout()
        self.btn_auto = QtWidgets.QPushButton("Auto (Physical)")
        self.btn_auto.clicked.connect(
            lambda: self._run_auto(strategy="physical"))
        btns.addWidget(self.btn_auto)

        self.btn_auto_astar = QtWidgets.QPushButton("Auto (Hybrid)")
        self.btn_auto_astar.clicked.connect(
            lambda: self._run_auto(strategy="hybrid"))
        btns.addWidget(self.btn_auto_astar)

        self.btn_auto_backward = QtWidgets.QPushButton(
            "Auto (Backward A*) [legacy]")
        self.btn_auto_backward.clicked.connect(
            lambda: self._run_auto(strategy="backward_astar"))
        btns.addWidget(self.btn_auto_backward)

        self.btn_auto_hybrid = QtWidgets.QPushButton(
            "Auto (Forward greedy) [legacy]")
        self.btn_auto_hybrid.clicked.connect(
            lambda: self._run_auto(strategy="forward_greedy"))
        btns.addWidget(self.btn_auto_hybrid)
        layout.addLayout(btns)

        btns2 = QtWidgets.QHBoxLayout()
        self.btn_clear_seq = QtWidgets.QPushButton("Очистить")
        self.btn_clear_seq.clicked.connect(self._clear_sequence)
        btns2.addWidget(self.btn_clear_seq)

        self.btn_show_numbers = QtWidgets.QPushButton("Показать номера")
        self.btn_show_numbers.clicked.connect(self._draw_numbers_on_flat)
        btns2.addWidget(self.btn_show_numbers)

        self.btn_export = QtWidgets.QPushButton("Экспорт отчёта")
        self.btn_export.clicked.connect(self.export_report)
        btns2.addWidget(self.btn_export)
        layout.addLayout(btns2)

        h_size = QtWidgets.QHBoxLayout()
        h_size.addWidget(QtWidgets.QLabel("Размер текста (мм):"))

        self.btn_font_minus = QtWidgets.QPushButton("A−")
        self.btn_font_minus.setFixedWidth(40)
        self.btn_font_minus.clicked.connect(
            lambda: self._on_font_size_button(-2))
        h_size.addWidget(self.btn_font_minus)

        self.spin_font_size = QtWidgets.QSpinBox()
        self.spin_font_size.setMinimum(3)
        self.spin_font_size.setMaximum(200)
        self.spin_font_size.setSingleStep(1)
        try:
            initial = int(SEQUENCE_CONFIG.get("char_height", 15.0))
        except Exception:
            initial = 15
        self.spin_font_size.setValue(initial)
        self.spin_font_size.valueChanged.connect(
            self._on_font_size_changed)
        h_size.addWidget(self.spin_font_size, 1)

        self.btn_font_plus = QtWidgets.QPushButton("A+")
        self.btn_font_plus.setFixedWidth(40)
        self.btn_font_plus.clicked.connect(
            lambda: self._on_font_size_button(+2))
        h_size.addWidget(self.btn_font_plus)

        layout.addLayout(h_size)

        layout.addWidget(QtWidgets.QLabel("Найденная последовательность:"))
        self.seq_tree = QtWidgets.QTreeWidget()
        self.seq_tree.setColumnCount(5)
        self.seq_tree.setHeaderLabels(
            ["#", "Гиб", "Feature/Sub", "Угол", "Score"])
        self.seq_tree.setRootIsDecorated(False)
        self.seq_tree.setAlternatingRowColors(True)
        self.seq_tree.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection)
        self.seq_tree.header().setStretchLastSection(True)
        layout.addWidget(self.seq_tree)

        self.auto_status_label = QtWidgets.QLabel(
            "Выделите грань и нажмите кнопку.")
        self.auto_status_label.setWordWrap(True)
        layout.addWidget(self.auto_status_label)

        self._main_tab_index = self.tabs.addTab(tab, "Auto")

    def _build_simulation_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        self.sim_info_label = QtWidgets.QLabel(
            "Симуляция появится после автопоиска и валидации.")
        self.sim_info_label.setWordWrap(True)
        layout.addWidget(self.sim_info_label)

        self.sim_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sim_slider.setMinimum(0)
        self.sim_slider.setMaximum(0)
        self.sim_slider.setEnabled(False)
        self.sim_slider.valueChanged.connect(self._on_sim_slider_changed)
        layout.addWidget(self.sim_slider)

        btns = QtWidgets.QHBoxLayout()
        self.btn_sim_prev = QtWidgets.QPushButton("<< Назад")
        self.btn_sim_next = QtWidgets.QPushButton("Вперёд >>")
        self.btn_sim_play = QtWidgets.QPushButton("▶ Играть")
        self.btn_sim_stop = QtWidgets.QPushButton("⏸ Пауза")
        self.btn_sim_fit = QtWidgets.QPushButton("Fit")
        self.btn_sim_show = QtWidgets.QPushButton("Показать 3D")

        self.btn_sim_prev.clicked.connect(self._sim_prev)
        self.btn_sim_next.clicked.connect(self._sim_next)
        self.btn_sim_play.clicked.connect(self._sim_play)
        self.btn_sim_stop.clicked.connect(self._sim_pause)
        self.btn_sim_fit.clicked.connect(self._sim_fit)
        self.btn_sim_show.clicked.connect(self._sim_show_clicked)

        for b in (self.btn_sim_prev, self.btn_sim_next,
                  self.btn_sim_play, self.btn_sim_stop,
                  self.btn_sim_fit, self.btn_sim_show):
            b.setEnabled(False)
            btns.addWidget(b)
        layout.addLayout(btns)

        self.chk_moment = QtWidgets.QCheckBox("Момент гиба (пол-угла)")
        self.chk_moment.setEnabled(False)
        self.chk_moment.toggled.connect(self._on_moment_toggled)
        layout.addWidget(self.chk_moment)

        self.sim_step_label = QtWidgets.QLabel("—")
        self.sim_step_label.setWordWrap(True)
        layout.addWidget(self.sim_step_label)

        layout.addStretch(1)
        self._sim_tab_index = self.tabs.addTab(tab, "Симуляция")

    def _build_debug_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(tab)

        layout.addWidget(QtWidgets.QLabel("Лог событий:"))
        self.debug_log = QtWidgets.QPlainTextEdit()
        self.debug_log.setReadOnly(True)
        try:
            self.debug_log.setMaximumBlockCount(5000)
        except Exception:
            pass
        layout.addWidget(self.debug_log)

        btns = QtWidgets.QHBoxLayout()
        self.btn_debug_clear = QtWidgets.QPushButton("Очистить лог")
        self.btn_debug_clear.clicked.connect(self._on_debug_clear)
        btns.addWidget(self.btn_debug_clear)

        self.btn_debug_save = QtWidgets.QPushButton("Сохранить дебаг-отчёт")
        self.btn_debug_save.clicked.connect(self._on_debug_save)
        btns.addWidget(self.btn_debug_save)
        layout.addLayout(btns)

        self._debug_tab_index = self.tabs.addTab(tab, "Дебаг")

        for line in self._debug_buffer:
            self.debug_log.appendPlainText(line)

    # ------------------------------------------------------------------
    # Логирование
    # ------------------------------------------------------------------

    def _logger(self, msg):
        text = str(msg).rstrip("\n")
        try:
            App.Console.PrintMessage(text + "\n")
        except Exception:
            pass
        self._append_debug(text)

    def _append_debug(self, line):
        self._debug_buffer.append(line)
        try:
            App.Console.PrintMessage("[task_panel] {}\n".format(line))
        except Exception:
            pass
        if not self.debug_enabled:
            return
        try:
            if hasattr(self, "debug_log") and self.debug_log is not None:
                self.debug_log.appendPlainText(line)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Жизненный цикл
    # ------------------------------------------------------------------

    @property
    def form(self):
        return self

    def accept(self):
        QtCore.QTimer.singleShot(0, self._cleanup)
        return True

    def reject(self):
        QtCore.QTimer.singleShot(0, self._cleanup)
        return False

    def _cleanup(self):
        try:
            if self._sim_timer is not None:
                self._sim_timer.stop()
        except Exception:
            pass
        try:
            if self._sim_controller is not None:
                self._sim_controller.clear()
                self._sim_controller = None
        except Exception:
            pass
        try:
            self.overlay.clear()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Tooling — библиотека инструментов
    # ------------------------------------------------------------------

    def _open_tooling_tab(self):
        """Открыть вкладку Tooling в отдельном окне."""
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
        btn_close = QtWidgets.QPushButton("Close")
        btn_close.clicked.connect(dlg.accept)
        row = QtWidgets.QHBoxLayout()
        row.addStretch(1)
        row.addWidget(btn_close)
        lay.addLayout(row)
        dlg.exec_()

        # После закрытия — обновить селектор Tool Set
        self._populate_tool_sets()

    def _populate_tool_sets(self):
        """Заполнить combo_toolset из библиотеки."""
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
            App.Console.PrintMessage(
                "[task_panel] tool sets: {} loaded from {}\n".format(
                    len(lib.list_sets()), lib.root))
        except Exception as e:
            App.Console.PrintWarning(
                "[task_panel] tooling library недоступна: {}\n".format(e))

    def _populate_press_gauge(self):
        """Заполнить combo_press и combo_gauge объектами документа."""
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
        unfold_obj = getattr(self.adapter, "unfold_obj", None)
        for obj in doc.Objects:
            if not hasattr(obj, "Shape") or obj.Shape.isNull():
                continue
            try:
                if obj.isDerivedFrom("Sketcher::SketchObject"):
                    continue
            except Exception:
                pass
            if obj is unfold_obj:
                continue
            name = (obj.Name + " " + getattr(obj, "Label", "")).lower()
            for _, (combo, keys) in roles.items():
                if any(k in name for k in keys):
                    combo.addItem(obj.Label, obj)
                    break

    def _build_adapters_from_combos(self):
        """Собрать адаптеры из Tool Set + станины + упора."""
        press_obj = self.combo_press.currentData()
        gauge_obj = self.combo_gauge.currentData()

        punch_adapter = None
        die_adapter = None

        set_id = self.combo_toolset.currentData()
        if set_id:
            try:
                from tooling import (
                    get_library, make_punch_adapter, make_die_adapter,
                )
                lib = get_library()
                s = lib.get_set(set_id)
                if s is None:
                    App.Console.PrintWarning(
                        "[task_panel] tool set '{}' не найден\n".format(
                            set_id))
                else:
                    p_spec = lib.get_punch(s.punch_id)
                    d_spec = lib.get_die(s.die_id)
                    if p_spec is not None:
                        punch_adapter = make_punch_adapter(
                            p_spec, lib, logger=self._logger)
                    if d_spec is not None:
                        die_adapter = make_die_adapter(
                            d_spec, lib, logger=self._logger)
                    App.Console.PrintMessage(
                        "[task_panel] tool set '{}' loaded: "
                        "punch={} die={}\n".format(
                            s.name, s.punch_id, s.die_id))
            except Exception as e:
                App.Console.PrintError(
                    "[task_panel] tool set load failed: {}\n".format(e))

        if punch_adapter is None:
            App.Console.PrintWarning(
                "[task_panel] punch_adapter=None — PUNCH_BEFORE/DIE "
                "проверки будут пропущены\n")
        if die_adapter is None:
            App.Console.PrintWarning(
                "[task_panel] die_adapter=None — DIE проверки будут "
                "пропущены\n")

        gauge_adapter = (ToolAdapter(gauge_obj, role="gauge")
                         if gauge_obj else None)
        press_adapter = PressGeometryAdapter(
            [press_obj] if press_obj else [])

        return {
            "punch": punch_adapter,
            "die": die_adapter,
            "press": press_adapter,
            "gauge": gauge_adapter,
        }

    # ------------------------------------------------------------------
    # Плоские координаты: Body -> flat
    # ------------------------------------------------------------------

    def _apply_flat_coords(self, bends):
        """Пересчитать bend.center/bend.axis в координаты плоской развёртки.

        Источник — PanelGraph2D.BendAxis2D: 2D-координаты линий гиба
        в системе Unfold.Shape (= системе flat_shape).
        Z = mid-thickness (cell_z - thickness/2).

        ВАЖНО — две разные «нормали», не путать:

          bend.normal    — нормаль ЛИСТА. (0,0,1) для плоского листа.
                           Используется валидатором для смещения пуансона.
                           НЕ трогаем здесь — она приходит от adapter'а.

          bend.up_hint   — нормаль ПЛОСКОСТИ РАЗРЕЗА.
                           = axis × worldZ. Перпендикулярна оси гиба
                           И плоскости листа. Используется:
                             - simulate_bend_occ для разрезания детали;
                             - SimulationController для ориентации
                               пуансона и матрицы в 3D.
                           ЕСЛИ ЗДЕСЬ ПОСТАВИТЬ (0,0,1) — деталь режется
                           горизонтально пополам, а инструмент ложится плашмя.

        Меняет bends in-place.
        """
        if not bends:
            return
        pg2 = getattr(self, "_pg2", None)
        if pg2 is None:
            App.Console.PrintMessage(
                "[task_panel] _apply_flat_coords: pg2 not ready, skip\n")
            return

        b2fp = self._build_bend_id_to_face_pair()
        if not b2fp:
            App.Console.PrintMessage(
                "[task_panel] _apply_flat_coords: "
                "no face-pair mapping, skip\n")
            return

        thickness = float(getattr(self, "_thickness", None) or 1.5)
        try:
            z_top = float(pg2.cell_z)
        except Exception:
            try:
                z_top = float(self.flat_shape.BoundBox.ZMax)
            except Exception:
                z_top = thickness
        z_flat = z_top - thickness * 0.5

        world_z = App.Vector(0.0, 0.0, 1.0)

        applied = 0
        for b in bends:
            bid = getattr(b, "id", None)
            fp = b2fp.get(bid)
            if fp is None:
                continue
            parent_fi, child_fi = fp
            axis2d = pg2.bend_by_face_pair(parent_fi, child_fi)
            if axis2d is None:
                continue
            mid_x = (axis2d.p1[0] + axis2d.p2[0]) * 0.5
            mid_y = (axis2d.p1[1] + axis2d.p2[1]) * 0.5
            dx, dy = axis2d.direction
            new_axis = App.Vector(float(dx), float(dy), 0.0)
            if new_axis.Length > 1e-9:
                new_axis.normalize()

            # up_hint = нормаль плоскости разреза = axis × worldZ.
            # Для axis=(-1,0,0) даёт (0,-1,0) — перпендикулярно листу.
            new_up = new_axis.cross(world_z)
            if new_up.Length > 1e-9:
                new_up.normalize()

            old_c = b.center
            b.center = App.Vector(mid_x, mid_y, z_flat)
            b.axis = new_axis
            b.up_hint = new_up
            # bend.normal НЕ трогаем — он остаётся нормалью листа (0,0,1)
            # от adapter'а и используется валидатором для offset пуансона.

            applied += 1
            App.Console.PrintMessage(
                "[task_panel] flat coords [{}]: "
                "center ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}), "
                "axis=({:+.3f},{:+.3f},{:+.3f}), "
                "up_hint=({:+.3f},{:+.3f},{:+.3f})\n".format(
                    bid,
                    float(old_c.x), float(old_c.y), float(old_c.z),
                    mid_x, mid_y, z_flat,
                    float(new_axis.x), float(new_axis.y),
                    float(new_axis.z),
                    float(new_up.x), float(new_up.y),
                    float(new_up.z)))

        App.Console.PrintMessage(
            "[task_panel] _apply_flat_coords: applied to {}/{}\n".format(
                applied, len(bends)))

    # ------------------------------------------------------------------
    # Auto
    # ------------------------------------------------------------------

    def _run_auto(self, strategy="physical"):
        sel = Gui.Selection.getSelectionEx()
        if not sel:
            self.auto_status_label.setText(
                "Выделите плоскую грань детали в 3D-виде.")
            self.auto_status_label.setStyleSheet(
                "color: #c08000; font-weight: bold;")
            return

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
            self.auto_status_label.setText(
                "В выделении нет грани (нужно FaceN).")
            self.auto_status_label.setStyleSheet(
                "color: #c08000; font-weight: bold;")
            return

        self._unfold_face_name = face_name

        self.auto_status_label.setText(
            "Строю развёртку от {}.{}…".format(
                face_owner.Name, face_name))
        self.auto_status_label.setStyleSheet("color: #0066cc;")

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
            unfold = self._create_unfold_from_selection(
                face_owner, face_name)
        finally:
            prog.close()

        if unfold is None:
            self.auto_status_label.setText(
                "Не удалось создать развёртку от этой грани.")
            self.auto_status_label.setStyleSheet(
                "color: #cc0000; font-weight: bold;")
            return

        self._unfold_obj = unfold

        try:
            from adapters.freecad_adapter import FreeCADSheetMetalAdapter
            self.adapter = FreeCADSheetMetalAdapter(
                unfold,
                logger=lambda m: App.Console.PrintMessage(m + "\n"))
            self.flat_shape = self.adapter.get_flat_shape()
            self.all_bends = self.adapter.get_bend_specs()
            self.state = ManualSequenceState(
                self.flat_shape, self.all_bends)
        except Exception as exc:
            App.Console.PrintError(
                "[AutoSeq] rebuild error: {}\n".format(exc))
            return

        if not self.all_bends:
            self.auto_status_label.setText(
                "Развёртка создана, но гибы не найдены.")
            self.auto_status_label.setStyleSheet(
                "color: #cc0000; font-weight: bold;")
            return

        self._do_auto_sequence(strategy=strategy)

    def _do_auto_sequence(self, strategy="physical"):
        try:
            from autosequencer import calculate_auto_sequence_ex
        except Exception as exc:
            App.Console.PrintError(
                "[AutoSeq] import failed: {}\n".format(exc))
            return

        if self._validating:
            return

        adapters = self._build_adapters_from_combos()
        self._adapters = adapters
        press_shapes = adapters["press"].get_virtual_shapes()

        self.auto_status_label.setText(
            "Auto ({}): поиск последовательности…".format(strategy))
        self.auto_status_label.setStyleSheet("color: #0066cc;")

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

        cancelled = {"flag": False}

        # --- progress callback для physical ---
        prog_state = {"last": 0.0}

        def _progress(done, total, oracle_calls):
            now = time.time()
            if now - prog_state["last"] < 0.5:
                return
            prog_state["last"] = now
            try:
                prog.setLabelText(
                    "Поиск: {}/{} шагов "
                    "(оракул вызовов: {})".format(
                        done, total, oracle_calls))
            except Exception:
                pass

        try:
            self._prepare_backward_context(adapters, press_shapes)
        except Exception as e:
            App.Console.PrintError(
                "[AutoSeq] backward-context error: {}\n".format(e))

        # --- Спеки в СИСТЕМЕ ПЛОСКОГО ЛИСТА для physical-поиска ---
        # Клонируем, чтобы не портить self.all_bends: они нужны
        # state, seq_tree и overlay. _apply_flat_coords мутирует
        # in-place.
        bends_flat = []
        try:
            for b in (self.all_bends or []):
                try:
                    nb = _clone_bend_spec(b)
                except Exception:
                    nb = b
                bends_flat.append(nb)
            self._apply_flat_coords(bends_flat)
        except Exception as e:
            import traceback
            App.Console.PrintError(
                "[task_panel] bends_flat preparation failed: {}\n{}\n"
                .format(e, traceback.format_exc()))
            bends_flat = []

        try:
            result = calculate_auto_sequence_ex(
                unfold_obj=getattr(self.adapter, "unfold_obj", None),
                adapter=self.adapter,
                flat_shape=self.flat_shape,
                punch_adapter=adapters["punch"],
                die_adapter=adapters["die"],
                press_shapes=press_shapes,
                gauge_adapter=adapters["gauge"],
                # --- physical ---
                bends_flat=bends_flat,
                time_budget_sec=120.0,
                # --- legacy backward ---
                panel_graph_2d=getattr(self, "_pg2", None),
                panel_graph=getattr(self, "_pg3", None),
                bend_info=getattr(self, "_bend_info", None),
                thickness_mm=getattr(self, "_thickness", None),
                strategy=strategy,
                max_iterations=2000,
                max_seconds=None,
                allow_partial=True,
                logger=self._logger,
                cancel_check=lambda: cancelled["flag"],
                progress_callback=_progress,
            )
        except Exception as exc:
            import traceback
            App.Console.PrintError(
                "[AutoSeq] exception: {}\n{}\n".format(
                    exc, traceback.format_exc()))
            result = {"ok": False, "order_ids": [],
                      "order_face_pairs": [], "source": None,
                      "score": None, "attempts": [],
                      "notes": ["exception: {}".format(exc)]}
        finally:
            prog.close()

        self._last_auto_result = result
        seq = list(result.get("order_ids") or [])

        App.Console.PrintMessage(
            "[AutoSeq] strategy={} ok={} source={} seq={}\n".format(
                strategy, result.get("ok"), result.get("source"), seq))

        self.seq_tree.clear()

        if not result.get("ok") or not seq:
            self.auto_status_label.setText(
                "Auto ({}): последовательность не найдена.\n"
                "Заметки: {}".format(
                    strategy, "; ".join(result.get("notes") or [])))
            self.auto_status_label.setStyleSheet(
                "color: #cc0000; font-weight: bold;")
            return

        self.state.reset()
        for bid in seq:
            self.state.append_bend(bid)

        self._populate_sequence_tree(seq, score=result.get("score"))

        n_total = len(self.all_bends)
        n_done = len(seq)
        partial_note = " (частичная)" if n_done < n_total else ""
        self.auto_status_label.setText(
            "Auto{}: {}/{} гибов ({}, source={}). Запуск проверки…".format(
                partial_note, n_done, n_total,
                strategy, result.get("source")))
        self.auto_status_label.setStyleSheet("color: #008000;")

        self._auto_show_3d_pending = True
        self._do_validate()

    # ------------------------------------------------------------------
    # Backward context
    # ------------------------------------------------------------------

    def _prepare_backward_context(self, adapters, press_shapes):
        self._pg3 = None
        self._pg2 = None
        self._bend_info = {}
        self._thickness = None

        App.Console.PrintMessage(
            "[task_panel] _prepare_backward_context: start\n")

        try:
            from core.unfold_extractor import extract_unfold_data_ex
            from core.panel_graph import PanelGraph
        except Exception as e:
            App.Console.PrintError(
                "[task_panel] import failed: {}\n".format(e))
            return

        body = None
        if hasattr(self.adapter, "_find_body"):
            body = self.adapter._find_body()
        if body is None:
            App.Console.PrintMessage(
                "[task_panel] body=None\n")
            return

        thickness = None
        try:
            from core.sequence_validator import _sheet_thickness
            thickness = _sheet_thickness(self.flat_shape)
        except Exception:
            pass
        if thickness is None or thickness <= 0:
            thickness = float(SEQUENCE_CONFIG.get(
                "sheet_thickness_mm", 1.5))
        self._thickness = thickness

        unfold_obj = getattr(self.adapter, "unfold_obj", None)
        self._unfold_obj = unfold_obj

        bends_sketch = None
        for o in App.ActiveDocument.Objects:
            n = getattr(o, "Name", "")
            lbl = getattr(o, "Label", "")
            if "Sketch_Bends" in n or "Sketch_Bends" in lbl:
                if "Labels" in n or "Label" in lbl:
                    continue
                bends_sketch = o
                App.Console.PrintMessage(
                    "[task_panel] found Sketch_Bends: {}\n".format(n))
                break
        self._bends_sketch = bends_sketch

        face_name = self._unfold_face_name or "Face4"
        d = extract_unfold_data_ex(body, face_name)
        panels = d.get("panels") or []
        hinges = d.get("hinges") or []
        App.Console.PrintMessage(
            "[task_panel] extract: panels={} hinges={} reason={}\n".format(
                len(panels), len(hinges),
                d["diagnostics"].get("reason")))
        if not panels:
            return

        features = self.adapter.build_bend_features()
        if not features:
            return

        self._pg3 = PanelGraph(panels, hinges, features)

        bend_info = {}
        for nid, node in self._pg3.nodes.items():
            if node.feature_id is None:
                continue
            parent = self._pg3.nodes.get(node.parent_id)
            if parent is None:
                continue
            feat = next((f for f in features
                          if f.id == node.feature_id), None)
            if feat is None:
                continue
            parent_fi = parent.resolved_face_index()
            child_fi = node.resolved_face_index()
            if parent_fi is None or child_fi is None:
                continue
            sign = -1 if feat.invert else 1
            bend_info[(parent_fi, child_fi)] = {
                "angle_deg": float(feat.angle_deg),
                "sign": sign,
            }
        self._bend_info = bend_info

        try:
            from core.panel_graph_2d import build_panel_graph_2d, SHAPELY_OK
        except Exception:
            return

        App.Console.PrintMessage(
            "[task_panel] SHAPELY_OK = {}\n".format(SHAPELY_OK))
        if not SHAPELY_OK:
            return
        if bends_sketch is None:
            return

        try:
            self._pg2 = build_panel_graph_2d(
                unfold_obj, bends_sketch, self._pg3,
                thickness_mm=thickness, body_shape=None)
            val = self._pg2.validate()
            App.Console.PrintMessage(
                "[task_panel] PanelGraph2D: panels={} bends={} ok={}\n"
                .format(val["panels_n"], val["bends_n"], val["ok"]))
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] PanelGraph2D failed: {}\n".format(e))
            self._pg2 = None

    # ------------------------------------------------------------------
    # Развёртка
    # ------------------------------------------------------------------

    @staticmethod
    def _unfold_name_prefixes():
        return ("AutoUnfold", "Auto_Unfold_", "Auto Unfold")

    def _is_unfold_artifact(self, obj):
        prefixes = self._unfold_name_prefixes()
        name = getattr(obj, "Name", "") or ""
        label = getattr(obj, "Label", "") or ""
        return any(name.startswith(p) or label.startswith(p)
                    for p in prefixes)

    def _cleanup_unfold_artifacts(self, doc):
        for _ in range(8):
            removed_any = False
            for obj in list(doc.Objects):
                if not self._is_unfold_artifact(obj):
                    continue
                in_list = list(getattr(obj, "InList", []) or [])
                if in_list:
                    continue
                try:
                    doc.removeObject(obj.Name)
                    removed_any = True
                except Exception:
                    pass
            if not removed_any:
                break
        try:
            doc.recompute()
        except Exception:
            pass

    def _create_unfold_from_selection(self, owner, face_name):
        import traceback
        try:
            doc = App.ActiveDocument
            if doc is None:
                return None
            self._cleanup_unfold_artifacts(doc)

            try:
                from SheetMetalUnfoldCmd import SMUnfold
            except Exception as exc:
                App.Console.PrintError(
                    "[AutoSeq] SMUnfold import failed: {}\n".format(exc))
                return None

            u = doc.addObject("Part::FeaturePython", "AutoUnfold")
            u.Label = "Auto Unfold"
            SMUnfold(u, owner, [face_name])
            doc.recompute()

            if not hasattr(u, "Shape") or u.Shape.isNull():
                return None

            App.Console.PrintMessage(
                "[AutoSeq] Unfold '{}' создан от {}.{}\n".format(
                    u.Name, owner.Name, face_name))
            return u
        except Exception as exc:
            App.Console.PrintError(
                "[AutoSeq] create unfold error: {}\n{}".format(
                    exc, traceback.format_exc()))
            return None

    # ------------------------------------------------------------------
    # Таблица
    # ------------------------------------------------------------------

    def _populate_sequence_tree(self, seq_ids, score=None):
        self.seq_tree.clear()
        if not seq_ids:
            return

        id_map = {b.id: b for b in self.all_bends}
        score_total = None
        if score:
            score_total = score.get("total")

        for step, bid in enumerate(seq_ids, start=1):
            bend = id_map.get(bid)
            if bend is None:
                continue
            meta = getattr(bend, "metadata", {}) or {}
            feature = str(meta.get("feature") or "—")
            sub = str(meta.get("sub_element") or "—")
            angle = float(getattr(bend, "angle", 0.0))
            direction = str(getattr(bend, "direction", "up")).lower()
            sign = "+" if direction == "up" else "−"

            item = QtWidgets.QTreeWidgetItem(self.seq_tree)
            item.setText(0, str(step))
            item.setText(1, bid)
            item.setText(2, "{}/{}".format(feature, sub))
            item.setText(3, "{}{:.1f}°".format(sign, angle))
            item.setText(4, "" if score_total is None
                           else "{:.2f}".format(score_total))
            item.setData(0, QtCore.Qt.UserRole, bid)

        for col in (0, 1, 2, 3, 4):
            self.seq_tree.resizeColumnToContents(col)

    def _clear_sequence(self):
        self.state.reset()
        self.seq_tree.clear()
        self.auto_status_label.setText("Очищено.")
        self.auto_status_label.setStyleSheet("")
        try:
            self.overlay.clear()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Размер шрифта
    # ------------------------------------------------------------------

    def _on_font_size_changed(self, value):
        try:
            self.overlay.set_font_size(float(value))
        except Exception:
            pass

    def _on_font_size_button(self, delta):
        new_val = self.spin_font_size.value() + delta
        new_val = max(self.spin_font_size.minimum(),
                       min(self.spin_font_size.maximum(), new_val))
        self.spin_font_size.setValue(new_val)

    # ------------------------------------------------------------------
    # Overlay: построение позиций
    # ------------------------------------------------------------------

    def _build_bend_id_to_position(self):
        pg2 = getattr(self, "_pg2", None)
        if pg2 is None:
            App.Console.PrintMessage(
                "[task_panel] pg2=None, fallback\n")
            return self._fallback_positions()

        try:
            bend_id_to_face_pair = self._build_bend_id_to_face_pair()
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] _build_bend_id_to_face_pair error: {}\n"
                .format(e))
            return self._fallback_positions()

        if not bend_id_to_face_pair:
            App.Console.PrintMessage(
                "[task_panel] bend_id_to_face_pair empty, fallback\n")
            return self._fallback_positions()

        App.Console.PrintMessage(
            "[task_panel] bend_id_to_face_pair: {} entries\n".format(
                len(bend_id_to_face_pair)))

        try:
            shape_bb = self._unfold_obj.Shape.BoundBox
            shift_x = -float(shape_bb.XMin)
            shift_y = -float(shape_bb.YMin)
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] shape_bb read error: {}\n".format(e))
            shift_x = shift_y = 0.0

        App.Console.PrintMessage(
            "[task_panel] shape_bb.min=({:.2f},{:.2f}), "
            "shift_to_raw=({:.2f},{:.2f})\n".format(
                -shift_x, -shift_y, shift_x, shift_y))

        try:
            pl_unfold = (self._unfold_obj.getGlobalPlacement()
                         if self._unfold_obj and
                         hasattr(self._unfold_obj, "getGlobalPlacement")
                         else App.Placement())
        except Exception:
            pl_unfold = App.Placement()

        offset = float(SEQUENCE_CONFIG.get("overlay_offset_mm", 25.0))
        slide = float(SEQUENCE_CONFIG.get("overlay_slide_mm", 0.0))
        extra_x = float(SEQUENCE_CONFIG.get("overlay_shift_x_mm", 0.0))
        extra_y = float(SEQUENCE_CONFIG.get("overlay_shift_y_mm", 0.0))
        z_above = 0.5

        App.Console.PrintMessage(
            "[task_panel] overlay: offset={:.1f}mm slide={:.1f}mm "
            "extra=({:+.1f},{:+.1f})mm\n".format(
                offset, slide, extra_x, extra_y))

        positions = {}
        for bend_id, fp in bend_id_to_face_pair.items():
            parent_fi, child_fi = fp
            axis = pg2.bend_by_face_pair(parent_fi, child_fi)
            if axis is None:
                continue

            mid_x = (axis.p1[0] + axis.p2[0]) * 0.5
            mid_y = (axis.p1[1] + axis.p2[1]) * 0.5
            vx = axis.p2[0] - axis.p1[0]
            vy = axis.p2[1] - axis.p1[1]
            L = (vx * vx + vy * vy) ** 0.5
            if L < 1e-9:
                continue

            nx = -vy / L
            ny = vx / L
            child_panel = pg2.panels.get(child_fi)
            if child_panel is not None:
                cx, cy = child_panel.centroid_2d
                if (cx - mid_x) * nx + (cy - mid_y) * ny < 0:
                    nx = -nx
                    ny = -ny

            shape_x = mid_x + nx * offset
            shape_y = mid_y + ny * offset

            slide_tag = "—"
            if slide != 0.0:
                if abs(vx) >= abs(vy):
                    shape_x += slide
                    slide_tag = "H→+X"
                else:
                    shape_y -= slide
                    slide_tag = "V→-Y"

            sketch_x = shape_x + shift_x + extra_x
            sketch_y = shape_y + shift_y + extra_y

            local_pos = App.Vector(sketch_x, sketch_y, z_above)
            positions[bend_id] = pl_unfold.multVec(local_pos)

            App.Console.PrintMessage(
                "[task_panel]   {}: shape=({:.2f},{:.2f}) -> "
                "sketch=({:.2f},{:.2f}) slide={} ({:+.1f}mm)\n".format(
                    bend_id, shape_x, shape_y,
                    sketch_x, sketch_y, slide_tag, slide))

        App.Console.PrintMessage(
            "[task_panel] _build_bend_id_to_position: {} positions\n"
            .format(len(positions)))
        return positions

    def _fallback_positions(self):
        positions = {}
        try:
            pl_body = None
            if self.adapter and hasattr(self.adapter, "_find_body"):
                body = self.adapter._find_body()
                if body and hasattr(body, "getGlobalPlacement"):
                    pl_body = body.getGlobalPlacement()
        except Exception:
            pl_body = None

        for bend in (self.all_bends or []):
            c = getattr(bend, "center", None)
            if c is None:
                continue
            p = App.Vector(float(c.x), float(c.y), float(c.z) + 1.0)
            if pl_body is not None:
                p = pl_body.multVec(p)
            positions[bend.id] = p
        return positions

    def _build_bend_id_to_face_pair(self):
        pg3 = getattr(self, "_pg3", None)
        pg2 = getattr(self, "_pg2", None)
        if pg3 is None or pg2 is None:
            return {}

        try:
            from core.sequence_state import build_face_pair_to_step_key_mapping
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] import mapping failed: {}\n".format(e))
            return {}

        try:
            pair_to_step = build_face_pair_to_step_key_mapping(pg3, pg2)
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] build_face_pair_to_step_key failed: {}\n"
                .format(e))
            return {}
        if not pair_to_step:
            return {}

        from collections import OrderedDict
        feat_to_bend_ids = OrderedDict()
        for b in (self.all_bends or []):
            meta = getattr(b, "metadata", {}) or {}
            feat = str(meta.get("feature") or "")
            if not feat:
                continue
            feat_to_bend_ids.setdefault(feat, []).append(b.id)

        App.Console.PrintMessage(
            "[task_panel] feat_to_bend_ids: {}\n".format(
                dict(feat_to_bend_ids)))

        step_to_bend_id = {}
        for feat, ids in feat_to_bend_ids.items():
            for k, bid in enumerate(ids):
                step_to_bend_id[(feat, k)] = bid

        App.Console.PrintMessage(
            "[task_panel] step_to_bend_id: {}\n".format(step_to_bend_id))

        out = {}
        for face_pair, step_key in pair_to_step.items():
            bid = step_to_bend_id.get(step_key)
            if bid is not None:
                out[bid] = face_pair

        App.Console.PrintMessage(
            "[task_panel] bend_id_to_face_pair: {} entries\n".format(
                len(out)))
        for bid, pair in list(out.items())[:12]:
            App.Console.PrintMessage(
                "[task_panel]   {} -> faces {}\n".format(bid, pair))
        return out

    def _draw_numbers_on_flat(self):
        App.Console.PrintMessage(
            "[task_panel] _draw_numbers_on_flat: ordered_ids={}\n".format(
                len(self.state.ordered_ids)))

        if self.overlay is None:
            App.Console.PrintWarning(
                "[task_panel] overlay недоступен — номера не рисуются\n")
            return

        if not self.state.ordered_ids:
            try:
                self.overlay.clear()
            except Exception:
                pass
            return

        class _Preview:
            __slots__ = ("bend_spec", "ok", "step_number")

            def __init__(self, bend, step, ok=True):
                self.bend_spec = bend
                self.ok = ok
                self.step_number = step

        id_map = {b.id: b for b in self.all_bends}
        previews = []
        for i, bid in enumerate(self.state.ordered_ids):
            bend = id_map.get(bid)
            if bend is None:
                continue
            ok = True
            if i < len(self.state.step_results):
                ok = bool(getattr(self.state.step_results[i], "ok", True))
            previews.append(_Preview(bend, i + 1, ok))

        try:
            positions = self._build_bend_id_to_position()
            self.overlay.set_bend_positions(positions)
        except Exception as e:
            App.Console.PrintMessage(
                "[task_panel] _build_bend_id_to_position failed: {}\n"
                .format(e))

        try:
            n = self.overlay.draw_numbers(previews)
            App.Console.PrintMessage(
                "[task_panel] overlay.draw_numbers -> placed={}\n".format(n))
        except Exception as e:
            import traceback
            App.Console.PrintError(
                "[task_panel] overlay.draw_numbers error: {}\n".format(e))
            App.Console.PrintError(traceback.format_exc() + "\n")

    # ------------------------------------------------------------------
    # Валидация
    # ------------------------------------------------------------------

    def _do_validate(self):
        if self._validating:
            return
        self._validating = True
        try:
            self.run_validation()
        except Exception as exc:
            import traceback
            App.Console.PrintError(
                "[validate] error: {}\n{}\n".format(
                    exc, traceback.format_exc()))
            self.auto_status_label.setText(
                "Ошибка валидации: {}".format(exc))
            self.auto_status_label.setStyleSheet(
                "color: #cc0000; font-weight: bold;")
        finally:
            self._validating = False

    def _maybe_override_with_kinematic(self, step_results):
        if not step_results or self._pg3 is None:
            return
        try:
            from core.kinematic_engine import KinematicEngine
        except Exception:
            return
        body = None
        try:
            if hasattr(self.adapter, "_find_body"):
                body = self.adapter._find_body()
        except Exception:
            body = None
        if body is None:
            return
        try:
            feats = self._pg3.features
            ke = KinematicEngine(self._pg3, feats, body=body)
            keys = ke.all_step_keys()
            final_kin = ke.build_step_compound(keys, moment_factor=1.0)
        except Exception:
            return
        if final_kin is None or final_kin.isNull():
            return
        step_results[-1].shape_after = final_kin

    def run_validation(self):
        ordered_bends = self.state.get_ordered_bends()
        if not ordered_bends:
            try:
                self.overlay.clear()
            except Exception:
                pass
            return

        if self.flat_shape is None or self.flat_shape.isNull():
            self.auto_status_label.setText(
                "Плоская развёртка недоступна.")
            return

        # --- Плоские координаты: пересчитываем копии ---
        flat_bends = []
        for b in ordered_bends:
            try:
                nb = _clone_bend_spec(b)
            except Exception:
                nb = b
            flat_bends.append(nb)
        try:
            self._apply_flat_coords(flat_bends)
        except Exception as e:
            App.Console.PrintError(
                "[task_panel] _apply_flat_coords failed: {}\n".format(e))
        ordered_bends = flat_bends

        adapters = self._adapters or self._build_adapters_from_combos()
        self._adapters = adapters
        press_shapes = adapters["press"].get_virtual_shapes()

        validator = SequenceValidator(
            punch_adapter=adapters["punch"],
            die_adapter=adapters["die"],
            press_shapes=press_shapes,
            backgauge_adapter=adapters["gauge"],
            logger=self._logger,
        )

        kinematics = BendKinematics.from_config()

        if self._sim_controller is not None:
            try:
                self._sim_controller.clear()
            except Exception:
                pass
            self._sim_controller = None
        self._sim_built = False

        block = [
            self.btn_auto, self.btn_auto_astar,
            self.btn_auto_backward, self.btn_auto_hybrid,
            self.btn_clear_seq, self.btn_show_numbers, self.btn_export,
            self.combo_toolset, self.btn_manage_tools,
            self.combo_press, self.combo_gauge,
        ]
        for w in block:
            try:
                w.setEnabled(False)
            except Exception:
                pass

        prog = QtWidgets.QProgressDialog(
            "Выполняется симуляция…", None, 0, 0, self)
        prog.setWindowTitle("Валидация последовательности")
        try:
            prog.setWindowModality(QtCore.Qt.WindowModal)
        except Exception:
            pass
        prog.setAutoClose(False)
        prog.setAutoReset(False)
        prog.setCancelButton(None)
        prog.setMinimumDuration(300)
        prog.show()

        try:
            body_shape = None
            try:
                if hasattr(self.adapter, "get_reference_body_shape"):
                    body_shape = self.adapter.get_reference_body_shape()
            except Exception:
                body_shape = None

            step_results = None
            if body_shape is not None and not body_shape.isNull():
                step_results = validator.validate_full_sequence_reverse(
                    body_shape, ordered_bends,
                    kinematics=kinematics,
                    flat_ref=self.flat_shape,
                )

            if step_results is None:
                step_results = validator.validate_full_sequence(
                    self.flat_shape, ordered_bends,
                    kinematics=kinematics,
                )

            try:
                self._maybe_override_with_kinematic(step_results)
            except Exception:
                pass
        finally:
            prog.close()
            for w in block:
                try:
                    w.setEnabled(True)
                except Exception:
                    pass

        for res, bend in zip(step_results, ordered_bends):
            if getattr(res, "bend_spec", None) is None:
                setattr(res, "bend_spec", bend)

        self.state.step_results = step_results
        self._draw_numbers_on_flat()

        ok_count = sum(1 for r in step_results if r.ok)
        total = len(step_results)
        is_complete = (len(self.state.ordered_ids) == len(self.all_bends))

        if not is_complete:
            self.auto_status_label.setText(
                "НЕПОЛНАЯ ПОСЛЕДОВАТЕЛЬНОСТЬ ({}/{}).".format(
                    len(self.state.ordered_ids), len(self.all_bends)))
            self.auto_status_label.setStyleSheet(
                "color: #c08000; font-weight: bold; font-size: 12pt;")
        elif ok_count != total:
            bad = [getattr(r, "bend_id", "?")
                    for r in step_results if not r.ok]
            self.auto_status_label.setText(
                "НЕЛЬЗЯ СОГНУТЬ. Коллизии: {}".format(
                    ", ".join(str(b) for b in bad)))
            self.auto_status_label.setStyleSheet(
                "color: #cc0000; font-weight: bold; font-size: 12pt;")
        else:
            self.auto_status_label.setText(
                "ФИЗИЧЕСКИ МОЖНО СОГНУТЬ. Все шаги без коллизий.")
            self.auto_status_label.setStyleSheet(
                "color: #008000; font-weight: bold; font-size: 12pt;")

        self._sim_after_validation()

        try:
            from core.verifier import verify_against_reference
            ref_shape = None
            if hasattr(self.adapter, "get_reference_body_shape"):
                ref_shape = self.adapter.get_reference_body_shape()
            if ref_shape is not None and self.state.step_results:
                final = self.state.step_results[-1].shape_after
                verify_against_reference(
                    final, ref_shape, logger=self._logger)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Симуляция
    # ------------------------------------------------------------------

    def _sim_after_validation(self):
        n = len(self.state.step_results)

        self.sim_slider.blockSignals(True)
        self.sim_slider.setMinimum(0)
        self.sim_slider.setMaximum(max(0, n - 1))
        self.sim_slider.setValue(0)
        self.sim_slider.setEnabled(n > 0)
        self.sim_slider.blockSignals(False)

        for b in (self.btn_sim_prev, self.btn_sim_next,
                  self.btn_sim_play, self.btn_sim_stop,
                  self.btn_sim_fit, self.btn_sim_show):
            try:
                b.setEnabled(n > 0)
            except Exception:
                pass
        try:
            self.chk_moment.setEnabled(n > 0)
        except Exception:
            pass

        self._sim_step = 0
        self._sim_built = False

        if n > 0:
            self.sim_info_label.setText(
                "Готово: {} шагов.".format(n))
            self.sim_step_label.setText("—")
            try:
                self.tabs.setCurrentIndex(self._sim_tab_index)
            except Exception:
                pass
        else:
            self.sim_info_label.setText("Нет данных для симуляции.")
            self.sim_step_label.setText("—")

        self._auto_show_3d_pending = False

    def _sim_show_clicked(self):
        if self._sim_ensure_built(auto=False):
            self._sim_show_step(0)

    def _want_kinematic_viz(self):
        try:
            return bool(SEQUENCE_CONFIG.get(
                "kinematic_visualization", False))
        except Exception:
            return False

    def _sim_ensure_built(self, auto=False):
        if self._sim_built and self._sim_controller is not None:
            return True
        if not getattr(self.state, "step_results", None):
            return False
        if auto and not self._sim_built:
            self.sim_info_label.setText(
                "Нажмите «Показать 3D».")
            return False

        adapters = self._adapters or {}

        try:
            kinematics = BendKinematics.from_config()
        except Exception:
            kinematics = None

        pad_shape = None
        bend_features = []
        unfold_panels = []
        unfold_hinges = []

        try:
            if hasattr(self.adapter, "get_pad_shape"):
                pad_shape = self.adapter.get_pad_shape()
            if hasattr(self.adapter, "build_bend_features"):
                bend_features = self.adapter.build_bend_features()
        except Exception:
            pass

        if self._want_kinematic_viz():
            try:
                from core.unfold_extractor import extract_unfold_data
                body = None
                if hasattr(self.adapter, "_find_body"):
                    body = self.adapter._find_body()
                if body is not None:
                    face_name = self._unfold_face_name or "Face4"
                    unfold_panels, unfold_hinges, _ = (
                        extract_unfold_data(body, face_name))
            except Exception:
                pass

        self._sim_controller = SimulationController(
            self.state.step_results,
            flat_shape=self.flat_shape,
            pad_shape=pad_shape,
            bend_features=bend_features,
            unfold_panels=unfold_panels,
            unfold_hinges=unfold_hinges,
            punch_adapter=adapters.get("punch"),
            die_adapter=adapters.get("die"),
            gauge_adapter=adapters.get("gauge"),
            press_adapter=adapters.get("press"),
            logger=self._append_debug,
            kinematics=kinematics,
        )
        self._sim_controller.build()
        self._sim_built = True
        self._sim_controller.fit_view()
        return True

    def _sim_show_step(self, idx):
        if self._sim_controller is None:
            return
        self._sim_controller.show_step(idx)
        self._sim_step = idx

        bid, ok = self._sim_controller.step_status(idx)
        status = "OK" if ok else "FAIL"
        moment_tag = (" [момент]"
                      if self._sim_controller.moment_mode else "")
        self.sim_step_label.setText(
            "Шаг {} / {} | {} | {}{}".format(
                idx + 1, len(self.state.step_results), bid,
                status, moment_tag))

        self.sim_slider.blockSignals(True)
        self.sim_slider.setValue(idx)
        self.sim_slider.blockSignals(False)

    def _on_sim_slider_changed(self, value):
        if not self._sim_ensure_built(auto=True):
            self.sim_slider.blockSignals(True)
            self.sim_slider.setValue(0)
            self.sim_slider.blockSignals(False)
            return
        self._sim_show_step(value)

    def _sim_prev(self):
        if not self._sim_ensure_built(auto=True):
            return
        if self._sim_step > 0:
            self._sim_show_step(self._sim_step - 1)

    def _sim_next(self):
        if not self._sim_ensure_built(auto=True):
            return
        if self._sim_step + 1 < len(self.state.step_results):
            self._sim_show_step(self._sim_step + 1)

    def _sim_play(self):
        if not self._sim_ensure_built(auto=True):
            return
        self._sim_timer.start()

    def _sim_pause(self):
        self._sim_timer.stop()

    def _sim_fit(self):
        if self._sim_controller is not None:
            self._sim_controller.fit_view()

    def _on_sim_tick(self):
        if self._sim_step + 1 < len(self.state.step_results):
            self._sim_show_step(self._sim_step + 1)
        else:
            self._sim_timer.stop()

    def _on_moment_toggled(self, checked):
        if self._sim_controller is None:
            return
        self._sim_controller.set_moment_mode(bool(checked))
        self._sim_show_step(self._sim_step)

    # ------------------------------------------------------------------
    # Дебаг
    # ------------------------------------------------------------------

    def _on_debug_clear(self):
        self._debug_buffer = []
        if hasattr(self, "debug_log") and self.debug_log is not None:
            self.debug_log.clear()

    def _on_debug_save(self):
        try:
            import freecad_sequence_debug as dbg
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self, "Дебаг", str(exc))
            return
        out_dir = Path(SEQUENCE_CONFIG.get("debug_default_path")
                        or str(Path.home() / "BendSeq_reports"))
        out_dir.mkdir(parents=True, exist_ok=True)
        try:
            auditor = dbg.ProjectAuditor()
            report = auditor.run(scene=True,
                                  document_dump=True,
                                  panel_dump=True)
            json_path, txt_path = auditor.save(out_dir)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(
                self, "Дебаг", str(exc))
            return
        App.Console.PrintMessage(
            "[debug] saved: {} | {}\n".format(json_path, txt_path))

    # ------------------------------------------------------------------
    # Экспорт
    # ------------------------------------------------------------------

    def export_report(self):
        doc = App.ActiveDocument
        doc_name = doc.Name if doc else "SheetMetalPart"

        dlg = QtWidgets.QFileDialog(self)
        dlg.setFileMode(QtWidgets.QFileDialog.AnyFile)
        dlg.setAcceptMode(QtWidgets.QFileDialog.AcceptSave)
        dlg.setNameFilter("JSON (*.json);;TXT (*.txt)")
        if not dlg.exec_():
            return
        out_path = Path(dlg.selectedFiles()[0])

        if out_path.suffix.lower() == ".json":
            write_json_report(self.state, out_path, doc_name)
        elif out_path.suffix.lower() == ".txt":
            write_txt_report(self.state, out_path, doc_name)
        else:
            QtWidgets.QMessageBox.warning(
                self, "Экспорт", "Используйте .json или .txt.")