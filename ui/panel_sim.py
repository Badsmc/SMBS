# -*- coding: utf-8 -*-
"""
ui/panel_sim.py — PanelSim: 3D-симуляция найденной последовательности.

Читает context.step_results. Если он пуст — показывает сообщение
«Сначала запустите Auto». Не считает сам ничего.
"""

from __future__ import annotations

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide2 import QtCore, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtWidgets

from ui.context import BendSeqContext
from ui.simulation_viewer import SimulationController
from core.bend_kinematics import BendKinematics

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


MODULE_VERSION = "BENDBEQ_PANEL_SIM_V1"
try:
    App.Console.PrintMessage(
        "[panel_sim] Загружена версия: {}\n".format(MODULE_VERSION))
except Exception:
    pass


class PanelSim(QtWidgets.QWidget):

    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setWindowTitle("BendSeq: Симуляция")
        self.ctx = BendSeqContext.get()

        self._sim_built = False
        self._sim_step = 0
        self._timer = QtCore.QTimer()
        self._timer.setInterval(800)
        self._timer.timeout.connect(self._on_tick)

        self._build_ui()
        self.ctx.add_listener(self._on_context_event)
        self._refresh_from_context()

    # ----------------------------------------------------------------

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        lbl = QtWidgets.QLabel("<b>Симуляция — 3D-просмотр</b>")
        lbl.setStyleSheet("font-size: 13pt; padding: 4px;")
        layout.addWidget(lbl)

        self.info_label = QtWidgets.QLabel(
            "Ожидание: сначала запустите Auto на панели BendSeq: Auto.")
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("color: #666; padding: 4px;")
        layout.addWidget(self.info_label)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self._on_slider)
        layout.addWidget(self.slider)

        btns = QtWidgets.QHBoxLayout()
        self.btn_prev = QtWidgets.QPushButton("<< Назад")
        self.btn_next = QtWidgets.QPushButton("Вперёд >>")
        self.btn_play = QtWidgets.QPushButton("▶ Играть")
        self.btn_pause = QtWidgets.QPushButton("⏸ Пауза")
        self.btn_fit = QtWidgets.QPushButton("Fit")
        self.btn_show = QtWidgets.QPushButton("Показать 3D")

        self.btn_prev.clicked.connect(self._on_prev)
        self.btn_next.clicked.connect(self._on_next)
        self.btn_play.clicked.connect(self._on_play)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_fit.clicked.connect(self._on_fit)
        self.btn_show.clicked.connect(self._on_show)

        for b in (self.btn_prev, self.btn_next, self.btn_play,
                  self.btn_pause, self.btn_fit, self.btn_show):
            b.setEnabled(False)
            btns.addWidget(b)
        layout.addLayout(btns)

        self.chk_moment = QtWidgets.QCheckBox("Момент гиба (пол-угла)")
        self.chk_moment.setEnabled(False)
        self.chk_moment.toggled.connect(self._on_moment)
        layout.addWidget(self.chk_moment)

        self.step_label = QtWidgets.QLabel("—")
        self.step_label.setWordWrap(True)
        layout.addWidget(self.step_label)

        layout.addStretch(1)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_close = QtWidgets.QPushButton("Закрыть")
        btn_close.setMinimumWidth(90)
        btn_close.clicked.connect(self._on_close)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

    # ----------------------------------------------------------------

    def _on_context_event(self, event):
        if event in ("results", "reset", "adapter"):
            self._refresh_from_context()

    def _refresh_from_context(self):
        n = len(self.ctx.step_results or [])
        if n == 0:
            self.info_label.setText(
                "Ожидание: сначала запустите Auto на панели BendSeq: Auto.")
            self.slider.setEnabled(False)
            self.slider.setMaximum(0)
            for b in (self.btn_prev, self.btn_next, self.btn_play,
                      self.btn_pause, self.btn_fit, self.btn_show):
                b.setEnabled(False)
            self.chk_moment.setEnabled(False)
            self._sim_built = False
            return

        self.info_label.setText("Готово: {} шагов.".format(n))
        self.slider.blockSignals(True)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, n - 1))
        self.slider.setValue(0)
        self.slider.setEnabled(True)
        self.slider.blockSignals(False)
        for b in (self.btn_prev, self.btn_next, self.btn_play,
                  self.btn_pause, self.btn_fit, self.btn_show):
            b.setEnabled(True)
        self.chk_moment.setEnabled(True)
        self._sim_built = False
        self._sim_step = 0

    # ----------------------------------------------------------------

    def _ensure_built(self, auto=False):
        if self._sim_built and self.ctx.sim_controller is not None:
            return True
        if not self.ctx.step_results:
            return False
        if auto and not self._sim_built:
            self.info_label.setText("Нажмите «Показать 3D».")
            return False

        adapters = self.ctx.tool_adapters or {}
        try:
            kinematics = BendKinematics.from_config()
        except Exception:
            kinematics = None

        pad_shape = None
        bend_features = []
        unfold_panels = []
        unfold_hinges = []
        try:
            if self.ctx.adapter and hasattr(self.ctx.adapter, "get_pad_shape"):
                pad_shape = self.ctx.adapter.get_pad_shape()
            if self.ctx.adapter and hasattr(self.ctx.adapter,
                                             "build_bend_features"):
                bend_features = self.ctx.adapter.build_bend_features()
        except Exception:
            pass

        try:
            if bool(SEQUENCE_CONFIG.get("kinematic_visualization", False)):
                from core.unfold_extractor import extract_unfold_data
                body = None
                if self.ctx.adapter and hasattr(self.ctx.adapter, "_find_body"):
                    body = self.ctx.adapter._find_body()
                if body is not None:
                    face_name = self.ctx.unfold_face_name or "Face4"
                    unfold_panels, unfold_hinges, _ = (
                        extract_unfold_data(body, face_name))
        except Exception:
            pass

        try:
            self.ctx.sim_controller = SimulationController(
                self.ctx.step_results,
                flat_shape=self.ctx.flat_shape,
                pad_shape=pad_shape,
                bend_features=bend_features,
                unfold_panels=unfold_panels,
                unfold_hinges=unfold_hinges,
                punch_adapter=adapters.get("punch"),
                die_adapter=adapters.get("die"),
                gauge_adapter=adapters.get("gauge"),
                press_adapter=adapters.get("press"),
                logger=self.ctx.log,
                kinematics=kinematics,
            )
            self.ctx.sim_controller.build()
            self._sim_built = True
            self.ctx.sim_controller.fit_view()
            return True
        except Exception as e:
            self.ctx.log("[panel_sim] build error: {}".format(e))
            return False

    def _show_step(self, idx):
        if self.ctx.sim_controller is None:
            return
        self.ctx.sim_controller.show_step(idx)
        self._sim_step = idx

        try:
            bid, ok = self.ctx.sim_controller.step_status(idx)
        except Exception:
            bid, ok = "?", True
        status = "OK" if ok else "FAIL"
        moment_tag = (" [момент]"
                      if getattr(self.ctx.sim_controller,
                                 "moment_mode", False) else "")
        self.step_label.setText(
            "Шаг {} / {} | {} | {}{}".format(
                idx + 1, len(self.ctx.step_results), bid,
                status, moment_tag))

        self.slider.blockSignals(True)
        self.slider.setValue(idx)
        self.slider.blockSignals(False)

    # ----------------------------------------------------------------

    def _on_show(self):
        if self._ensure_built(auto=False):
            self._show_step(0)

    def _on_slider(self, value):
        if not self._ensure_built(auto=True):
            self.slider.blockSignals(True)
            self.slider.setValue(0)
            self.slider.blockSignals(False)
            return
        self._show_step(value)

    def _on_prev(self):
        if not self._ensure_built(auto=True):
            return
        if self._sim_step > 0:
            self._show_step(self._sim_step - 1)

    def _on_next(self):
        if not self._ensure_built(auto=True):
            return
        if self._sim_step + 1 < len(self.ctx.step_results):
            self._show_step(self._sim_step + 1)

    def _on_play(self):
        if not self._ensure_built(auto=True):
            return
        self._timer.start()

    def _on_pause(self):
        self._timer.stop()

    def _on_fit(self):
        if self.ctx.sim_controller is not None:
            self.ctx.sim_controller.fit_view()

    def _on_tick(self):
        if self._sim_step + 1 < len(self.ctx.step_results):
            self._show_step(self._sim_step + 1)
        else:
            self._timer.stop()

    def _on_moment(self, checked):
        if self.ctx.sim_controller is None:
            return
        self.ctx.sim_controller.set_moment_mode(bool(checked))
        self._show_step(self._sim_step)

    # ----------------------------------------------------------------

    def _on_close(self):
        try:
            self._timer.stop()
        except Exception:
            pass
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