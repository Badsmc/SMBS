# -*- coding: utf-8 -*-
"""
ui/panel_debug.py — PanelDebug: лог и отчёты.

Слушает context.log() и обновляет QPlainTextEdit в реальном
времени. Также даёт кнопки очистки лога и сохранения дебаг-отчёта.
"""

from __future__ import annotations

from pathlib import Path

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide2 import QtCore, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtWidgets

from ui.context import BendSeqContext

try:
    from config import SEQUENCE_CONFIG
except ImportError:
    SEQUENCE_CONFIG = {}


MODULE_VERSION = "BENDBEQ_PANEL_DEBUG_V1"
try:
    App.Console.PrintMessage(
        "[panel_debug] Загружена версия: {}\n".format(MODULE_VERSION))
except Exception:
    pass


class PanelDebug(QtWidgets.QWidget):

    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setWindowTitle("BendSeq: Дебаг")
        self.ctx = BendSeqContext.get()

        self._build_ui()
        self._reload_log()
        self.ctx.add_listener(self._on_context_event)

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        lbl = QtWidgets.QLabel("<b>Дебаг — лог и отчёты</b>")
        lbl.setStyleSheet("font-size: 13pt; padding: 4px;")
        layout.addWidget(lbl)

        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        try:
            self.text.setMaximumBlockCount(10000)
        except Exception:
            pass
        layout.addWidget(self.text, 1)

        btns = QtWidgets.QHBoxLayout()
        self.btn_clear = QtWidgets.QPushButton("Очистить лог")
        self.btn_save = QtWidgets.QPushButton("Сохранить дебаг-отчёт")
        self.btn_clear.clicked.connect(self._on_clear)
        self.btn_save.clicked.connect(self._on_save)
        btns.addWidget(self.btn_clear)
        btns.addWidget(self.btn_save)
        btns.addStretch(1)
        layout.addLayout(btns)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_close = QtWidgets.QPushButton("Закрыть")
        btn_close.setMinimumWidth(90)
        btn_close.clicked.connect(self._on_close)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

    # ----------------------------------------------------------------

    def _on_context_event(self, event):
        if event == "log":
            self._append_last()
        elif event == "log_clear":
            self.text.clear()

    def _reload_log(self):
        self.text.clear()
        for line in self.ctx.log_lines():
            self.text.appendPlainText(line)

    def _append_last(self):
        try:
            lines = self.ctx.log_lines()
            if not lines:
                return
            self.text.appendPlainText(lines[-1])
        except Exception:
            pass

    def _on_clear(self):
        self.ctx.clear_log()

    def _on_save(self):
        try:
            import freecad_sequence_debug as dbg
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Дебаг", str(exc))
            return
        out_dir = Path(SEQUENCE_CONFIG.get("debug_default_path")
                       or str(Path.home() / "BendSeq_reports"))
        out_dir.mkdir(parents=True, exist_ok=True)
        try:
            auditor = dbg.ProjectAuditor()
            auditor.run(scene=True, document_dump=True, panel_dump=True)
            json_path, txt_path = auditor.save(out_dir)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Дебаг", str(exc))
            return
        self.ctx.log("[debug] saved: {} | {}".format(json_path, txt_path))

    # ----------------------------------------------------------------

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