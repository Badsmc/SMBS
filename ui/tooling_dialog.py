# -*- coding: utf-8 -*-
"""Вкладка Tooling: Punches / Dies / Sets."""

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide2 import QtCore, QtWidgets
except ImportError:
    try:
        from PySide6 import QtCore, QtWidgets
    except ImportError:
        from PySide import QtCore, QtGui as QtWidgets

from tooling import (
    get_library, set_library, default_library_path,
    PunchSpec, DieSpec, ToolSetSpec,
)


def _make_line(label, val, parent=None):
    w = QtWidgets.QWidget()
    h = QtWidgets.QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.addWidget(QtWidgets.QLabel(label))
    e = QtWidgets.QLineEdit(str(val))
    h.addWidget(e, 1)
    return w, e


class PunchEditor(QtWidgets.QDialog):
    def __init__(self, spec=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Punch")
        self.spec = spec
        form = QtWidgets.QFormLayout(self)

        self.e_id = QtWidgets.QLineEdit(spec.id if spec else "")
        self.e_name = QtWidgets.QLineEdit(spec.name if spec else "")
        self.e_angle = QtWidgets.QDoubleSpinBox()
        self.e_angle.setRange(1, 179); self.e_angle.setDecimals(2)
        self.e_angle.setValue(spec.angle if spec else 88.0)
        self.e_h = QtWidgets.QDoubleSpinBox()
        self.e_h.setRange(1, 2000); self.e_h.setValue(spec.height if spec else 100)
        self.e_r = QtWidgets.QDoubleSpinBox()
        self.e_r.setRange(0, 50); self.e_r.setDecimals(2)
        self.e_r.setValue(spec.nose_radius if spec else 0.8)
        self.e_tw = QtWidgets.QDoubleSpinBox()
        self.e_tw.setRange(0.1, 500); self.e_tw.setValue(spec.tip_width if spec else 10)
        self.e_bw = QtWidgets.QDoubleSpinBox()
        self.e_bw.setRange(1, 500); self.e_bw.setValue(spec.body_width if spec else 60)
        self.e_th = QtWidgets.QDoubleSpinBox()
        self.e_th.setRange(0.1, 500); self.e_th.setValue(spec.tip_height if spec else 6)
        self.cb_prof = QtWidgets.QComboBox()
        self.cb_prof.addItems(["brick", "gooseneck"])
        if spec and spec.profile == "gooseneck":
            self.cb_prof.setCurrentIndex(1)

        form.addRow("ID:", self.e_id)
        form.addRow("Name:", self.e_name)
        form.addRow("Angle:", self.e_angle)
        form.addRow("Height:", self.e_h)
        form.addRow("Nose R:", self.e_r)
        form.addRow("Tip width:", self.e_tw)
        form.addRow("Body width:", self.e_bw)
        form.addRow("Tip height:", self.e_th)
        form.addRow("Profile:", self.cb_prof)

        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def result_spec(self):
        return PunchSpec(
            id=self.e_id.text().strip() or self.e_name.text().strip(),
            name=self.e_name.text().strip() or self.e_id.text().strip(),
            geometry_mode="parametric",
            angle=self.e_angle.value(),
            height=self.e_h.value(),
            nose_radius=self.e_r.value(),
            tip_width=self.e_tw.value(),
            body_width=self.e_bw.value(),
            tip_height=self.e_th.value(),
            profile=self.cb_prof.currentText(),
        )


class DieEditor(QtWidgets.QDialog):
    def __init__(self, spec=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Die")
        form = QtWidgets.QFormLayout(self)

        self.e_id = QtWidgets.QLineEdit(spec.id if spec else "")
        self.e_name = QtWidgets.QLineEdit(spec.name if spec else "")
        self.e_op = QtWidgets.QDoubleSpinBox()
        self.e_op.setRange(0.5, 500); self.e_op.setDecimals(2)
        self.e_op.setValue(spec.opening if spec else 12.0)
        self.e_angle = QtWidgets.QDoubleSpinBox()
        self.e_angle.setRange(1, 179); self.e_angle.setDecimals(2)
        self.e_angle.setValue(spec.angle if spec else 88.0)
        self.e_r = QtWidgets.QDoubleSpinBox()
        self.e_r.setRange(0, 50); self.e_r.setDecimals(2)
        self.e_r.setValue(spec.radius if spec else 0.5)
        self.e_h = QtWidgets.QDoubleSpinBox()
        self.e_h.setRange(1, 1000); self.e_h.setValue(spec.height if spec else 80)
        self.e_w = QtWidgets.QDoubleSpinBox()
        self.e_w.setRange(1, 1000); self.e_w.setValue(spec.width if spec else 70)

        form.addRow("ID:", self.e_id)
        form.addRow("Name:", self.e_name)
        form.addRow("Opening (V):", self.e_op)
        form.addRow("Angle:", self.e_angle)
        form.addRow("Radius:", self.e_r)
        form.addRow("Height:", self.e_h)
        form.addRow("Width:", self.e_w)

        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def result_spec(self):
        return DieSpec(
            id=self.e_id.text().strip() or self.e_name.text().strip(),
            name=self.e_name.text().strip() or self.e_id.text().strip(),
            geometry_mode="parametric",
            opening=self.e_op.value(),
            angle=self.e_angle.value(),
            radius=self.e_r.value(),
            height=self.e_h.value(),
            width=self.e_w.value(),
        )


class ToolingTab(QtWidgets.QWidget):
    """Вкладка Tooling."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lib = get_library()

        root = QtWidgets.QVBoxLayout(self)

        # Панель пути
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Library:"))
        self.lbl_path = QtWidgets.QLabel(str(self.lib.root))
        top.addWidget(self.lbl_path, 1)
        btn_reload = QtWidgets.QPushButton("Reload")
        btn_reload.clicked.connect(self._reload)
        top.addWidget(btn_reload)
        root.addLayout(top)

        # Основной блок — три списка в ряд
        row = QtWidgets.QHBoxLayout()

        # --- Punches ---
        col_p = QtWidgets.QVBoxLayout()
        col_p.addWidget(QtWidgets.QLabel("Punches"))
        self.list_p = QtWidgets.QListWidget()
        col_p.addWidget(self.list_p, 1)
        hb = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("New");      b.clicked.connect(self._p_new);     hb.addWidget(b)
        b = QtWidgets.QPushButton("Edit");     b.clicked.connect(self._p_edit);    hb.addWidget(b)
        b = QtWidgets.QPushButton("Import");   b.clicked.connect(self._p_import);  hb.addWidget(b)
        b = QtWidgets.QPushButton("Delete");   b.clicked.connect(self._p_del);     hb.addWidget(b)
        col_p.addLayout(hb)
        row.addLayout(col_p)

        # --- Dies ---
        col_d = QtWidgets.QVBoxLayout()
        col_d.addWidget(QtWidgets.QLabel("Dies"))
        self.list_d = QtWidgets.QListWidget()
        col_d.addWidget(self.list_d, 1)
        hb = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("New");      b.clicked.connect(self._d_new);     hb.addWidget(b)
        b = QtWidgets.QPushButton("Edit");     b.clicked.connect(self._d_edit);    hb.addWidget(b)
        b = QtWidgets.QPushButton("Import");   b.clicked.connect(self._d_import);  hb.addWidget(b)
        b = QtWidgets.QPushButton("Delete");   b.clicked.connect(self._d_del);     hb.addWidget(b)
        col_d.addLayout(hb)
        row.addLayout(col_d)

        # --- Sets ---
        col_s = QtWidgets.QVBoxLayout()
        col_s.addWidget(QtWidgets.QLabel("Tool Sets (presets)"))
        self.list_s = QtWidgets.QListWidget()
        col_s.addWidget(self.list_s, 1)
        hb = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("New");      b.clicked.connect(self._s_new);     hb.addWidget(b)
        b = QtWidgets.QPushButton("Edit");     b.clicked.connect(self._s_edit);    hb.addWidget(b)
        b = QtWidgets.QPushButton("Delete");   b.clicked.connect(self._s_del);     hb.addWidget(b)
        col_s.addLayout(hb)
        row.addLayout(col_s)

        root.addLayout(row, 1)
        self._refresh()

    # ------------------------------------------------------------------

    def _reload(self):
        set_library(None)  # force re-init
        from tooling.registry import get_library as _get
        self.lib = _get()
        self._refresh()

    def _refresh(self):
        self.list_p.clear()
        for s in self.lib.list_punches():
            self.list_p.addItem("{}  —  {} ({}°)".format(s.id, s.name, s.angle))
        self.list_d.clear()
        for s in self.lib.list_dies():
            self.list_d.addItem("{}  —  {} (V={}mm)".format(s.id, s.name, s.opening))
        self.list_s.clear()
        for s in self.lib.list_sets():
            self.list_s.addItem("{}  —  {} [{} + {}]".format(
                s.id, s.name, s.punch_id, s.die_id))

    def _selected_id(self, lst):
        it = lst.currentItem()
        if it is None:
            return None
        return it.text().split("  —  ")[0].strip()

    # --- Punches ---

    def _p_new(self):
        dlg = PunchEditor(None, self)
        if dlg.exec_():
            self.lib.add_punch(dlg.result_spec())
            self._refresh()

    def _p_edit(self):
        pid = self._selected_id(self.list_p)
        if not pid:
            return
        spec = self.lib.get_punch(pid)
        if spec is None:
            return
        dlg = PunchEditor(spec, self)
        if dlg.exec_():
            self.lib.add_punch(dlg.result_spec())
            self._refresh()

    def _p_import(self):
        sel = Gui.Selection.getSelection()
        if not sel:
            QtWidgets.QMessageBox.warning(
                self, "Import", "Выделите объект в дереве FreeCAD.")
            return
        obj = sel[0]
        name, ok = QtWidgets.QInputDialog.getText(
            self, "Import punch", "Имя:", text=obj.Label)
        if not ok or not name.strip():
            return
        new_id = name.strip().replace(" ", "_")
        self.lib.import_from_freecad(obj, "punch", new_id, name.strip())
        self._refresh()

    def _p_del(self):
        pid = self._selected_id(self.list_p)
        if not pid:
            return
        self.lib.remove_punch(pid)
        self._refresh()

    # --- Dies ---

    def _d_new(self):
        dlg = DieEditor(None, self)
        if dlg.exec_():
            self.lib.add_die(dlg.result_spec())
            self._refresh()

    def _d_edit(self):
        did = self._selected_id(self.list_d)
        if not did:
            return
        spec = self.lib.get_die(did)
        if spec is None:
            return
        dlg = DieEditor(spec, self)
        if dlg.exec_():
            self.lib.add_die(dlg.result_spec())
            self._refresh()

    def _d_import(self):
        sel = Gui.Selection.getSelection()
        if not sel:
            QtWidgets.QMessageBox.warning(
                self, "Import", "Выделите объект в дереве FreeCAD.")
            return
        obj = sel[0]
        name, ok = QtWidgets.QInputDialog.getText(
            self, "Import die", "Имя:", text=obj.Label)
        if not ok or not name.strip():
            return
        new_id = name.strip().replace(" ", "_")
        self.lib.import_from_freecad(obj, "die", new_id, name.strip())
        self._refresh()

    def _d_del(self):
        did = self._selected_id(self.list_d)
        if not did:
            return
        self.lib.remove_die(did)
        self._refresh()

    # --- Sets ---

    def _s_new(self):
        punches = self.lib.list_punches()
        dies = self.lib.list_dies()
        if not punches or not dies:
            QtWidgets.QMessageBox.warning(
                self, "Tool Set",
                "Сначала добавьте минимум один пуансон и одну матрицу.")
            return
        dlg = _SetEditor(punches, dies, None, self)
        if dlg.exec_():
            self.lib.add_set(dlg.result_spec())
            self._refresh()

    def _s_edit(self):
        sid = self._selected_id(self.list_s)
        if not sid:
            return
        spec = self.lib.get_set(sid)
        if spec is None:
            return
        punches = self.lib.list_punches()
        dies = self.lib.list_dies()
        dlg = _SetEditor(punches, dies, spec, self)
        if dlg.exec_():
            self.lib.add_set(dlg.result_spec())
            self._refresh()

    def _s_del(self):
        sid = self._selected_id(self.list_s)
        if not sid:
            return
        self.lib.remove_set(sid)
        self._refresh()


class _SetEditor(QtWidgets.QDialog):
    def __init__(self, punches, dies, spec=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Tool Set")
        form = QtWidgets.QFormLayout(self)

        self.e_id = QtWidgets.QLineEdit(spec.id if spec else "")
        self.e_name = QtWidgets.QLineEdit(spec.name if spec else "")

        self.cb_p = QtWidgets.QComboBox()
        for p in punches:
            self.cb_p.addItem("{} — {} ({}°)".format(p.id, p.name, p.angle), p.id)
        if spec:
            i = self.cb_p.findData(spec.punch_id)
            if i >= 0:
                self.cb_p.setCurrentIndex(i)

        self.cb_d = QtWidgets.QComboBox()
        for d in dies:
            self.cb_d.addItem("{} — {} (V={}mm)".format(d.id, d.name, d.opening), d.id)
        if spec:
            i = self.cb_d.findData(spec.die_id)
            if i >= 0:
                self.cb_d.setCurrentIndex(i)

        self.e_desc = QtWidgets.QLineEdit(spec.description if spec else "")

        form.addRow("ID:", self.e_id)
        form.addRow("Name:", self.e_name)
        form.addRow("Punch:", self.cb_p)
        form.addRow("Die:", self.cb_d)
        form.addRow("Description:", self.e_desc)

        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def result_spec(self):
        return ToolSetSpec(
            id=self.e_id.text().strip() or self.e_name.text().strip(),
            name=self.e_name.text().strip() or self.e_id.text().strip(),
            punch_id=self.cb_p.currentData(),
            die_id=self.cb_d.currentData(),
            description=self.e_desc.text().strip(),
        )