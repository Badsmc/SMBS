"""
bend_sequence_view.py - Flat Pattern Sequence TaskPanel

Displays a table of the final bend sequence and attempts to map
Bend IDs to the flat pattern bend lines.
"""

import json
import os
from typing import Dict, List

try:
    import FreeCAD
    import FreeCADGui
    from PySide import QtGui, QtCore
except ImportError:
    try:
        from PySide2 import QtWidgets as QtGui
        from PySide2 import QtCore
    except ImportError:
        pass

class BendSequenceViewPanel:
    """
    Task Panel for displaying the computed Bend Sequence.
    Shows a table of operations.
    """
    def __init__(self, json_file: str, target_obj):
        self.json_file = json_file
        self.target_obj = target_obj
        self.form = QtGui.QWidget()
        self.layout = QtGui.QVBoxLayout(self.form)
        self.sequence_data = self._load_data()
        
        self._setup_ui()
        self._populate_table()
        
    def _load_data(self) -> List[Dict]:
        if not os.path.exists(self.json_file):
            return []
        try:
            with open(self.json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                # Handle both 'steps' (newer) and 'sequence' (older) keys
                if "steps" in data:
                    return data["steps"]
                elif "sequence" in data:
                    return data["sequence"]
                return []
        except Exception as e:
            FreeCAD.Console.PrintError(f"Error loading JSON: {e}\n")
            return []

    def _setup_ui(self):
        # Title
        title = QtGui.QLabel("<b>Bend Sequence Operations</b>")
        self.layout.addWidget(title)
        
        # Diagnostic warning
        warning_lbl = QtGui.QLabel(
            "<font color='orange'><b>Внимание:</b> Визуализация номеров на 3D "
            "развертке отключена. Невозможно достоверно сопоставить линии "
            "эскиза SheetMetal с Bend ID.</font>"
        )
        warning_lbl.setWordWrap(True)
        self.layout.addWidget(warning_lbl)

        # Table
        self.table = QtGui.QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Step", "Bend ID", "Angle", "Dir", "Length (mm)"])
        self.table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.layout.addWidget(self.table)
        
    def _populate_table(self):
        self.table.setRowCount(len(self.sequence_data))
        for row, step in enumerate(self.sequence_data):
            # Read step details safely
            step_num = step.get("step", step.get("step_number", row + 1))
            bend_id = step.get("bend_id", "Unknown")
            angle = step.get("angle_deg", step.get("angle", ""))
            
            # Extract direction and length
            direction = "Up"
            length = ""
            if "axis" in step:
                # Approximation of direction for UI purposes
                direction = "Up" if step.get("bend_sign", 1) > 0 else "Down"
            if "length_mm" in step:
                length = f"{step['length_mm']:.1f}"
            elif "length" in step:
                length = f"{step['length']:.1f}"

            self.table.setItem(row, 0, QtGui.QTableWidgetItem(str(step_num)))
            self.table.setItem(row, 1, QtGui.QTableWidgetItem(str(bend_id)))
            self.table.setItem(row, 2, QtGui.QTableWidgetItem(f"{angle}°"))
            self.table.setItem(row, 3, QtGui.QTableWidgetItem(direction))
            self.table.setItem(row, 4, QtGui.QTableWidgetItem(length))

    def accept(self):
        FreeCADGui.Control.closeDialog()
        return True

    def reject(self):
        FreeCADGui.Control.closeDialog()
        return True

def show_sequence_view():
    """Command execution entry point."""
    doc = FreeCAD.ActiveDocument
    if doc is None:
        FreeCAD.Console.PrintError("BendSeq: No active document.\n")
        return

    sel = FreeCADGui.Selection.getSelection()
    target_obj = sel[0] if sel else None

    # Find json file
    if doc and getattr(doc, 'FileName', '') and os.path.isabs(doc.FileName):
        doc_dir = os.path.dirname(doc.FileName)
        json_file = os.path.join(doc_dir, "part_bendseq.json")
    else:
        json_file = os.path.expanduser("~/part_bendseq.json")

    if not os.path.exists(json_file):
        FreeCAD.Console.PrintError(f"BendSeq: Sequence file not found: {json_file}\n")
        return

    panel = BendSequenceViewPanel(json_file, target_obj)
    FreeCADGui.Control.showDialog(panel)
