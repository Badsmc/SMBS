"""
bend_sequence_view.py - Flat Pattern Sequence TaskPanel

Displays a table of the final bend sequence and mathematically matches
Bend IDs to the flat pattern bend lines from SheetMetal Unfold.
"""

import json
import os
from typing import Dict, List, Any

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

from Geometry.feature_extractor import FeatureExtractor
from Core.bend_graph import BendRecord
from Core.panel_graph import PanelGraph
from Geometry.bend_matcher import compute_flat_coordinates, match_bends_to_sketch, BendMatchStatus

class BendSequenceViewPanel:
    """
    Task Panel for displaying the computed Bend Sequence.
    Matches operations to physical sketch lines and draws numbers on the flat pattern.
    """
    def __init__(self, json_file: str, target_obj):
        self.json_file = json_file
        self.target_obj = target_obj
        self.form = QtGui.QWidget()
        self.layout = QtGui.QVBoxLayout(self.form)
        self.sequence_data = self._load_data()
        
        self.matches = {}
        if self.target_obj:
            self._compute_matching()
            self._draw_annotations()
            
        self._setup_ui()
        self._populate_table()
        
    def _load_data(self) -> List[Dict]:
        if not os.path.exists(self.json_file):
            return []
        try:
            with open(self.json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if "steps" in data:
                    return data["steps"]
                elif "sequence" in data:
                    return data["sequence"]
                return []
        except Exception as e:
            if hasattr(FreeCAD, "Console"):
                FreeCAD.Console.PrintError(f"Error loading JSON: {e}\n")
            return []

    def _compute_matching(self):
        """Extracts features, finds SheetMetal unfold sketch, and maps lines."""
        if not hasattr(FreeCAD, "ActiveDocument"):
            return
            
        doc = FreeCAD.ActiveDocument
        
        # 1. Extract geometry from target
        extractor = FeatureExtractor()
        features = extractor.extract_features(self.target_obj.Shape)
        if not features or "bends" not in features:
            return
            
        bends_map = features["bends"]
        
        # 2. Build PanelGraph
        pg = PanelGraph()
        for idx, (b_id, b_info) in enumerate(bends_map.items()):
            parent_p = b_info.get("parent_panel_id", f"PANEL_{idx:02d}")
            child_p = b_info.get("child_panel_id", f"PANEL_{(idx+1):02d}")
            pg.add_bend_connection(b_id, parent_p, child_p)
            
        # 3. Compute mathematically flattened centers
        flat_centers = compute_flat_coordinates(bends_map, pg)
        
        # 4. Find SheetMetal Unfold Sketch
        self.sketch = None
        for obj in doc.Objects:
            if hasattr(obj, "Proxy") and obj.Proxy.__class__.__name__ == "SMUnfold":
                if hasattr(obj, "UnfoldSketches") and len(obj.UnfoldSketches) > 1:
                    sketch_name = obj.UnfoldSketches[1]
                    self.sketch = doc.getObject(sketch_name)
                    break
                    
        if not self.sketch:
            FreeCAD.Console.PrintWarning("BendSeq: No SheetMetal Unfold object found. Please create one to see numbers on 3D.\n")
            return
            
        # 5. Extract lines from sketch
        sketch_lines = []
        for i, geom in enumerate(self.sketch.Geometry):
            if geom.__class__.__name__ == "LineSegment":
                sketch_lines.append({
                    "index": i,
                    "start": (geom.StartPoint.x, geom.StartPoint.y, geom.StartPoint.z),
                    "end": (geom.EndPoint.x, geom.EndPoint.y, geom.EndPoint.z),
                    "length": geom.length()
                })
                
        # 6. Match!
        self.matches = match_bends_to_sketch(flat_centers, bends_map, sketch_lines)
        
    def _draw_annotations(self):
        """Draws App::Annotation labels in the FreeCAD document for matched bends."""
        if not hasattr(FreeCAD, "ActiveDocument"):
            return
            
        doc = FreeCAD.ActiveDocument
        
        # Cleanup old group
        old_group = doc.getObject("BendSeq_Labels")
        if old_group:
            for obj in old_group.Group:
                doc.removeObject(obj.Name)
            doc.removeObject(old_group.Name)
            
        if not self.matches:
            return
            
        group = doc.addObject("App::DocumentObjectGroup", "BendSeq_Labels")
        group.Label = "Bend Sequence Labels"
        
        # Map Bend ID to Step Number from sequence_data
        step_map = {}
        for row, step in enumerate(self.sequence_data):
            bid = step.get("bend_id")
            snum = step.get("step", step.get("step_number", row + 1))
            step_map[bid] = snum
            
        for bid, match_rec in self.matches.items():
            if match_rec.match_status == BendMatchStatus.MATCHED:
                snum = step_map.get(bid, "?")
                
                # Center of the flat line in Sketch coordinates
                cx = (match_rec.flat_start_point[0] + match_rec.flat_end_point[0]) / 2.0
                cy = (match_rec.flat_start_point[1] + match_rec.flat_end_point[1]) / 2.0
                cz = (match_rec.flat_start_point[2] + match_rec.flat_end_point[2]) / 2.0
                
                # Transform to Global coordinates
                vec = FreeCAD.Vector(cx, cy, cz)
                if hasattr(self, "sketch") and self.sketch:
                    vec = self.sketch.Placement.multVec(vec)
                
                # Try to use Draft for reliable 3D text if available, otherwise fallback to Annotation
                try:
                    import Draft
                    anno = Draft.makeText([str(snum)], FreeCAD.Vector(vec.x, vec.y, vec.z))
                    anno.Label = f"Label_{bid}"
                    if hasattr(anno, "ViewObject") and anno.ViewObject:
                        anno.ViewObject.FontSize = 10.0
                        if hasattr(anno.ViewObject, "TextColor"):
                            anno.ViewObject.TextColor = (1.0, 0.0, 0.0, 0.0)
                        elif hasattr(anno.ViewObject, "LineColor"):
                            anno.ViewObject.LineColor = (1.0, 0.0, 0.0)
                except Exception:
                    anno = doc.addObject("App::Annotation", f"Label_{bid}")
                    anno.LabelText = [str(snum)]
                    anno.Position = (vec.x, vec.y, vec.z)
                    if hasattr(anno, "ViewObject") and anno.ViewObject:
                        anno.ViewObject.Visibility = True
                        if hasattr(anno.ViewObject, "FontSize"):
                            anno.ViewObject.FontSize = 10
                        if hasattr(anno.ViewObject, "TextColor"):
                            anno.ViewObject.TextColor = (1.0, 0.0, 0.0)
                        
                group.addObject(anno)
                
        doc.recompute()

    def _setup_ui(self):
        title = QtGui.QLabel("<b>Bend Sequence Operations</b>")
        self.layout.addWidget(title)
        
        if not self.matches:
            warning_lbl = QtGui.QLabel(
                "<font color='orange'><b>Внимание:</b> Развертка SheetMetal не найдена, "
                "либо сопоставление невозможно. Визуализация в 3D отключена.</font>"
            )
            warning_lbl.setWordWrap(True)
            self.layout.addWidget(warning_lbl)

        self.table = QtGui.QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["Step", "Bend ID", "Angle", "Dir", "Length", "Status"])
        self.table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.layout.addWidget(self.table)
        
    def _on_table_selection_changed(self):
        """Highlights the corresponding 3D annotation when a table row is selected."""
        if not hasattr(FreeCADGui, "Selection") or not hasattr(FreeCAD, "ActiveDocument"):
            return
            
        doc = FreeCAD.ActiveDocument
        if not doc:
            return
            
        FreeCADGui.Selection.clearSelection()
        
        selected_items = self.table.selectedItems()
        if not selected_items:
            return
            
        # Get row of the first selected item
        row = selected_items[0].row()
        bend_id_item = self.table.item(row, 1)
        if not bend_id_item:
            return
            
        bend_id = bend_id_item.text()
        
        # Select the annotation label in the 3D view
        anno = doc.getObject(f"Label_{bend_id}")
        if anno:
            FreeCADGui.Selection.addSelection(doc.Name, anno.Name)
            
        # Optional: Select the actual line inside the sketch
        # FreeCADGui.Selection.addSelection(doc.Name, sketch.Name, f"Edge{line_idx+1}")
        
    def _populate_table(self):
        self.table.setRowCount(len(self.sequence_data))
        for row, step in enumerate(self.sequence_data):
            step_num = step.get("step", step.get("step_number", row + 1))
            bend_id = step.get("bend_id", "Unknown")
            angle = step.get("angle_deg", step.get("angle", ""))
            
            direction = "Up" if step.get("bend_sign", 1) > 0 else "Down"
            length = f"{step.get('length_mm', step.get('length', 0)):.1f}"
            
            status = "N/A"
            if self.matches and bend_id in self.matches:
                status = self.matches[bend_id].match_status
                if status == BendMatchStatus.AMBIGUOUS:
                    status = "AMBIGUOUS"

            self.table.setItem(row, 0, QtGui.QTableWidgetItem(str(step_num)))
            self.table.setItem(row, 1, QtGui.QTableWidgetItem(str(bend_id)))
            self.table.setItem(row, 2, QtGui.QTableWidgetItem(f"{angle}°"))
            self.table.setItem(row, 3, QtGui.QTableWidgetItem(direction))
            self.table.setItem(row, 4, QtGui.QTableWidgetItem(length))
            self.table.setItem(row, 5, QtGui.QTableWidgetItem(status))

    def accept(self):
        if hasattr(FreeCADGui, "Control"):
            FreeCADGui.Control.closeDialog()
        return True

    def reject(self):
        if hasattr(FreeCADGui, "Control"):
            FreeCADGui.Control.closeDialog()
        return True

def show_sequence_view():
    doc = FreeCAD.ActiveDocument
    if doc is None:
        FreeCAD.Console.PrintError("BendSeq: No active document.\n")
        return

    sel = FreeCADGui.Selection.getSelection()
    target_obj = sel[0] if sel else None

    # If not selected correctly (e.g. they selected Pad or Bend001), try to find the final part from the SMUnfold object!
    unfold_obj = None
    for obj in doc.Objects:
        if hasattr(obj, "Proxy") and obj.Proxy.__class__.__name__ == "SMUnfold":
            unfold_obj = obj
            break
            
    if unfold_obj and hasattr(unfold_obj, "baseObject") and unfold_obj.baseObject:
        final_part = unfold_obj.baseObject[0]
        # If user selected nothing, or they selected something that has fewer bends than the final part
        target_obj = final_part

    if not target_obj:
        FreeCAD.Console.PrintError("BendSeq: Please select the 3D sheet metal part or create an Unfold first.\n")
        return

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
