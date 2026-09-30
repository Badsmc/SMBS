# FreeCAD InitGui.py for BendSeq Workbench
# This file is executed when FreeCAD GUI starts up.

import os
import sys

try:
    import FreeCAD
    import FreeCADGui
except ImportError:
    pass

class BendSeqWorkbench (FreeCADGui.Workbench):
    """
    BendSeq Workbench for FreeCAD.
    Implements Backward Planning for sheet metal bending sequence optimization.
    """
    MenuText = "BendSeq Planner"
    ToolTip = "Backward Planning Sheet Metal Bending Sequence Planner"

    def Initialize(self):
        """Initialize workbench commands, toolbars and menus."""
        import BendSeqWorkbench
        self.appendToolbar("BendSeq Tools", ["BendSeq_RunPlanner", "BendSeq_ShowKinematics"])
        self.appendMenu("BendSeq", ["BendSeq_RunPlanner", "BendSeq_ShowKinematics"])

    def GetClassName(self):
        return "Gui::PythonWorkbench"

# Attach icon after class definition using a safe path evaluation
try:
    _app_data = FreeCAD.getUserAppDataDir()
    _icon_path = os.path.join(_app_data, "Mod", "BendSeq", "Icons", "workbench.svg")
    if os.path.exists(_icon_path):
        BendSeqWorkbench.Icon = _icon_path
    else:
        # Fallback if installed in another location
        _alt_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Icons", "workbench.svg")
        if os.path.exists(_alt_path):
            BendSeqWorkbench.Icon = _alt_path
except Exception:
    pass

FreeCADGui.addWorkbench(BendSeqWorkbench())
