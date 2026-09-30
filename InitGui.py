# FreeCAD InitGui.py for BendSeq Workbench
# This file is executed when FreeCAD GUI starts up.

import FreeCAD
import FreeCADGui

class BendSeqWorkbench (FreeCADGui.Workbench):
    """
    BendSeq Workbench for FreeCAD.
    Implements Backward Planning for sheet metal bending sequence optimization.
    """
    MenuText = "BendSeq Planner"
    ToolTip = "Backward Planning Sheet Metal Bending Sequence Planner"
    Icon = """
    /* XPM */
    static char * bendseq_xpm[] = {
    "16 16 3 1",
    " 	c None",
    ".	c #00557F",
    "+	c #00AAFF",
    "                ",
    "  ............  ",
    "  .++++++++++.  ",
    "  .++......++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++.    .++.  ",
    "  .++++++++++.  ",
    "  ............  ",
    "                ",
    "                "};
    """

    def Initialize(self):
        """Initialize workbench commands, toolbars and menus."""
        import BendSeqWorkbench
        self.appendToolbar("BendSeq Tools", ["BendSeq_RunPlanner", "BendSeq_ShowKinematics"])
        self.appendMenu("BendSeq", ["BendSeq_RunPlanner", "BendSeq_ShowKinematics"])

    def GetClassName(self):
        return "Gui::PythonWorkbench"

FreeCADGui.addWorkbench(BendSeqWorkbench())
