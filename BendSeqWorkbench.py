"""
BendSeqWorkbench.py - GUI Commands & Workbench Definition for FreeCAD
"""

import sys
import os

try:
    import FreeCAD
    import FreeCADGui
    HAS_FREECAD_GUI = True
except ImportError:
    HAS_FREECAD_GUI = False

# Ensure module path is accessible
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

if HAS_FREECAD_GUI:
    XPM_PLANNER = """/* XPM */
static char * xpm_planner[] = {
"16 16 3 1",
" 	c None",
".	c #0088FF",
"+	c #004488",
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
"  .++++++++++.  ",
"  ............  ",
"                ",
"                "};"""

    XPM_KINEMATICS = """/* XPM */
static char * xpm_kinematics[] = {
"16 16 3 1",
" 	c None",
".	c #FF8800",
"+	c #884400",
"                ",
"  ............  ",
"  .++++++++++.  ",
"  .++++++++++.  ",
"  .++......++.  ",
"  .++.    .++.  ",
"  .++.    .++.  ",
"  .++.    .++.  ",
"  .++++++++++.  ",
"  .++++++++++.  ",
"  ............  ",
"                ",
"                ",
"                "};"""

    class CommandRunPlanner:
        """FreeCAD GUI Command: Run BendSeq Backward Planner on active object."""

        def GetResources(self):
            return {
                'Pixmap': XPM_PLANNER,
                'MenuText': 'Run Backward Planner',
                'ToolTip': 'Executes backward search (bent -> flat) to find collision-free bend sequence'
            }

        def IsActive(self):
            doc = FreeCAD.ActiveDocument
            if doc is None or FreeCADGui.Selection.getSelection() == []:
                return False
            return True

        def Activated(self):
            from Core.orchestrator import BendSeqOrchestrator
            sel = FreeCADGui.Selection.getSelection()
            if not sel:
                FreeCAD.Console.PrintError("BendSeq: Please select a 3D sheet metal object.\n")
                return
            
            target_obj = sel[0]
            FreeCAD.Console.PrintMessage(f"BendSeq: Starting planning for '{target_obj.Label}'...\n")

            orchestrator = BendSeqOrchestrator()
            result = orchestrator.run_planning(target_obj.Shape, algorithm="astar")

            if result and result.get("success"):
                FreeCAD.Console.PrintMessage(
                    f"BendSeq: Sequence successfully solved in {result['planning_time']:.2f}s! "
                    f"Steps: {len(result['sequence'])}\n"
                )
                output_file = result.get("bendseq_file", "part_bendseq.json")
                FreeCAD.Console.PrintMessage(f"BendSeq file written to: {output_file}\n")
            else:
                FreeCAD.Console.PrintError(f"BendSeq: Planning failed. Error: {result.get('error', 'Unknown')}\n")

    class CommandShowKinematics:
        """FreeCAD GUI Command: Visualize Machine Kinematics and Die setup."""

        def GetResources(self):
            return {
                'Pixmap': XPM_KINEMATICS,
                'MenuText': 'Show Machine Kinematics',
                'ToolTip': 'Display punch, die, and backgauge tooling in 3D View'
            }

        def IsActive(self):
            return FreeCAD.ActiveDocument is not None

        def Activated(self):
            from Physics.machine_kinematics import PressBrakeMachine
            doc = FreeCAD.ActiveDocument
            machine = PressBrakeMachine.default_setup()
            machine.add_to_doc(doc)
            FreeCAD.Console.PrintMessage("BendSeq: Press brake tooling solids added to document.\n")

    FreeCADGui.addCommand("BendSeq_RunPlanner", CommandRunPlanner())
    FreeCADGui.addCommand("BendSeq_ShowKinematics", CommandShowKinematics())
