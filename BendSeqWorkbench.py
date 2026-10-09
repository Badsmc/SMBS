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

def get_module_dir():
    if '__file__' in globals() and __file__:
        return os.path.dirname(os.path.abspath(__file__))
    try:
        if 'FreeCAD' in sys.modules:
            app_data = FreeCAD.getUserAppDataDir()
            smbs_path = os.path.join(app_data, "Mod", "SMBS")
            if os.path.exists(smbs_path):
                return smbs_path
            bendseq_path = os.path.join(app_data, "Mod", "BendSeq")
            if os.path.exists(bendseq_path):
                return bendseq_path
    except Exception:
        pass
    return os.getcwd()

def get_icon_path(filename):
    path = os.path.join(get_module_dir(), "Icons", filename)
    return path if os.path.exists(path) else ""

MODULE_DIR = get_module_dir()
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

if HAS_FREECAD_GUI:
    class CommandRunPlanner:
        """FreeCAD GUI Command: Run BendSeq Backward Planner on active object."""

        def GetResources(self):
            return {
                'Pixmap': get_icon_path("planner.svg"),
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
                'Pixmap': get_icon_path("kinematics.svg"),
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

    class CommandImportTooling:
        """FreeCAD GUI Command: Import Tooling (FCStd) into Library."""
        
        def __init__(self, tool_type: str):
            self.tool_type = tool_type
            
        def GetResources(self):
            return {
                'Pixmap': get_icon_path(f"import_{self.tool_type}.svg"),
                'MenuText': f'Import {self.tool_type.capitalize()}',
                'ToolTip': f'Import an FCStd file to the SMBS Tool Library ({self.tool_type})'
            }

        def IsActive(self):
            return True

        def Activated(self):
            import shutil
            import os
            try:
                from PySide2 import QtWidgets as QtGui
            except ImportError:
                try:
                    from PySide import QtGui
                except ImportError:
                    from PySide6 import QtWidgets as QtGui
            
            filename, _ = QtGui.QFileDialog.getOpenFileName(
                None,
                f"Select {self.tool_type.capitalize()} FCStd",
                "",
                "FreeCAD Documents (*.FCStd)"
            )
            
            if not filename:
                return
                
            # Save directly inside the BendSeq module folder in FreeCAD Mod
            base_dir = get_module_dir()
            
            dest_dir = os.path.join(base_dir, "toollibrary", self.tool_type)
            os.makedirs(dest_dir, exist_ok=True)
            
            basename = os.path.basename(filename)
            dest_path = os.path.join(dest_dir, basename)
            
            shutil.copy2(filename, dest_path)
            FreeCAD.Console.PrintMessage(f"BendSeq: Imported {self.tool_type} -> {dest_path}\n")

    FreeCADGui.addCommand("BendSeq_RunPlanner", CommandRunPlanner())
    FreeCADGui.addCommand("BendSeq_ShowKinematics", CommandShowKinematics())
    FreeCADGui.addCommand("BendSeq_ImportPunch", CommandImportTooling("punch"))
    FreeCADGui.addCommand("BendSeq_ImportDie", CommandImportTooling("die"))
    FreeCADGui.addCommand("BendSeq_ImportPress", CommandImportTooling("presses"))
