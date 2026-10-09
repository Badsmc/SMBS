"""
machine_kinematics.py - Machine Geometry & Physical Tooling Profiles (SMBS Phase 4)

SPEC v1.0 Requirement:
Replaces simplistic box shapes with geometric PunchProfile and DieProfile objects:
- PunchProfile: tip_radius, tip_angle, height, width, length, gooseneck profile
- DieProfile: V-groove opening, shoulder_radius, die_angle, height, length
- PressBrakeMachine: bed, ram stroke, working envelope, backgauge limits
"""

from typing import Dict, Any, Optional, Tuple
import math

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False


def _load_shape_from_fcstd(filepath: str) -> Any:
    if not HAS_FREECAD: return None
    import os
    if not os.path.exists(filepath):
        import FreeCAD
        FreeCAD.Console.PrintError(f"BendSeq: File not found {filepath}\n")
        return None
    
    try:
        import FreeCAD
        # Use openDocument to ensure it works in GUI mode
        doc = FreeCAD.openDocument(filepath)
        if doc is None:
            FreeCAD.Console.PrintError(f"BendSeq: Failed to open {filepath}\n")
            return None
            
        shapes = [obj.Shape for obj in doc.Objects if hasattr(obj, 'Shape') and not obj.Shape.isNull()]
        solids = [s for s in shapes if hasattr(s, 'Volume') and s.Volume > 0]
        
        if not solids:
            FreeCAD.Console.PrintError(f"BendSeq: No solids found in {filepath}\n")
            FreeCAD.closeDocument(doc.Name)
            return None
            
        res = solids[0].copy()
        for s in solids[1:]:
            res = res.fuse(s.copy())
            
        FreeCAD.closeDocument(doc.Name)
        FreeCAD.Console.PrintMessage(f"BendSeq: Loaded FCStd shape from {filepath}\n")
        return res
    except Exception as e:
        import FreeCAD
        FreeCAD.Console.PrintError(f"BendSeq: Error loading FCStd {filepath}: {str(e)}\n")
        return None

class PunchProfile:
    """Represents a press brake punch upper tool."""

    def __init__(
        self,
        name: str = "PUNCH_GOOSENECK_120",
        punch_type: str = "gooseneck",
        height_mm: float = 120.0,
        width_mm: float = 30.0,
        length_mm: float = 1000.0,
        tip_radius_mm: float = 1.0,
        tip_angle_deg: float = 88.0,
        filepath: Optional[str] = None
    ):
        self.name = name
        self.punch_type = punch_type
        self.height_mm = height_mm
        self.width_mm = width_mm
        self.length_mm = length_mm
        self.tip_radius_mm = tip_radius_mm
        self.tip_angle_deg = tip_angle_deg
        self.filepath = filepath
        self.topo_shape = self._build_topo_shape()

    def _build_topo_shape(self) -> Any:
        if not HAS_FREECAD:
            return None
        if self.filepath:
            shape = _load_shape_from_fcstd(self.filepath)
            if shape is not None:
                return shape
        try:
            # Create Punch blade solid positioned at Top Dead Center (TDC) open daylight height (150mm above die)
            punch_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
            punch_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, 150.0))
            return punch_box
        except Exception:
            return None


class DieProfile:
    """Represents a press brake V-die lower tool."""

    def __init__(
        self,
        name: str = "DIE_V12",
        v_width_mm: float = 12.0,
        height_mm: float = 80.0,
        width_mm: float = 50.0,
        length_mm: float = 1000.0,
        v_angle_deg: float = 88.0,
        shoulder_radius_mm: float = 1.5,
        filepath: Optional[str] = None
    ):
        self.name = name
        self.v_width_mm = v_width_mm
        self.height_mm = height_mm
        self.width_mm = width_mm
        self.length_mm = length_mm
        self.v_angle_deg = v_angle_deg
        self.shoulder_radius_mm = shoulder_radius_mm
        self.filepath = filepath
        self.topo_shape = self._build_topo_shape()

    def _build_topo_shape(self) -> Any:
        if not HAS_FREECAD:
            return None
        if self.filepath:
            shape = _load_shape_from_fcstd(self.filepath)
            if shape is not None:
                return shape
        try:
            # Create V-die block solid (top resting face at Z = 0)
            die_box = Part.makeBox(self.width_mm, self.length_mm, self.height_mm)
            die_box.translate(FreeCAD.Vector(-self.width_mm / 2.0, -self.length_mm / 2.0, -self.height_mm))
            return die_box
        except Exception:
            return None


class PressBrakeMachine:
    """
    Physical model of press brake machine environment and tooling.
    """

    def __init__(
        self,
        name: str = "Standard_PressBrake_100T",
        max_tonnage: float = 100.0,
        bed_length_mm: float = 2000.0,
        backgauge_x_max_mm: float = 800.0,
        backgauge_r_max_mm: float = 250.0,
        punch: Optional[PunchProfile] = None,
        die: Optional[DieProfile] = None,
        machine_shape: Optional[Any] = None
    ):
        self.name = name
        self.max_tonnage = max_tonnage
        self.bed_length_mm = bed_length_mm
        self.backgauge_x_max_mm = backgauge_x_max_mm
        self.backgauge_r_max_mm = backgauge_r_max_mm
        self.punch = punch or PunchProfile()
        self.die = die or DieProfile()
        self.machine_shape = machine_shape

    @classmethod
    def default_setup(cls) -> 'PressBrakeMachine':
        import os
        import glob
        # Use abspath so we stay in FreeCAD's Mod folder
        current_file = os.path.abspath(__file__)
        # current_file is .../Mod/BendSeq/Physics/machine_kinematics.py
        base_dir = os.path.dirname(os.path.dirname(current_file))
        # base_dir is .../Mod/BendSeq
        lib_dir = os.path.join(base_dir, "toollibrary")
        
        def get_first_fcstd(subfolder):
            path = os.path.join(lib_dir, subfolder, "*.FCStd")
            files = glob.glob(path)
            return files[0] if files else None
            
        punch_path = get_first_fcstd("punch")
        die_path = get_first_fcstd("die")
        machine_path = get_first_fcstd("presses")
        
        punch = PunchProfile(filepath=punch_path)
        die = DieProfile(filepath=die_path)
        machine_shape = _load_shape_from_fcstd(machine_path) if machine_path else None
        
        return cls(punch=punch, die=die, machine_shape=machine_shape)

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> 'PressBrakeMachine':
        return cls(
            name=config.get("name", "Custom_PressBrake"),
            max_tonnage=config.get("max_tonnage", 100.0),
            bed_length_mm=config.get("bed_length_mm", 2000.0)
        )

    def add_to_doc(self, doc: Any) -> None:
        """Add press brake tool solids to FreeCAD document for 3D GUI visualization."""
        if not HAS_FREECAD or doc is None:
            return

        if self.die.topo_shape:
            die_obj = doc.addObject("Part::Feature", "PressBrake_Die")
            die_obj.Shape = self.die.topo_shape
            die_obj.ViewObject.ShapeColor = (0.2, 0.2, 0.2)

        if self.punch.topo_shape:
            punch_obj = doc.addObject("Part::Feature", "PressBrake_Punch")
            punch_obj.Shape = self.punch.topo_shape
            punch_obj.ViewObject.ShapeColor = (0.6, 0.6, 0.6)

        doc.recompute()
